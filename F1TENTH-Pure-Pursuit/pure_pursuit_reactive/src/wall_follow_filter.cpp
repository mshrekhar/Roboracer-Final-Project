#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/string.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

// ═══════════════════════════════════════════════════════════════════
//  Wall-Following Overtake Filter
//  
//  Drop-in replacement for velocity_obstacles.cpp
//  Topic chain:  PP → /drive_requested → [this filter] → /drive
//
//  Architecture:
//    1. Cartesian bounding box forward check (ignores side walls)
//    2. Side selection (which side has more room)
//    3. PD wall-following controller
//    4. Three-tier alpha blending:
//       - Normal:    smooth 0.3s ramp (opponent seen from far away)
//       - Emergency: fast ramp ~0.175s (sudden obstacle)
//       - Critical:  instant snap + hard brake (about to crash)
// ═══════════════════════════════════════════════════════════════════

enum class FilterMode {
    PASSTHROUGH,   // No obstacle — pass base command unchanged
    BLENDING_IN,   // Ramping alpha toward 1.0
    WALL_FOLLOW,   // Full wall-following authority
    BLENDING_OUT   // Ramping alpha back toward 0.0
};

class WallFollowOvertakeFilter : public rclcpp::Node {
public:
    WallFollowOvertakeFilter() : Node("wall_follow_filter_node") {
        // ─── Parameters ───
        this->declare_parameter("blend_in_time", 0.3);
        this->declare_parameter("blend_out_time", 0.5);
        this->declare_parameter("obstacle_detect_dist", 3.0);   // meters: start normal avoidance
        this->declare_parameter("emergency_time_factor", 0.35); // emergency_dist = speed * this
        this->declare_parameter("critical_dist", 0.5);          // meters: instant snap
        this->declare_parameter("forward_cone_deg", 30.0);      // (Legacy parameter, kept for compatibility)
        this->declare_parameter("min_cluster_points", 3);       // noise filter

        // Wall follow PD
        this->declare_parameter("wf_kp", 0.5);
        this->declare_parameter("wf_kd", 0.5);
        this->declare_parameter("wf_target_dist", 0.5);     // meters from wall
        this->declare_parameter("wf_min_dist", 0.25);       // emergency too-close threshold
        this->declare_parameter("wf_beam_angle_a", 45.0);   // degrees from side
        this->declare_parameter("wf_beam_angle_b", 90.0);   // degrees from side (perpendicular)
        this->declare_parameter("wf_lookahead", 1.5);       // meters ahead for projected distance

        // Speed limits
        this->declare_parameter("overtake_speed", 3.5);
        this->declare_parameter("emergency_speed", 2.0);
        this->declare_parameter("critical_speed", 1.0);
        this->declare_parameter("speed_steer_penalty", 4.0);
        this->declare_parameter("min_speed", 1.0);

        this->declare_parameter("min_lidar_dist", 0.15);    // ignore self-hits

        // ─── Subscribers ───
        drive_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive_requested", 10,
            std::bind(&WallFollowOvertakeFilter::drive_callback, this, std::placeholders::_1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&WallFollowOvertakeFilter::scan_callback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom_ekf", 10,
            std::bind(&WallFollowOvertakeFilter::odom_callback, this, std::placeholders::_1));

        // ─── Publisher ───
        safe_drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive", 10);
        
        // Debug: publish current mode as string
        mode_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/overtake_filter/mode", 10);

        RCLCPP_INFO(this->get_logger(), "Wall-follow overtake filter started");
    }

private:
    // ─── ROS interfaces ───
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr safe_drive_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;

    // ─── Cached sensor data ───
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
    double current_speed_ = 0.0;

    // ─── Filter state ───
    FilterMode mode_ = FilterMode::PASSTHROUGH;
    double alpha_ = 0.0;            // 0 = pure base, 1 = pure wall follow
    double prev_wf_error_ = 0.0;    // PD derivative term
    std::string current_side_ = "right";
    FilterMode prev_mode_ = FilterMode::PASSTHROUGH;  // for logging transitions

    // ═══════════════════════════════════════════════════════════════
    //  Callbacks
    // ═══════════════════════════════════════════════════════════════

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        last_scan_ = msg;
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_speed_ = std::sqrt(
            std::pow(msg->twist.twist.linear.x, 2) +
            std::pow(msg->twist.twist.linear.y, 2));
    }

    // ═══════════════════════════════════════════════════════════════
    //  Main filter — runs every time PP publishes a command
    // ═══════════════════════════════════════════════════════════════

    void drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
        if (!last_scan_) {
            safe_drive_pub_->publish(*msg);
            return;
        }

        double base_speed = msg->drive.speed;
        double base_steer = msg->drive.steering_angle;

        // ─── 1. Forward obstacle check (Cartesian bounding box) ───
        double front_dist = get_forward_min_range();

        // ─── 2. Compute thresholds based on current speed ───
        double emergency_time_factor = this->get_parameter("emergency_time_factor").as_double();
        double critical_dist = this->get_parameter("critical_dist").as_double();
        double obstacle_detect_dist = this->get_parameter("obstacle_detect_dist").as_double();

        double emergency_dist = std::max(0.8, current_speed_ * emergency_time_factor);
        bool obstacle_in_path = (front_dist < obstacle_detect_dist);

        // ─── 3. State machine transitions ───
        update_mode(obstacle_in_path, front_dist);

        // ─── 4. Update alpha with three-tier reactivity ───
        update_alpha(front_dist, emergency_dist, critical_dist);

        // ─── 5. Compute wall-follow command if needed ───
        double wf_speed = base_speed;
        double wf_steer = base_steer;

        if (alpha_ > 0.001) {
            current_side_ = pick_side();
            compute_wall_follow(wf_speed, wf_steer);
        }

        // ─── 6. Blend ───
        double final_speed = (1.0 - alpha_) * base_speed + alpha_ * wf_speed;
        double final_steer = (1.0 - alpha_) * base_steer + alpha_ * wf_steer;

        // ─── 7. Speed management by proximity ───
        double emergency_speed = this->get_parameter("emergency_speed").as_double();
        double critical_speed = this->get_parameter("critical_speed").as_double();
        double overtake_speed = this->get_parameter("overtake_speed").as_double();

        if (front_dist < critical_dist) {
            final_speed = std::min(final_speed, critical_speed);
        } else if (front_dist < emergency_dist) {
            final_speed = std::min(final_speed, emergency_speed);
        } else if (alpha_ > 0.1) {
            final_speed = std::min(final_speed, overtake_speed);
        }

        // ─── 8. Publish ───
        ackermann_msgs::msg::AckermannDriveStamped drive_out;
        drive_out.header.stamp = this->get_clock()->now();
        drive_out.header.frame_id = "base_link";
        drive_out.drive.speed = final_speed;
        drive_out.drive.steering_angle = final_steer;
        safe_drive_pub_->publish(drive_out);

        // Debug mode publish
        publish_mode();
    }

    // ═══════════════════════════════════════════════════════════════
    //  Forward obstacle detection — Cartesian Bounding Box ("Tube")
    // ═══════════════════════════════════════════════════════════════

    double get_forward_min_range() {
        if (!last_scan_) return std::numeric_limits<double>::infinity();

        const auto &ranges = last_scan_->ranges;
        int num_beams = static_cast<int>(ranges.size());
        double angle_min = last_scan_->angle_min;
        double angle_inc = last_scan_->angle_increment;
        double min_lidar = this->get_parameter("min_lidar_dist").as_double();

        // 0.6m gives a safe clearance on both sides of a standard 1/10th scale car.
        double track_width = 0.6; 
        double min_x = std::numeric_limits<double>::infinity();
        
        int min_cluster_pts = this->get_parameter("min_cluster_points").as_int();
        double close_threshold = this->get_parameter("obstacle_detect_dist").as_double();
        int close_count = 0;

        for (int i = 0; i < num_beams; ++i) {
            double angle = angle_min + i * angle_inc;

            // Ignore anything behind the car or totally sideways
            if (std::abs(angle) > M_PI / 2.0) continue;

            double r = ranges[i];
            if (!std::isfinite(r) || r < min_lidar) continue;

            // Convert polar to Cartesian relative to base_link
            double x = r * std::cos(angle);
            double y = r * std::sin(angle);

            // Is the point inside our forward driving tube?
            if (x > 0.0 && std::abs(y) < (track_width / 2.0)) {
                if (x < close_threshold) close_count++;
                min_x = std::min(min_x, x);
            }
        }

        // If fewer than min_cluster_points beams see something in the tube, it's noise
        if (close_count < min_cluster_pts) {
            return std::numeric_limits<double>::infinity();
        }

        return min_x;
    }

    // ═══════════════════════════════════════════════════════════════
    //  Side selection — which wall to follow
    // ═══════════════════════════════════════════════════════════════

    std::string pick_side() {
        if (!last_scan_) return "right";

        const auto &ranges = last_scan_->ranges;
        int n = static_cast<int>(ranges.size());
        double min_lidar = this->get_parameter("min_lidar_dist").as_double();

        // Left side: beams roughly 45-135 deg (positive angles)
        int left_start = static_cast<int>(n * 0.6);
        int left_end = static_cast<int>(n * 0.85);
        double left_sum = 0.0;
        int left_count = 0;

        for (int i = left_start; i < left_end && i < n; ++i) {
            if (std::isfinite(ranges[i]) && ranges[i] > min_lidar) {
                left_sum += ranges[i];
                left_count++;
            }
        }

        // Right side: beams roughly -135 to -45 deg (negative angles)
        int right_start = static_cast<int>(n * 0.15);
        int right_end = static_cast<int>(n * 0.4);
        double right_sum = 0.0;
        int right_count = 0;

        for (int i = right_start; i < right_end && i < n; ++i) {
            if (std::isfinite(ranges[i]) && ranges[i] > min_lidar) {
                right_sum += ranges[i];
                right_count++;
            }
        }

        double left_avg = (left_count > 0) ? left_sum / left_count : 0.0;
        double right_avg = (right_count > 0) ? right_sum / right_count : 0.0;

        return (left_avg > right_avg) ? "left" : "right";
    }

    // ═══════════════════════════════════════════════════════════════
    //  PD wall-following controller
    // ═══════════════════════════════════════════════════════════════

    void compute_wall_follow(double &speed_out, double &steer_out) {
        if (!last_scan_) return;

        double kp = this->get_parameter("wf_kp").as_double();
        double kd = this->get_parameter("wf_kd").as_double();
        double target_dist = this->get_parameter("wf_target_dist").as_double();
        double min_dist = this->get_parameter("wf_min_dist").as_double();
        double beam_a_deg = this->get_parameter("wf_beam_angle_a").as_double();
        double beam_b_deg = this->get_parameter("wf_beam_angle_b").as_double();
        double lookahead = this->get_parameter("wf_lookahead").as_double();
        double overtake_speed = this->get_parameter("overtake_speed").as_double();
        double speed_penalty = this->get_parameter("speed_steer_penalty").as_double();
        double min_speed = this->get_parameter("min_speed").as_double();

        // ─── Get two beam distances for wall distance calculation ───
        double angle_a_rad, angle_b_rad;
        if (current_side_ == "right") {
            angle_a_rad = -beam_a_deg * M_PI / 180.0;
            angle_b_rad = -beam_b_deg * M_PI / 180.0;
        } else {
            angle_a_rad = beam_a_deg * M_PI / 180.0;
            angle_b_rad = beam_b_deg * M_PI / 180.0;
        }

        double dist_a = get_range_at_angle(angle_a_rad);
        double dist_b = get_range_at_angle(angle_b_rad);

        if (dist_a <= 0.0 || dist_b <= 0.0 || 
            !std::isfinite(dist_a) || !std::isfinite(dist_b)) {
            // Bad data — output safe defaults
            speed_out = min_speed;
            steer_out = 0.0;
            return;
        }

        // ─── Two-beam wall distance formula ───
        double theta = std::abs(beam_a_deg - beam_b_deg) * M_PI / 180.0;
        double alpha_angle = std::atan2(
            dist_a * std::cos(theta) - dist_b,
            dist_a * std::sin(theta));

        // Perpendicular distance to wall
        double wall_dist = dist_b * std::cos(alpha_angle);

        // Projected distance at lookahead (accounts for wall angle)
        double wall_dist_ahead = wall_dist + lookahead * std::sin(alpha_angle);

        // ─── PD control ───
        double error = target_dist - wall_dist_ahead;

        // Flip sign for left wall
        if (current_side_ == "left") {
            error = -error;
        }

        if (std::abs(error) < 0.03) {
            error = 0.0;
        }

        double d_error = error - prev_wf_error_;
        prev_wf_error_ = error;

        double steer = kp * error + kd * d_error;
        steer = std::clamp(steer, -0.4189, 0.4189);

        // ─── Speed with steering penalty ───
        double target_speed = overtake_speed - speed_penalty * std::abs(steer);
        target_speed = std::max(target_speed, min_speed);

        // ─── Emergency: too close to wall ───
        if (wall_dist < min_dist) {
            if (current_side_ == "right") {
                steer = std::min(steer, -0.15);  // steer left away from right wall
            } else {
                steer = std::max(steer, 0.15);   // steer right away from left wall
            }
            target_speed = min_speed;
        }

        speed_out = target_speed;
        steer_out = steer;
    }

    double get_range_at_angle(double angle_rad) {
        if (!last_scan_) return -1.0;

        const auto &ranges = last_scan_->ranges;
        int num_beams = static_cast<int>(ranges.size());
        double angle_min = last_scan_->angle_min;
        double angle_inc = last_scan_->angle_increment;
        double min_lidar = this->get_parameter("min_lidar_dist").as_double();

        int idx = static_cast<int>((angle_rad - angle_min) / angle_inc);
        idx = std::clamp(idx, 0, num_beams - 1);

        // Average over a small window for noise robustness
        int window = 3;
        int start = std::max(0, idx - window);
        int end = std::min(num_beams, idx + window + 1);

        double sum = 0.0;
        int count = 0;
        for (int i = start; i < end; ++i) {
            double r = ranges[i];
            if (std::isfinite(r) && r > min_lidar && r < 30.0) {
                sum += r;
                count++;
            }
        }

        return (count > 0) ? sum / count : -1.0;
    }

    // ═══════════════════════════════════════════════════════════════
    //  State machine
    // ═══════════════════════════════════════════════════════════════

    void update_mode(bool obstacle_in_path, double front_dist) {
        
        switch (mode_) {
        case FilterMode::PASSTHROUGH:
            if (obstacle_in_path) {
                mode_ = FilterMode::BLENDING_IN;
                prev_wf_error_ = 0.0;  // reset PD state on transition
            }
            break;

        case FilterMode::BLENDING_IN:
            if (alpha_ >= 0.99) {
                mode_ = FilterMode::WALL_FOLLOW;
            } else if (!obstacle_in_path) {
                mode_ = FilterMode::BLENDING_OUT;
            }
            break;

        case FilterMode::WALL_FOLLOW:
            if (!obstacle_in_path && is_front_clear()) {
                mode_ = FilterMode::BLENDING_OUT;
            }
            break;

        case FilterMode::BLENDING_OUT:
            if (alpha_ <= 0.01) {
                mode_ = FilterMode::PASSTHROUGH;
                alpha_ = 0.0;
            } else if (obstacle_in_path) {
                mode_ = FilterMode::BLENDING_IN;
            }
            break;
        }

        // Log transitions
        if (mode_ != prev_mode_) {
            RCLCPP_INFO(this->get_logger(), "MODE: %s → %s | front=%.2fm | α=%.2f | side=%s",
                mode_string(prev_mode_).c_str(),
                mode_string(mode_).c_str(),
                front_dist, alpha_, current_side_.c_str());
            prev_mode_ = mode_;
        }
    }

    void update_alpha(double front_dist, double emergency_dist, double critical_dist) {
        // dt estimate: PP typically runs at ~40Hz
        double dt = 0.025;

        if (front_dist < critical_dist) {
            // ─── CRITICAL: instant snap ───
            alpha_ = 1.0;
            mode_ = FilterMode::WALL_FOLLOW;
            RCLCPP_WARN(this->get_logger(), 
                "CRITICAL: obstacle at %.2fm — instant wall follow + brake", front_dist);
        } else if (front_dist < emergency_dist) {
            // ─── EMERGENCY: fast ramp (~0.175s to full) ───
            alpha_ = std::min(1.0, alpha_ + 0.15);
        } else {
            // ─── Normal blend based on mode ───
            double blend_in = this->get_parameter("blend_in_time").as_double();
            double blend_out = this->get_parameter("blend_out_time").as_double();

            switch (mode_) {
            case FilterMode::BLENDING_IN:
                alpha_ = std::min(1.0, alpha_ + dt / blend_in);
                break;
            case FilterMode::BLENDING_OUT:
                alpha_ = std::max(0.0, alpha_ - dt / blend_out);
                break;
            case FilterMode::PASSTHROUGH:
                alpha_ = 0.0;
                break;
            case FilterMode::WALL_FOLLOW:
                // alpha stays at 1.0
                break;
            }
        }
    }

    bool is_front_clear() {
        // Check if forward tube is clear (obstacle has passed)
        double front_dist = get_forward_min_range();
        return (front_dist > 2.0 || !std::isfinite(front_dist));
    }

    // ═══════════════════════════════════════════════════════════════
    //  Utilities
    // ═══════════════════════════════════════════════════════════════

    std::string mode_string(FilterMode m) {
        switch (m) {
            case FilterMode::PASSTHROUGH:  return "PASSTHROUGH";
            case FilterMode::BLENDING_IN:  return "BLENDING_IN";
            case FilterMode::WALL_FOLLOW:  return "WALL_FOLLOW";
            case FilterMode::BLENDING_OUT: return "BLENDING_OUT";
        }
        return "UNKNOWN";
    }

    void publish_mode() {
        std_msgs::msg::String msg;
        msg.data = mode_string(mode_) + " | alpha=" + std::to_string(alpha_).substr(0, 4) +
                   " | side=" + current_side_;
        mode_pub_->publish(msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollowOvertakeFilter>());
    rclcpp::shutdown();
    return 0;
}
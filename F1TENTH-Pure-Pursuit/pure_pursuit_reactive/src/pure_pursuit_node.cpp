#include <sstream>
#include <string>
#include <cmath>
#include <vector>
#include <fstream>
#include <algorithm>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using namespace std;

struct Waypoint {
    double x, y, yaw;
};

class PurePursuit : public rclcpp::Node
{
private:
    // ROS Communications
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_, path_pub_, base_path_pub_, tube_pub_, obs_marker_pub_;

    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // Racing Parameters
    double min_lookahead_, max_lookahead_;
    int curvature_lookahead_pts_;
    double curvature_shrink_gain_;
    double wheelbase_, max_steer_, max_speed_, min_speed_;
    string waypoint_file_;

    // Reactive & Safety Parameters
    double obs_lookahead_time_;
    double track_width_threshold_;
    double max_avoidance_width_;
    double emergency_dist_;
    double avoidance_offset_;
    double avoidance_length_;

    vector<Waypoint> base_waypoints_;    
    vector<Waypoint> active_waypoints_;  
    int last_closest_idx_{0};
    double curr_v_{0.0};
    bool emergency_brake_active_{false};

    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;

public:
    PurePursuit() : Node("pure_pursuit_node")
    {
        // Declare Parameters with defaults
        this->declare_parameter("max_speed", 3.0);
        this->declare_parameter("min_speed", 1.5);
        this->declare_parameter("obs_lookahead_time", 0.0); 
        this->declare_parameter("max_avoidance_width", 0.65);
        this->declare_parameter("track_width_threshold", 0.0);
        this->declare_parameter("emergency_dist", 0.0);
        this->declare_parameter("avoidance_offset", 0.8);
        this->declare_parameter("avoidance_length", 2.5);
        this->declare_parameter("waypoint_file", "/home/nvidia/f1tenth_ws/src/centerline.csv");
        
        // Curvature-driven lookahead parameters
        this->declare_parameter("min_lookahead", 1.0);
        this->declare_parameter("max_lookahead", 3.5);
        this->declare_parameter("curvature_lookahead_pts", 20);   // # waypoints ahead to scan curvature
        this->declare_parameter("curvature_shrink_gain", 3.0);    // bigger = more aggressive shrink
        this->declare_parameter("wheelbase", 0.33);
        this->declare_parameter("max_steer_angle", 0.4189);

        // Get Parameters
        max_speed_ = this->get_parameter("max_speed").as_double();
        min_speed_ = this->get_parameter("min_speed").as_double();
        obs_lookahead_time_ = this->get_parameter("obs_lookahead_time").as_double();
        max_avoidance_width_ = this->get_parameter("max_avoidance_width").as_double();
        track_width_threshold_ = this->get_parameter("track_width_threshold").as_double();
        emergency_dist_ = this->get_parameter("emergency_dist").as_double();
        avoidance_offset_ = this->get_parameter("avoidance_offset").as_double();
        avoidance_length_ = this->get_parameter("avoidance_length").as_double();
        waypoint_file_ = this->get_parameter("waypoint_file").as_string();
        
        min_lookahead_ = this->get_parameter("min_lookahead").as_double();
        max_lookahead_ = this->get_parameter("max_lookahead").as_double();
        curvature_lookahead_pts_ = this->get_parameter("curvature_lookahead_pts").as_int();
        curvature_shrink_gain_ = this->get_parameter("curvature_shrink_gain").as_double();
        wheelbase_ = this->get_parameter("wheelbase").as_double();
        max_steer_ = this->get_parameter("max_steer_angle").as_double();

        load_waypoints(waypoint_file_);

        // ROS Setup
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
        marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/pure_pursuit/goal_marker", 10);
        path_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/pure_pursuit/active_path", 10);
        base_path_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/pure_pursuit/base_path", 10);
        tube_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/pure_pursuit/detection_tube", 10);
        obs_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("/pure_pursuit/obstacle_center", 10);

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 10, std::bind(&PurePursuit::scan_callback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom_ekf", 10, std::bind(&PurePursuit::odom_callback, this, std::placeholders::_1));

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        control_timer_ = this->create_wall_timer(std::chrono::milliseconds(20), std::bind(&PurePursuit::control_loop, this));
        
        RCLCPP_INFO(this->get_logger(), "Reactive Pure Pursuit Initialized (curvature-driven lookahead).");
    }

    void load_waypoints(const string &filepath) {
        ifstream file(filepath);
        if (!file.is_open()) { RCLCPP_ERROR(this->get_logger(), "FAILED TO LOAD WAYPOINTS: %s", filepath.c_str()); return; }
        string line;
        while (getline(file, line)) {
            istringstream ss(line); string tok; Waypoint wp;
            if(!(getline(ss, tok, ','))) continue; wp.x = stod(tok);
            if(!(getline(ss, tok, ','))) continue; wp.y = stod(tok);
            if(!(getline(ss, tok, ','))) continue; wp.yaw = stod(tok);
            base_waypoints_.push_back(wp);
        }
        active_waypoints_ = base_waypoints_; 
        RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints.", base_waypoints_.size());
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) { curr_v_ = msg->twist.twist.linear.x; }
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) { last_scan_ = msg; }

    // Menger curvature at waypoint i using neighbors at i-5 and i+5
    double waypoint_curvature(int i) const {
        int N = active_waypoints_.size();
        if (N < 11) return 0.0;
        int prev = (i - 5 + N) % N;
        int next = (i + 5) % N;
        double ax = active_waypoints_[prev].x, ay = active_waypoints_[prev].y;
        double bx = active_waypoints_[i].x,    by = active_waypoints_[i].y;
        double cx = active_waypoints_[next].x, cy = active_waypoints_[next].y;
        double a = distance(bx, by, cx, cy);
        double b = distance(ax, ay, cx, cy);
        double c = distance(ax, ay, bx, by);
        if (a * b * c < 1e-6) return 0.0;
        double area2 = std::abs((bx - ax) * (cy - ay) - (cx - ax) * (by - ay));
        return 2.0 * area2 / (a * b * c);   // 1 / radius
    }

    // Peak curvature in next curvature_lookahead_pts_ waypoints
    double upcoming_curvature(int from_idx) const {
        int N = active_waypoints_.size();
        double kappa_max = 0.0;
        for (int k = 0; k < curvature_lookahead_pts_; ++k) {
            kappa_max = std::max(kappa_max, waypoint_curvature((from_idx + k) % N));
        }
        return kappa_max;
    }

    void control_loop() {
        if (base_waypoints_.empty()) return;
        geometry_msgs::msg::TransformStamped tf;
        try { tf = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero); } catch (...) { return; }

        // ── Re-read max_speed so set_parameters takes effect live ──
        max_speed_ = this->get_parameter("max_speed").as_double();

        double cx = tf.transform.translation.x, cy = tf.transform.translation.y;
        tf2::Quaternion q(tf.transform.rotation.x, tf.transform.rotation.y, tf.transform.rotation.z, tf.transform.rotation.w);
        double r, p, yaw; tf2::Matrix3x3(q).getRPY(r, p, yaw);

        // 1. Reactive Path Modification
        update_active_waypoints(cx, cy, yaw);

        // 2. Pure Pursuit Path Tracking — curvature-driven lookahead
        //    Straight (kappa ~ 0)        → ld = max_lookahead
        //    Tight U-turn (kappa ~ 0.7)  → ld shrinks toward min_lookahead
        int closest_idx = find_closest_index(cx, cy);
        double kappa = upcoming_curvature(closest_idx);

        double ld = max_lookahead_ / (1.0 + curvature_shrink_gain_ * kappa);
        ld = clamp(ld, min_lookahead_, max_lookahead_);

        int goal_idx;
        Waypoint goal = find_goal_point(cx, cy, ld, closest_idx, goal_idx);

        double dx = goal.x - cx, dy = goal.y - cy;
        double ly = -dx * sin(yaw) + dy * cos(yaw);
        double steer = clamp(atan(wheelbase_ * 2.0 * ly / (ld * ld)), -max_steer_, max_steer_);

        // 3. Final Speed Decision
        double target_speed = max_speed_ - (fabs(steer)/max_steer_) * (max_speed_ - min_speed_);
        if (emergency_brake_active_) {
            target_speed = 0.0;
        }

        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.header.stamp = this->get_clock()->now();
        drive_msg.drive.steering_angle = steer;
        drive_msg.drive.speed = target_speed;
        drive_pub_->publish(drive_msg);

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 200,
            "ld=%.2f kappa=%.3f closest=%d v=%.2f", ld, kappa, closest_idx, curr_v_);

        publish_visuals(goal, ld);
    }

    void update_active_waypoints(double cx, double cy, double yaw) {
        active_waypoints_ = base_waypoints_;
        if (!last_scan_) return;

        double det_d = max(1.2, curr_v_ * obs_lookahead_time_); 
        double tube_w = 0.6;
        
        vector<double> local_obs_y_points;
        double obs_sum_x = 0, obs_sum_y = 0;
        int count = 0;

        for (size_t i = 0; i < last_scan_->ranges.size(); ++i) {
            double r = last_scan_->ranges[i];
            if (!std::isfinite(r) || r < 0.1) continue;
            double angle = last_scan_->angle_min + i * last_scan_->angle_increment;
            double lx = r * cos(angle), ly = r * sin(angle);

            if (lx > 0.1 && lx < det_d && std::abs(ly) < (tube_w / 2.0)) {
                double mx = cx + lx * cos(yaw) - ly * sin(yaw);
                double my = cy + lx * sin(yaw) + ly * cos(yaw);
                
                bool on_track = false;
                int start = max(0, last_closest_idx_ - 10);
                int end = min((int)base_waypoints_.size(), last_closest_idx_ + 100);
                for (int j = start; j < end; ++j) {
                    if (distance(mx, my, base_waypoints_[j].x, base_waypoints_[j].y) < track_width_threshold_) {
                        on_track = true; break;
                    }
                }

                if (on_track) {
                    obs_sum_x += lx; obs_sum_y += ly; count++;
                    local_obs_y_points.push_back(ly);
                }
            }
        }

        if (count < 8) {
            if (emergency_brake_active_) {
                RCLCPP_INFO(this->get_logger(), "OBSTACLE CLEARED: Resuming path.");
                emergency_brake_active_ = false;
            } else {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "PATH CLEAR: Running at %.1f m/s", curr_v_);
            }
            publish_active_path(); 
            return; 
        }

        double avg_lx = obs_sum_x / count;
        double avg_ly = obs_sum_y / count;
        auto [min_y, max_y] = std::minmax_element(local_obs_y_points.begin(), local_obs_y_points.end());
        double obstacle_width = *max_y - *min_y;

        if (avg_lx < emergency_dist_ || obstacle_width > max_avoidance_width_) {
            emergency_brake_active_ = true;
            if (obstacle_width > max_avoidance_width_) {
                RCLCPP_WARN(this->get_logger(), "DECISION: BRAKE! Obstacle width (%.2f) exceeds max allowed.", obstacle_width);
            } else {
                RCLCPP_WARN(this->get_logger(), "DECISION: BRAKE! Obstacle detected at critical dist: %.2f", avg_lx);
            }
            return;
        }

        emergency_brake_active_ = false;
        string side = (avg_ly < 0) ? "LEFT" : "RIGHT";
        double final_shift = (avg_ly < 0) ? avoidance_offset_ : -avoidance_offset_;
        RCLCPP_INFO(this->get_logger(), "DECISION: SWERVE %s. Dist: %.2f | Width: %.2f", side.c_str(), avg_lx, obstacle_width);

        double mx = cx + avg_lx * cos(yaw) - avg_ly * sin(yaw);
        double my = cy + avg_lx * sin(yaw) + avg_ly * cos(yaw);
        publish_obstacle_marker(mx, my);

        int obs_idx = 0; double min_d = 1e9;
        for (size_t i = 0; i < base_waypoints_.size(); ++i) {
            double d = distance(mx, my, base_waypoints_[i].x, base_waypoints_[i].y);
            if (d < min_d) { min_d = d; obs_idx = i; }
        }

        for (int i = 0; i < (int)base_waypoints_.size(); ++i) {
            double d = distance(base_waypoints_[i].x, base_waypoints_[i].y, base_waypoints_[obs_idx].x, base_waypoints_[obs_idx].y);
            if (d < avoidance_length_) {
                int next = (i + 1) % base_waypoints_.size(), prev = (i == 0) ? base_waypoints_.size()-1 : i-1;
                double dx = base_waypoints_[next].x - base_waypoints_[prev].x, dy = base_waypoints_[next].y - base_waypoints_[prev].y;
                double mag = sqrt(dx*dx + dy*dy);
                double scale = cos((d / avoidance_length_) * (M_PI / 2.0));
                active_waypoints_[i].x += (-dy/mag) * final_shift * scale;
                active_waypoints_[i].y += (dx/mag) * final_shift * scale;
            }
        }
        publish_active_path();
    }

    int find_closest_index(double cx, double cy) {
        int closest = last_closest_idx_; double min_d = 1e9;
        for (int k = 0; k < 100; ++k) {
            int idx = (last_closest_idx_ + k) % active_waypoints_.size();
            double d = distance(cx, cy, active_waypoints_[idx].x, active_waypoints_[idx].y);
            if (d < min_d) { min_d = d; closest = idx; }
        }
        last_closest_idx_ = closest; return closest;
    }

    Waypoint find_goal_point(double cx, double cy, double ld, int start, int &goal_idx) {
        for (int k = 0; k < (int)active_waypoints_.size(); ++k) {
            int idx = (start + k) % active_waypoints_.size(), nxt = (idx + 1) % active_waypoints_.size();
            double d0 = distance(cx, cy, active_waypoints_[idx].x, active_waypoints_[idx].y);
            double d1 = distance(cx, cy, active_waypoints_[nxt].x, active_waypoints_[nxt].y);
            if (d0 < ld && d1 >= ld) { goal_idx = idx; return active_waypoints_[idx]; }
        }
        return active_waypoints_[(start + 10) % active_waypoints_.size()];
    }

    inline double distance(double x1, double y1, double x2, double y2) const { return sqrt(pow(x1-x2,2) + pow(y1-y2,2)); }
    inline double clamp(double v, double lo, double hi) const { return max(lo, min(hi, v)); }

    // Visualization Helpers
    void publish_visuals(const Waypoint &g, double ld) {
        visualization_msgs::msg::Marker m; m.header.frame_id = "map"; m.id = 0; m.type = 2;
        m.pose.position.x = g.x; m.pose.position.y = g.y; m.scale.x = m.scale.y = m.scale.z = 0.2;
        m.color.g = 1.0; m.color.a = 1.0; marker_pub_->publish(m);

        m.header.frame_id = "base_link"; m.type = 4; m.scale.x = 0.03; m.points.clear();
        double d = max(1.2, curr_v_ * obs_lookahead_time_);
        geometry_msgs::msg::Point p1, p2, p3, p4;
        p1.x = 0.1; p1.y = 0.3; p2.x = d; p2.y = 0.3; p3.x = d; p3.y = -0.3; p4.x = 0.1; p4.y = -0.3;
        m.points.push_back(p1); m.points.push_back(p2); m.points.push_back(p3); m.points.push_back(p4); m.points.push_back(p1);
        if (emergency_brake_active_) { m.color.r = 1.0; m.color.g = 0.0; } else { m.color.r = 1.0; m.color.g = 1.0; }
        tube_pub_->publish(m);
        
        m.header.frame_id = "map"; m.type = 4; m.scale.x = 0.02; m.color.r = 1.0; m.color.g = 1.0; m.color.b = 1.0; m.color.a = 0.3;
        m.points.clear();
        for (auto &wp : base_waypoints_) { geometry_msgs::msg::Point p; p.x = wp.x; p.y = wp.y; m.points.push_back(p); }
        base_path_pub_->publish(m);
    }

    void publish_active_path() {
        visualization_msgs::msg::Marker m; m.header.frame_id = "map"; m.type = 4; m.scale.x = 0.05; m.color.g = 1.0; m.color.a = 1.0;
        for (auto &wp : active_waypoints_) { geometry_msgs::msg::Point p; p.x = wp.x; p.y = wp.y; m.points.push_back(p); }
        path_pub_->publish(m);
    }
    void publish_obstacle_marker(double x, double y) {
        visualization_msgs::msg::Marker m; m.header.frame_id = "map"; m.type = 2; m.pose.position.x = x; m.pose.position.y = y;
        m.scale.x = m.scale.y = m.scale.z = 0.3; m.color.r = 1.0; m.color.a = 1.0; obs_marker_pub_->publish(m);
    }
};

int main(int argc, char **argv) { 
    rclcpp::init(argc, argv); 
    rclcpp::spin(std::make_shared<PurePursuit>()); 
    rclcpp::shutdown(); 
    return 0; 
}
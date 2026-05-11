#include "rclcpp/rclcpp.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

class VelocityObstacleFilter : public rclcpp::Node {
public:
    VelocityObstacleFilter() : Node("vo_filter_node") {
        this->declare_parameter("robot_radius", 0.1);
        this->declare_parameter("safety_margin", 0.15);
        this->declare_parameter("time_horizon", 1.0);
        this->declare_parameter("track_width", 1.5);
        this->declare_parameter("boundary_tolerance", 0.4); // Increased tolerance
        this->declare_parameter("min_lidar_dist", 0.15);    // IGNORE SELF-HITS (0.0m hits)

        drive_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive_requested", 10, std::bind(&VelocityObstacleFilter::drive_callback, this, std::placeholders::_1));

        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&VelocityObstacleFilter::scan_callback, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom_ekf", 10, std::bind(&VelocityObstacleFilter::odom_callback, this, std::placeholders::_1));

        safe_drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);

        last_time_ = this->now();
    }

private:
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr safe_drive_pub_;
    
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
    nav_msgs::msg::Odometry::SharedPtr last_odom_;

    bool is_in_avoidance_mode_ = false;
    double avoidance_steering_direction_ = 0.0;
    double dist_since_avoidance_ = 0.0;
    rclcpp::Time last_time_;

    const double FOV_LIMIT = 60.0 * M_PI / 180.0; 

    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) { last_scan_ = msg; }
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) { last_odom_ = msg; }

    bool is_likely_boundary(double r, double angle) const {
        if (!last_odom_) return false;
        
        double track_width = this->get_parameter("track_width").as_double();
        double tolerance = this->get_parameter("boundary_tolerance").as_double();

        // 1. SANITY CHECK: If the hit is further away than the track is wide, 
        // it's likely a wall in a turn or background noise.
        if (r > (track_width * 1.2)) return true; 

        // 2. GEOMETRIC CHECK
        double current_y = std::abs(last_odom_->pose.pose.position.y);
        double dist_to_wall = (track_width / 2.0) - current_y;
        double projected_wall_dist = dist_to_wall / std::cos(angle);
        
        return (std::abs(r - projected_wall_dist) < tolerance);
    }

    double min_range_filtered(double center_angle, double half_window) const {
        if (!last_scan_) return 0.0;
        double min_r = std::numeric_limits<double>::infinity();
        double min_allowed = this->get_parameter("min_lidar_dist").as_double();

        for (size_t i = 0; i < last_scan_->ranges.size(); ++i) {
            double angle = last_scan_->angle_min + i * last_scan_->angle_increment;
            
            if (angle < -FOV_LIMIT || angle > FOV_LIMIT) continue;

            if (std::abs(angle - center_angle) <= half_window) {
                double r = last_scan_->ranges[i];
                
                // FILTER SELF-HITS: Only look at ranges > min_allowed
                if (std::isfinite(r) && r > min_allowed) {
                    if (!is_likely_boundary(r, angle)) {
                        min_r = std::min(min_r, (double)r);
                    }
                }
            }
        }
        return min_r;
    }

    void drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
        if (!last_scan_) {
            safe_drive_pub_->publish(*msg);
            return;
        }

        double req_speed = msg->drive.speed;
        double req_steer = msg->drive.steering_angle;
        double horizon = this->get_parameter("time_horizon").as_double();

        double front_dist = min_range_filtered(0.0, 0.17); 

        // Ensure we don't trigger on infinity or self-hits
        if (front_dist > 0.05 && front_dist < (req_speed * horizon) && !is_in_avoidance_mode_) {
            double left_gap = min_range_filtered(0.5, 0.2);
            double right_gap = min_range_filtered(-0.5, 0.2);
            
            is_in_avoidance_mode_ = true;
            dist_since_avoidance_ = 0.0;
            avoidance_steering_direction_ = (left_gap > right_gap) ? 1.0 : -1.0;

            RCLCPP_INFO(this->get_logger(), 
                "MONITOR: Obstacle at %.2fm. Recommending %s steer.", 
                front_dist, (avoidance_steering_direction_ > 0 ? "LEFT" : "RIGHT"));
        }

        if (is_in_avoidance_mode_) {
            double dt = (this->now() - last_time_).seconds();
            dist_since_avoidance_ += std::abs(req_speed) * dt;
            if (dist_since_avoidance_ > 1.5) {
                is_in_avoidance_mode_ = false;
                RCLCPP_INFO(this->get_logger(), "MONITOR: Clear.");
            }
        }

        last_time_ = this->now();
        safe_drive_pub_->publish(*msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VelocityObstacleFilter>());
    rclcpp::shutdown();
    return 0;
}
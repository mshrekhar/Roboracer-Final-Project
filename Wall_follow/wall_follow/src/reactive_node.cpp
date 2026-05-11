#include "rclcpp/rclcpp.hpp"
#include <string>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <cmath>
#include <iostream>
using namespace std;
class WallFollow : public rclcpp::Node {

public:
    WallFollow() : Node("wall_follow_node")
    {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            lidarscan_topic,
            10,
            std::bind(&WallFollow::scan_callback, this, std::placeholders::_1)
        );

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/drive",
            10
        );
    }

private:
    double kp = 1.0;
    double ki = 0.0;
    double kd = 0.1;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    double servo_offset = 0.0;
    double prev_error = 0.0;
    double integral = 0.0;
    double angle_min_;
    double angle_increment_;
    int num_ranges_;

    // Topics
    std::string lidarscan_topic = "/scan";
    /// TODO: create ROS subscribers and publishers

    double get_range(float* range_data, double angle)
    {
        double some_default_value = 10.0;

        int index = (angle - angle_min_) / angle_increment_;

        if (index < 0 || index >= num_ranges_)
            return some_default_value;

        double range = range_data[index];

        if (std::isnan(range) || std::isinf(range))
            return some_default_value;

        return range;
    }


    double get_error(float* range_data, double dist)
    {
        double theta = 0.785398;
        double b = get_range(range_data, 1.5708);
        double a = get_range(range_data, theta);

        double alpha = atan (( a*cos(theta)-b)/(a*sin(theta)));
        double Dt = b*cos(alpha);
        double L = 1.0;
        double D2 = Dt + L*sin(alpha);
        double err= D2 - dist;
        return err;

    }
    void pid_control(double error, double velocity)
    {

        integral += error;
        double derivative = error - prev_error;
        double angle = 0.0;
        angle = (kp*error + ki*integral + kd*derivative);
        prev_error=error;
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        drive_msg.drive.steering_angle = angle;
        drive_msg.drive.speed = velocity;
        drive_pub_->publish(drive_msg);

    }


    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg)
    {
        num_ranges_ = scan_msg->ranges.size();
        angle_min_ = scan_msg->angle_min;
        angle_increment_ = scan_msg->angle_increment;
        float* ranges = (float*)&scan_msg->ranges[0];
        double desired_dist = 1.0;
        double error = get_error(ranges, desired_dist);
        double velocity = 0.0;
        double abs_error = fabs(error);

        if (abs_error < 0.1)
            velocity = 1.5;
        else if (abs_error < 0.3)
            velocity = 1.0;
        else
            velocity = 0.5;
        pid_control(error, velocity);
    }

};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WallFollow>());
    rclcpp::shutdown();
    return 0;
}
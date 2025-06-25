#ifndef DRONE_CONTROL_GNC_FUNCTIONS_HPP
#define DRONE_CONTROL_GNC_FUNCTIONS_HPP

#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/msg/command_code.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <chrono>
#include <string>

namespace drone_control {

// ---------- Core Functions ----------

inline bool set_mode_guided(rclcpp::Node::SharedPtr node) {
    auto client = node->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    if (!client->wait_for_service(std::chrono::seconds(5))) return false;

    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = "GUIDED";

    auto future = client->async_send_request(request);
    return rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS && future.get()->mode_sent;
}

inline bool arm_drone(rclcpp::Node::SharedPtr node) {
    auto client = node->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    if (!client->wait_for_service(std::chrono::seconds(5))) return false;

    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = true;

    auto result = client->async_send_request(req);
    return rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS && result.get()->success;
}

inline bool takeoff(rclcpp::Node::SharedPtr node, double altitude) {
    auto client = node->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
    if (!client->wait_for_service(std::chrono::seconds(5))) return false;

    auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    request->altitude = altitude;

    auto future = client->async_send_request(request);
    return rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS && future.get()->success;
}

inline bool land_drone(rclcpp::Node::SharedPtr node) {
    auto client = node->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");
    if (!client->wait_for_service(std::chrono::seconds(5))) return false;

    auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    auto future = client->async_send_request(request);
    return rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS && future.get()->success;
}

inline bool return_to_home(rclcpp::Node::SharedPtr node) {
    auto client = node->create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command");
    if (!client->wait_for_service(std::chrono::seconds(5))) return false;

    auto req = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
    req->command = mavros_msgs::msg::CommandCode::NAV_RETURN_TO_LAUNCH;
    req->confirmation = true;

    auto result = client->async_send_request(req);
    return rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS && result.get()->success;
}

// ---------- Movement Functions ----------

inline void send_velocity(rclcpp::Node::SharedPtr node, double vx, double vy, double vz, double duration_sec) {
    auto pub = node->create_publisher<geometry_msgs::msg::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
    rclcpp::Rate rate(20);

    geometry_msgs::msg::TwistStamped msg;
    msg.twist.linear.x = vx;
    msg.twist.linear.y = vy;
    msg.twist.linear.z = vz;

    auto start = node->now();
    while ((node->now() - start).seconds() < duration_sec) {
        msg.header.stamp = node->now();
        pub->publish(msg);
        rate.sleep();
    }

    // Stop
    msg.twist.linear.x = 0.0;
    msg.twist.linear.y = 0.0;
    msg.twist.linear.z = 0.0;
    pub->publish(msg);
}

inline void move_forward_by(rclcpp::Node::SharedPtr node, double speed, double meters) {
    send_velocity(node, speed, 0.0, 0.0, meters / speed);
}

inline void move_backward_by(rclcpp::Node::SharedPtr node, double speed, double meters) {
    send_velocity(node, -speed, 0.0, 0.0, meters / speed);
}

inline void move_right_by(rclcpp::Node::SharedPtr node, double speed, double meters) {
    send_velocity(node, 0.0, -speed, 0.0, meters / speed);
}

inline void move_left_by(rclcpp::Node::SharedPtr node, double speed, double meters) {
    send_velocity(node, 0.0, speed, 0.0, meters / speed);
}

inline void move_up_by(rclcpp::Node::SharedPtr node, double speed, double meters) {
    send_velocity(node, 0.0, 0.0, speed, meters / speed);
}

inline void move_down_by(rclcpp::Node::SharedPtr node, double speed, double meters) {
    send_velocity(node, 0.0, 0.0, -speed, meters / speed);
}

inline void hover(rclcpp::Node::SharedPtr node, int seconds) {
    send_velocity(node, 0.0, 0.0, 0.0, seconds);
}

inline void stop(rclcpp::Node::SharedPtr node) {
    send_velocity(node, 0.0, 0.0, 0.0, 0.5);
}

// ---------- Yaw Rotation Functions ----------

inline void rotate_drone(rclcpp::Node::SharedPtr node, const std::string &direction, double yaw_rate_deg = 30.0, double duration_sec = 1.0) {
    auto pub = node->create_publisher<geometry_msgs::msg::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
    rclcpp::Rate rate(20);

    double yaw_rate_rad = yaw_rate_deg * M_PI / 180.0;
    if (direction == "left") yaw_rate_rad = std::abs(yaw_rate_rad);
    else if (direction == "right") yaw_rate_rad = -std::abs(yaw_rate_rad);
    else yaw_rate_rad = 0.0;

    geometry_msgs::msg::TwistStamped msg;
    msg.twist.angular.z = yaw_rate_rad;

    auto start = node->now();
    while ((node->now() - start).seconds() < duration_sec) {
        msg.header.stamp = node->now();
        pub->publish(msg);
        rate.sleep();
    }

    msg.twist.angular.z = 0.0;
    pub->publish(msg);
}

} // namespace drone_control

#endif  // DRONE_CONTROL_GNC_FUNCTIONS_HPP

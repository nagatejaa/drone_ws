#ifndef DRONE_CONTROL_GNC_FUNCTIONS_HPP
#define DRONE_CONTROL_GNC_FUNCTIONS_HPP

#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <chrono>

namespace drone_control {

inline bool arm_drone(rclcpp::Node::SharedPtr node)
{
    auto client = node->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    if (!client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node->get_logger(), "Arming service not available.");
        return false;
    }

    auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    req->value = true;

    auto result = client->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS && result.get()->success) {
        RCLCPP_INFO(node->get_logger(), "Drone armed.");
        return true;
    }

    RCLCPP_ERROR(node->get_logger(), "Failed to arm drone.");
    return false;
}

inline bool takeoff(rclcpp::Node::SharedPtr node, double altitude)
{
    auto client = node->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
    if (!client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node->get_logger(), "Takeoff service not available.");
        return false;
    }

    auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
    req->min_pitch = 0.0;
    req->yaw = 0.0;
    req->latitude = 0.0;
    req->longitude = 0.0;
    req->altitude = altitude;

    auto result = client->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS && result.get()->success) {
        RCLCPP_INFO(node->get_logger(), "Takeoff command sent.");
        return true;
    }

    RCLCPP_ERROR(node->get_logger(), "Takeoff command failed.");
    return false;
}

}  // namespace drone_control

#endif  // DRONE_CONTROL_GNC_FUNCTIONS_HPP

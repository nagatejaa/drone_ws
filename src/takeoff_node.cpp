#include <rclcpp/rclcpp.hpp>
#include "drone_control/gnc_functions.hpp"
#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("takeoff_node");

    RCLCPP_INFO(node->get_logger(), "Waiting for MAVROS to connect...");
    rclcpp::sleep_for(3s);

    RCLCPP_INFO(node->get_logger(), "Setting mode to GUIDED...");
    if (!drone_control::set_mode_guided(node)) {
        RCLCPP_ERROR(node->get_logger(), "Failed to set GUIDED mode.");
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::sleep_for(1s);

    RCLCPP_INFO(node->get_logger(), "Arming the drone...");
    if (!drone_control::arm_drone(node)) {
        RCLCPP_ERROR(node->get_logger(), "Aborting: Failed to arm drone.");
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::sleep_for(2s);

    RCLCPP_INFO(node->get_logger(), "Taking off...");
    if (!drone_control::takeoff(node, 5.0)) {
        RCLCPP_ERROR(node->get_logger(), "Aborting: Takeoff failed.");
        rclcpp::shutdown();
        return 1;
    }

    RCLCPP_INFO(node->get_logger(), "Takeoff command sent successfully.");

    rclcpp::shutdown();
    return 0;
}
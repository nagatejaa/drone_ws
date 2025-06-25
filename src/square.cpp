#include <rclcpp/rclcpp.hpp>
#include "drone_control/gnc_functions.hpp"
#include <chrono>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("square");

    drone_control::set_mode_guided(node);
    drone_control::arm_drone(node);
    rclcpp::sleep_for(1s);
    RCLCPP_INFO(node->get_logger(), "Moving up by 10m...");
    drone_control::takeoff(node, 10.0);
    rclcpp::sleep_for(5s);

    // RCLCPP_INFO(node->get_logger(), "Moving forward by 10m...");
    // drone_control::move_forward_by(node, 10.0);
    // rclcpp::sleep_for(1s);

    // RCLCPP_INFO(node->get_logger(), "Moving right by 5m...");
    // drone_control::move_right_by(node, 5.0);
    // rclcpp::sleep_for(1s);

    // RCLCPP_INFO(node->get_logger(), "Moving backward by 10m...");
    // drone_control::move_backward_by(node, 10.0);
    rclcpp::sleep_for(1s);

    RCLCPP_INFO(node->get_logger(), "Moving left by 500m...");
    drone_control::move_left_by(node, 500.0, 20.0); //500m  with speed 20m/s 
    rclcpp::sleep_for(1s);

    RCLCPP_INFO(node->get_logger(), "Moving down by 2m...");
    drone_control::move_down_by(node, 2.0, 2.0);
    rclcpp::sleep_for(1s);

    RCLCPP_INFO(node->get_logger(), "Landing...");
    if (!drone_control::land_drone(node)) {
        RCLCPP_ERROR(node->get_logger(), "Landing failed.");
    }

    // Uncomment the following lines if you want to return to launch (RTL) after landing

    // rclcpp::sleep_for(2s);
    // RCLCPP_INFO(node->get_logger(), "Returning to launch (RTL)...");
    // drone_control::return_to_home(node);

    rclcpp::shutdown();
    return 0;
}


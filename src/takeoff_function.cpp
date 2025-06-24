#include <rclcpp/rclcpp.hpp>
#include "drone_control/gnc_functions.hpp"
#include <chrono>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("takeoff_function");

    rclcpp::sleep_for(std::chrono::seconds(3));

    drone_control::set_mode_guided(node);
    drone_control::arm_drone(node);
    rclcpp::sleep_for(std::chrono::seconds(2));
    drone_control::takeoff(node, 5.0);
    rclcpp::sleep_for(std::chrono::seconds(3));
    drone_control::land_drone(node);
    
    rclcpp::shutdown();
    return 0;
}

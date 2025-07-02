#include "drone_control/test_functions.hpp"
#include <chrono>
#include <rclcpp/rclcpp.hpp>
using namespace std::chrono_literals;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("test_node");
    gnc::DroneNavigator drone(node, "/drone1");

    drone.wait_for_pose();
    rclcpp::sleep_for(3s);

    RCLCPP_INFO(node->get_logger(), "Streaming initial setpoints...");
    for (int i = 0; i < 100; ++i) {
        drone.set_destination(0, 0, 3, 0);
        drone.publish_setpoint();
        rclcpp::sleep_for(50ms);
        rclcpp::spin_some(node);
    }

    RCLCPP_INFO(node->get_logger(), "Switching to OFFBOARD mode...");
    // drone.set_mode("OFFBOARD");
    rclcpp::sleep_for(2s);

    RCLCPP_INFO(node->get_logger(), "Arming the drone...");
    drone.arm();
    rclcpp::sleep_for(2s);

    std::vector<gnc_api_waypoint> waypoints = {
        {0, 0, 3, 0},
        {5, 0, 3, -90},
        {5, 5, 3, 0},
        {0, 5, 3, 90},
        {0, 0, 3, 180},
    };

    size_t i = 0;
    rclcpp::Rate rate(20.0);

    while (rclcpp::ok() && i < waypoints.size()) {
        rclcpp::spin_some(node);
        if (drone.check_waypoint_reached()) {
            auto wp = waypoints[i++];
            RCLCPP_INFO(node->get_logger(), "Going to waypoint %ld: (%.1f, %.1f, %.1f, %.1f)", i, wp.x, wp.y, wp.z, wp.psi);
            drone.set_destination(wp.x, wp.y, wp.z, wp.psi);
        }
        drone.publish_setpoint();
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}

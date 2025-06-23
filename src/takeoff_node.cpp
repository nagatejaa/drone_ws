// #include <rclcpp/rclcpp.hpp>
// #include <mavros_msgs/srv/command_tol.hpp>
// #include <mavros_msgs/srv/command_bool.hpp>
// #include <chrono>

// using namespace std::chrono_literals;

// class TakeoffNode : public rclcpp::Node
// {
// public:
//     TakeoffNode() : Node("takeoff_node")
//     {
//         arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
//         takeoff_client_ = this->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");

//         wait_for_services();

//         arm_drone();
//         takeoff_drone();
//     }

// private:
//     rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
//     rclcpp::Client<mavros_msgs::srv::CommandTOL>::SharedPtr takeoff_client_;

//     void wait_for_services()
//     {
//         RCLCPP_INFO(this->get_logger(), "Waiting for services to be available...");
//         while (!arming_client_->wait_for_service(1s) || !takeoff_client_->wait_for_service(1s)) {
//             RCLCPP_INFO(this->get_logger(), "Still waiting...");
//             if (!rclcpp::ok()) return;
//         }
//     }

//     void arm_drone()
//     {
//         auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
//         request->value = true;

//         auto result = arming_client_->async_send_request(request);
//         if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
//             rclcpp::FutureReturnCode::SUCCESS && result.get()->success) {
//             RCLCPP_INFO(this->get_logger(), "Drone armed successfully.");
//         } else {
//             RCLCPP_ERROR(this->get_logger(), "Failed to arm the drone.");
//         }

//         rclcpp::sleep_for(2s);  // Wait before takeoff
//     }

//     void takeoff_drone()
//     {
//         auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
//         request->min_pitch = 0.0;
//         request->yaw = 0.0;
//         request->latitude = 0.0;
//         request->longitude = 0.0;
//         request->altitude = 5.0;

//         auto result = takeoff_client_->async_send_request(request);
//         if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) ==
//             rclcpp::FutureReturnCode::SUCCESS && result.get()->success) {
//             RCLCPP_INFO(this->get_logger(), "Takeoff command sent successfully.");
//         } else {
//             RCLCPP_ERROR(this->get_logger(), "Failed to send takeoff command.");
//         }
//     }
// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<TakeoffNode>());
//     rclcpp::shutdown();
//     return 0;
// }
#include <rclcpp/rclcpp.hpp>
#include "drone_control/gnc_functions.hpp"
#include <chrono>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("takeoff_node");

    RCLCPP_INFO(node->get_logger(), "Waiting for MAVROS to connect...");
    rclcpp::sleep_for(std::chrono::seconds(3));

    if (!drone_control::arm_drone(node)) {
        RCLCPP_ERROR(node->get_logger(), "Aborting: Failed to arm drone.");
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::sleep_for(std::chrono::seconds(2));

    if (!drone_control::takeoff(node, 5.0)) {
        RCLCPP_ERROR(node->get_logger(), "Aborting: Failed to take off.");
        rclcpp::shutdown();
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}


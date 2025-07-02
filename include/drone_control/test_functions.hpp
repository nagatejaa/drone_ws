#pragma once

#include <rclcpp/rclcpp.hpp>
#include <chrono>  // Needed for 1s
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/msg/position_target.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <vector>

using namespace std::chrono_literals;  // <-- This fixes the 1s error


struct gnc_api_waypoint {
    double x, y, z, psi;
};

namespace gnc {

class DroneNavigator {
public:
    DroneNavigator(rclcpp::Node::SharedPtr node, const std::string& ns)
        : node_(node), namespace_(ns)
    {
        state_sub_ = node_->create_subscription<mavros_msgs::msg::State>(
            namespace_ + "/state", 10,
            [this](const mavros_msgs::msg::State::SharedPtr msg) {
                current_state_ = *msg;
            });

        setpoint_pub_ = node_->create_publisher<mavros_msgs::msg::PositionTarget>(
            namespace_ + "/setpoint_raw/local", 10);

        set_mode_client_ = node_->create_client<mavros_msgs::srv::SetMode>(
            namespace_ + "/set_mode");
        arm_client_ = node_->create_client<mavros_msgs::srv::CommandBool>(
            namespace_ + "/cmd/arming");
    }

    void wait_for_pose()
    {
        RCLCPP_INFO(node_->get_logger(), "Waiting for FCU connection...");
        rclcpp::Rate rate(1.0);
        while (rclcpp::ok() && current_state_.header.stamp.sec == 0) {
            rclcpp::spin_some(node_);
            rate.sleep();
        }
    }



    inline bool set_mode_guided(rclcpp::Node::SharedPtr node) {
        auto client = node->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
        if (!client->wait_for_service(std::chrono::seconds(5))) return false;
    
        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = "GUIDED";
    
        auto future = client->async_send_request(request);
        return rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS && future.get()->mode_sent;
    }

    // void set_mode(const std::string& mode)
    // {
    //     auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    //     request->custom_mode = mode;

    //     while (!set_mode_client_->wait_for_service(1s)) {
    //         RCLCPP_WARN(node_->get_logger(), "Waiting for set_mode service...");
    //     }

    //     auto result = set_mode_client_->async_send_request(request);
    //     if (rclcpp::spin_until_future_complete(node_, result) ==
    //         rclcpp::FutureReturnCode::SUCCESS && result.get()->mode_sent) {
    //         RCLCPP_INFO(node_->get_logger(), "Mode changed to %s", mode.c_str());
    //     } else {
    //         RCLCPP_ERROR(node_->get_logger(), "Failed to set mode.");
    //     }
    // }

    void arm()
    {
        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = true;

        while (!arm_client_->wait_for_service(1s)) {
            RCLCPP_WARN(node_->get_logger(), "Waiting for arm service...");
        }

        auto result = arm_client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node_, result) ==
            rclcpp::FutureReturnCode::SUCCESS && result.get()->success) {
            RCLCPP_INFO(node_->get_logger(), "Drone armed.");
        } else {
            RCLCPP_ERROR(node_->get_logger(), "Failed to arm the drone.");
        }
    }

    void set_destination(double x, double y, double z, double yaw_deg)
    {
        last_setpoint_ = mavros_msgs::msg::PositionTarget();
        last_setpoint_.coordinate_frame = mavros_msgs::msg::PositionTarget::FRAME_LOCAL_NED;
        last_setpoint_.type_mask =
            mavros_msgs::msg::PositionTarget::IGNORE_VX |
            mavros_msgs::msg::PositionTarget::IGNORE_VY |
            mavros_msgs::msg::PositionTarget::IGNORE_VZ |
            mavros_msgs::msg::PositionTarget::IGNORE_AFX |
            mavros_msgs::msg::PositionTarget::IGNORE_AFY |
            mavros_msgs::msg::PositionTarget::IGNORE_AFZ |
            mavros_msgs::msg::PositionTarget::IGNORE_YAW_RATE;

        last_setpoint_.position.x = x;
        last_setpoint_.position.y = y;
        last_setpoint_.position.z = z;
        last_setpoint_.yaw = yaw_deg * M_PI / 180.0;
    }

    void publish_setpoint()
    {
        setpoint_pub_->publish(last_setpoint_);
    }

    bool check_waypoint_reached()
    {
        // Placeholder always returns true; replace with actual position check if needed
        return true;
    }

    mavros_msgs::msg::PositionTarget get_last_setpoint() const
    {
        return last_setpoint_;
    }

private:
    rclcpp::Node::SharedPtr node_;
    std::string namespace_;
    mavros_msgs::msg::State current_state_;
    mavros_msgs::msg::PositionTarget last_setpoint_;
    rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_sub_;
    rclcpp::Publisher<mavros_msgs::msg::PositionTarget>::SharedPtr setpoint_pub_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arm_client_;
};
}  // namespace gnc

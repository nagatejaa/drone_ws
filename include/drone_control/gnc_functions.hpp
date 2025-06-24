#ifndef DRONE_CONTROL_GNC_FUNCTIONS_HPP
#define DRONE_CONTROL_GNC_FUNCTIONS_HPP

#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <mavros_msgs/srv/command_tol.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/srv/command_long.hpp>
#include <mavros_msgs/msg/command_code.hpp>
#include <chrono>

namespace drone_control {
    inline bool set_mode_guided(rclcpp::Node::SharedPtr node)
    {
        auto client = node->create_client<mavros_msgs::srv::SetMode>("/mavros/set_mode");
    
        if (!client->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node->get_logger(), "Service /mavros/set_mode not available");
            return false;
        }
    
        auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        request->custom_mode = "GUIDED";
    
        auto future = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
            if (future.get()->mode_sent) {
                RCLCPP_INFO(node->get_logger(), "Mode set to GUIDED");
                return true;
            }
        }
    
        RCLCPP_ERROR(node->get_logger(), "Failed to set mode to GUIDED");
        return false;
    }
    
    // Arm the drone
    inline bool arm_drone(rclcpp::Node::SharedPtr node)
    {
        auto client = node->create_client<mavros_msgs::srv::CommandBool>("/mavros/cmd/arming");
    
        if (!client->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node->get_logger(), "Service /mavros/cmd/arming not available");
            return false;
        }
    
        auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        request->value = true;
    
        auto future = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
            if (future.get()->success) {
                RCLCPP_INFO(node->get_logger(), "Drone armed successfully");
                return true;
            }
        }
    
        RCLCPP_ERROR(node->get_logger(), "Failed to arm the drone");
        return false;
    }
    
    // Takeoff to given altitude (meters)
    inline bool takeoff(rclcpp::Node::SharedPtr node, double altitude)
    {
        auto client = node->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/takeoff");
    
        if (!client->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(node->get_logger(), "Service /mavros/cmd/takeoff not available");
            return false;
        }
    
        auto request = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();
        request->min_pitch = 0.0;
        request->yaw = 0.0;
        request->latitude = 0.0;   // ignored for local frame
        request->longitude = 0.0;  // ignored for local frame
        request->altitude = altitude;
    
        auto future = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
            if (future.get()->success) {
                RCLCPP_INFO(node->get_logger(), "Takeoff command successful");
                return true;
            }
        }
    
        RCLCPP_ERROR(node->get_logger(), "Failed to send takeoff command");
        return false;
    }
inline bool land_drone(rclcpp::Node::SharedPtr node)
{
    auto client = node->create_client<mavros_msgs::srv::CommandTOL>("/mavros/cmd/land");
    if (!client->wait_for_service(std::chrono::seconds(5))) return false;

    auto req = std::make_shared<mavros_msgs::srv::CommandTOL::Request>();

    auto result = client->async_send_request(req);
    return rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS && result.get()->success;
}


inline bool return_to_home(rclcpp::Node::SharedPtr node)
{
    auto client = node->create_client<mavros_msgs::srv::CommandLong>("/mavros/cmd/command");
    if (!client->wait_for_service(std::chrono::seconds(5))) {
        RCLCPP_ERROR(node->get_logger(), "Return to home service not available.");
        return false;
    }

    auto req = std::make_shared<mavros_msgs::srv::CommandLong::Request>();
    req->command = mavros_msgs::msg::CommandCode::NAV_RETURN_TO_LAUNCH;
    req->confirmation = true;
    req->param1 = 0;  // All params 0 for RTL

    auto result = client->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS && result.get()->success) {
        RCLCPP_INFO(node->get_logger(), "Return to Home (RTL) command sent.");
        return true;
    } else {
        RCLCPP_ERROR(node->get_logger(), "Failed to send RTL command.");
        return false;
    }
}

inline void send_velocity(rclcpp::Node::SharedPtr node, double vx, double vy, double vz, double duration_sec)
{
    auto pub = node->create_publisher<geometry_msgs::msg::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 10);
    rclcpp::Rate rate(20);  // 20 Hz

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

// velocity = 1.0 m/s → time = distance / velocity
constexpr double SPEED = 1.0;

inline void move_forward_by(rclcpp::Node::SharedPtr node, double meters)
{
    send_velocity(node, SPEED, 0.0, 0.0, meters / SPEED);
}

inline void move_backward_by(rclcpp::Node::SharedPtr node, double meters)
{
    send_velocity(node, -SPEED, 0.0, 0.0, meters / SPEED);
}

inline void move_right_by(rclcpp::Node::SharedPtr node, double meters)
{
    send_velocity(node, 0.0, -SPEED, 0.0, meters / SPEED);
}

inline void move_left_by(rclcpp::Node::SharedPtr node, double meters)
{
    send_velocity(node, 0.0, SPEED, 0.0, meters / SPEED);
}

inline void move_up_by(rclcpp::Node::SharedPtr node, double meters)
{
    send_velocity(node, 0.0, 0.0, SPEED, meters / SPEED);
}

inline void move_down_by(rclcpp::Node::SharedPtr node, double meters)
{
    send_velocity(node, 0.0, 0.0, -SPEED, meters / SPEED);
}
inline void hover(rclcpp::Node::SharedPtr node, int seconds)
{
    send_velocity(node, 0.0, 0.0, 0.0, seconds);
}
inline void stop(rclcpp::Node::SharedPtr node)
{
    send_velocity(node, 0.0, 0.0, 0.0, 0.5);  // small pulse to zero
}



}  // namespace drone_control

#endif  // DRONE_CONTROL_GNC_FUNCTIONS_HPP

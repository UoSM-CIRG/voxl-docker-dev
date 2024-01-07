/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version for ROS2 as an alternate to PX4 Pro Flight FastRTPS/uXRCE-DDS
 */

#include "rclcpp/rclcpp.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

mavros_msgs::msg::State current_state;

// Callback function for handling changes in the state of the vehicle
void state_cb(const mavros_msgs::msg::State::SharedPtr msg)
{
    current_state = *msg;
}

int main(int argc, char *argv[])
{
    // Initialize ROS 2 node
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("mavros_offb_node");

    // Create subscribers and clients for MAVROS topics and services
    auto state_sub = node->create_subscription<mavros_msgs::msg::State>(
        "mavros/state", 10, state_cb);
    auto arming_client = node->create_client<mavros_msgs::srv::CommandBool>(
        "mavros/cmd/arming");
    auto set_mode_client = node->create_client<mavros_msgs::srv::SetMode>(
        "mavros/set_mode");
    auto local_pos_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "mavros/setpoint_position/local", 10);

    // The setpoint publishing rate MUST be faster than 20 Hz
    rclcpp::Rate rate(20.0);

    RCLCPP_INFO(node->get_logger(), "Waiting FCU connection");

    // Wait for FCU connection
    while (rclcpp::ok() && !current_state.connected)
    {
        rclcpp::spin_some(node);
        rate.sleep();
    }

    // Initialize a setpoint position for the vehicle
    auto pose = geometry_msgs::msg::PoseStamped();
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1.5;
    pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1)); // No rotation (yaw = 0)

    RCLCPP_INFO(node->get_logger(), "Sending Setpoints");
    // Send a few setpoints before starting
    for (int i = 100; rclcpp::ok() && i > 0; --i)
    {
        local_pos_pub->publish(pose);
        rclcpp::spin_some(node);
        rate.sleep();
    }

    // Create requests for SetMode and Arming services
    auto offb_set_mode = mavros_msgs::srv::SetMode::Request();
    offb_set_mode.custom_mode = "OFFBOARD";

    auto arm_cmd = mavros_msgs::srv::CommandBool::Request();
    arm_cmd.value = true;

    auto last_request = node->now();

    while (rclcpp::ok())
    {
        // Set offboard mode
        if (current_state.mode != "OFFBOARD" &&
            (node->now() - last_request) > rclcpp::Duration(5, 0))
        {
            if (set_mode_client->wait_for_service(std::chrono::seconds(5)))
            {
                set_mode_client->async_send_request(
                    std::make_shared<mavros_msgs::srv::SetMode::Request>(offb_set_mode),
                    [node](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture response_future)
                    {
                        // Callback function
                        auto result = response_future.get();
                        if (result)
                        {
                            RCLCPP_INFO(node->get_logger(), "OFFBOARD MODE set");
                        }
                        else
                        {
                            RCLCPP_INFO(node->get_logger(), "FAILED TO SET OFFBOARD MODE!");
                        }
                    });
                last_request = node->now();
            }
        }
        else
        {
            // Arm vehicle
            if (!current_state.armed && (node->now() - last_request) > rclcpp::Duration(5, 0))
            {
                if (arming_client->wait_for_service(std::chrono::seconds(5)))
                {
                    arming_client->async_send_request(
                        std::make_shared<mavros_msgs::srv::CommandBool::Request>(arm_cmd),
                        [node](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture response_future)
                        {
                            // Callback function
                            auto result = response_future.get();
                            if (result)
                            {
                                RCLCPP_INFO(node->get_logger(), "Vehicle armed");
                            }
                            else
                            {
                                RCLCPP_ERROR(node->get_logger(), "FAILED to arm vehicle!");
                            }
                        });
                }
            }
        }

        local_pos_pub->publish(pose);
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}

/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version for ROS2 as an alternate to PX4 Pro Flight FastRTPS/uXRCE-DDS
 * @author UoSM-CIRG Janitor
 * @date 2024-05-24
 */

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/parameter.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <bits/stdc++.h> 

#include "FlightPattern.hpp"


constexpr float PUBLISH_RATE(20.0);                 // pose publishing rate
const std::string FLIGHT_MODE_OFFBOARD("OFFBOARD"); // offboard mode

// global
mavros_msgs::msg::State current_state;
nav_msgs::msg::Odometry current_odom;

// Callback function for handling changes in the state of the vehicle
void state_cb(const mavros_msgs::msg::State::SharedPtr msg)
{
    current_state = *msg;
}

void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    current_odom = *msg;
}

int main(int argc, char *argv[])
{
    // Initialize ROS 2 node
    rclcpp::init(argc, argv);
    rmw_qos_profile_t qos_profile = rmw_qos_profile_services_default;
    const auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
    const auto node = rclcpp::Node::make_shared("mavros_offb_node");

    node->declare_parameter("flight_pattern", rclcpp::ParameterValue(0)); // Default to HOVER
    rclcpp::Parameter flight_pattern_ = node->get_parameter("flight_pattern");
    const auto flight_pattern = static_cast<uosm::flight_pattern::PatternFactory::PatternType>(flight_pattern_.as_int());

    node->declare_parameter("max_iter", rclcpp::ParameterValue(2));
    const int max_iter = node->get_parameter("max_iter").as_int();
    node->declare_parameter("dt", rclcpp::ParameterValue(0.05));
    node->declare_parameter("radius", rclcpp::ParameterValue(0.80));
    node->declare_parameter("height", rclcpp::ParameterValue(1.00));
    node->declare_parameter("speed", rclcpp::ParameterValue(0.30));
    node->declare_parameter("min_speed", rclcpp::ParameterValue(0.05));
    node->declare_parameter("offset_x", rclcpp::ParameterValue(0.00));
    node->declare_parameter("offset_y", rclcpp::ParameterValue(0.00));
    node->declare_parameter("offset_z", rclcpp::ParameterValue(0.00));
    node->declare_parameter("frequency", rclcpp::ParameterValue(0.00));
    node->declare_parameter("ngram_vertices", rclcpp::ParameterValue(7));
    node->declare_parameter("ngram_step", rclcpp::ParameterValue(2));

    const uosm::flight_pattern::PatternParameters flight_params = {
        node->get_parameter("dt").as_double(),
        node->get_parameter("radius").as_double(),
        node->get_parameter("height").as_double(),
        node->get_parameter("speed").as_double(),
        node->get_parameter("min_speed").as_double(),
        node->get_parameter("offset_x").as_double(),
        node->get_parameter("offset_y").as_double(),
        node->get_parameter("offset_z").as_double(),
        node->get_parameter("frequency").as_double(),
        node->get_parameter("ngram_vertices").as_int(),
        node->get_parameter("ngram_step").as_int()};

    if(std::__gcd(flight_params.ngram_vertices, flight_params.ngram_step) != 1)
    {
        RCLCPP_ERROR(node->get_logger(), "[UoSM] ngram_vertices and ngram_step must be co-prime!");
        return 0;
    }

    RCLCPP_INFO(node->get_logger(), "[UoSM] Flight Height: %.2f", flight_params.height);

    switch (flight_pattern)
    {
    case uosm::flight_pattern::PatternFactory::PatternType::CIRCULAR:
        RCLCPP_INFO(node->get_logger(), "[UoSM] Flight Pattern: Circular");
        break;
    case uosm::flight_pattern::PatternFactory::PatternType::SPIRAL:
        RCLCPP_INFO(node->get_logger(), "[UoSM] Flight Pattern: Spiral");
        break;
    case uosm::flight_pattern::PatternFactory::PatternType::CLOUD:
        RCLCPP_INFO(node->get_logger(), "[UoSM] Flight Pattern: Cloud");
        break;
    case uosm::flight_pattern::PatternFactory::PatternType::SINE:
        RCLCPP_INFO(node->get_logger(), "[UoSM] Flight Pattern: Sine");
        break;
    case uosm::flight_pattern::PatternFactory::PatternType::NGRAM:
        RCLCPP_INFO(node->get_logger(), "[UoSM] Flight Pattern: N-Gram");
        break;
    default:
        RCLCPP_ERROR(node->get_logger(), "[UoSM] Flight Pattern Invalid!");
        return 0;
    }

    // Create subscribers/publisher and clients for MAVROS topics and services
    auto state_sub = node->create_subscription<mavros_msgs::msg::State>(
        "mavros/state", qos, state_cb);
    auto odom_sub = node->create_subscription<nav_msgs::msg::Odometry>(
        "mavros/odometry/in", qos, odom_cb);
    auto local_pos_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>(
        "mavros/setpoint_position/local", qos);
    auto arming_client = node->create_client<mavros_msgs::srv::CommandBool>(
        "mavros/cmd/arming");
    auto set_mode_client = node->create_client<mavros_msgs::srv::SetMode>(
        "mavros/set_mode");

    // The setpoint publishing rate MUST be faster than 20 Hz
    rclcpp::Rate rate(PUBLISH_RATE);
    RCLCPP_INFO(node->get_logger(), "Waiting FCU connection");

    // Wait for FCU connection
    while (rclcpp::ok() && !current_state.connected)
    {
        rclcpp::spin_some(node);
        rate.sleep();
    }

    // Initialize a setpoint position for the vehicle
    auto pose = geometry_msgs::msg::PoseStamped();
    pose.pose.position.x = flight_params.offset_x;
    pose.pose.position.y = flight_params.offset_y;
    pose.pose.position.z = flight_params.height;
    pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1)); // No rotation (yaw = 0)

    RCLCPP_INFO(node->get_logger(),
                "[UoSM] Sending Setpoints \n pos (x=%.2f m, y=%.2f m, z=%.2f m) \n rot (x=%.2f, y=%.2f, z=%.2f, w=%.2f) ",
                pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w);
    // Send a few setpoints before starting
    for (int i = 100; rclcpp::ok() && i > 0; --i)
    {
        local_pos_pub->publish(pose);
        rclcpp::spin_some(node);
        rate.sleep();
    }

    // Create requests for SetMode and Arming services
    auto offb_set_mode = mavros_msgs::srv::SetMode::Request();
    offb_set_mode.custom_mode = FLIGHT_MODE_OFFBOARD;

    auto arm_cmd = mavros_msgs::srv::CommandBool::Request();
    arm_cmd.value = true;

    bool isCompleted = false;
    bool isReady = false;
    int wait_period_sec = 2;
    auto last_request = node->now();
    auto pattern = uosm::flight_pattern::PatternFactory::createPattern(flight_pattern, flight_params);

    while (rclcpp::ok())
    {
        // Set offboard mode
        if (current_state.mode != FLIGHT_MODE_OFFBOARD &&
            (node->now() - last_request) > rclcpp::Duration(wait_period_sec, 0))
        {
            if (set_mode_client->wait_for_service(std::chrono::seconds(wait_period_sec)))
            {
                set_mode_client->async_send_request(
                    std::make_shared<mavros_msgs::srv::SetMode::Request>(offb_set_mode),
                    [node](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture response_future)
                    {
                        // Callback function
                        auto result = response_future.get();
                        if (result)
                            RCLCPP_INFO(node->get_logger(), "[UoSM] OFFBOARD mode set!");
                        else
                            RCLCPP_INFO(node->get_logger(), "[UoSM] FAILED to set OFFBOARD mode!");
                    });
                last_request = node->now();
            }
        }
        else
        {
            // Arm vehicle
            if (!current_state.armed && (node->now() - last_request) > rclcpp::Duration(wait_period_sec, 0))
            {
                if (arming_client->wait_for_service(std::chrono::seconds(wait_period_sec)))
                {
                    arming_client->async_send_request(
                        std::make_shared<mavros_msgs::srv::CommandBool::Request>(arm_cmd),
                        [node](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture response_future)
                        {
                            // Callback function
                            auto result = response_future.get();
                            if (result)
                            {
                                RCLCPP_INFO(node->get_logger(), "Vehicle ARMED!");
                            }
                            else
                            {
                                RCLCPP_ERROR(node->get_logger(), "FAILED to ARM vehicle!");
                            }
                        });
                }
                last_request = node->now();
            }
        }

        // fly predefined pattern
        if (current_state.mode == FLIGHT_MODE_OFFBOARD && current_state.armed && !isCompleted)
        {
            if (!isReady)
            {
                pattern->hover(pose);
                last_request = node->now();
                isReady = true;
            }
            // hover first for 5 sec
            if (isReady && (node->now() - last_request) > rclcpp::Duration(5, 0))
            {
                pattern->run(pose);
                isCompleted = pattern->is_done(max_iter);
            }
        }

        // return to origin if still mid air
        // @todo: use px4 automated return to home and land mode
        if (current_state.armed && isCompleted && current_odom.pose.pose.position.z >= 0.1)
        {
            pattern->return_origin(pose);
            RCLCPP_WARN(node->get_logger(), "[UoSM] Returning to Origin! Altitude = %.2f m", current_odom.pose.pose.position.z);
        }
        else if (current_state.armed && isCompleted && current_odom.pose.pose.position.z < 0.1)
        {
            arm_cmd.value = false;
            if (arming_client->wait_for_service(std::chrono::seconds(wait_period_sec)))
            {
                arming_client->async_send_request(
                    std::make_shared<mavros_msgs::srv::CommandBool::Request>(arm_cmd),
                    [node](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture response_future)
                    {
                        // Callback function
                        auto result = response_future.get();
                        if (result)
                            RCLCPP_INFO(node->get_logger(), "[UoSM] Vehicle DISARMED!");
                        else
                            RCLCPP_ERROR(node->get_logger(), "[UoSM] FAILED to DISARM vehicle!");
                    });
            }
        }

        // have to keep publishing pose
        local_pos_pub->publish(pose);
        rclcpp::spin_some(node);
        rate.sleep();

        if (!current_state.armed && isCompleted)
        {
            rclcpp::shutdown();
            return 0;
        }
    }
    return 0;
}
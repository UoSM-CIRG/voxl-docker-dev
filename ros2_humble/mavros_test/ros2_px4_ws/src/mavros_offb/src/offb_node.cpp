/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version for ROS2 as an alternate to PX4 Pro Flight FastRTPS/uXRCE-DDS
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

constexpr float PUBLISH_RATE (20.0); // pose publishing rate
constexpr float REVOLUTION(2 * M_PI); // 1 full circle


// global
mavros_msgs::msg::State current_state;
nav_msgs::msg::Odometry current_odom;
float height; // const flight height

/**
 * dt_ (Time step/Update rate)
 * theta_ (Current angle along the circular path)
 * radius_ (Radius of circle)
 * omega_ (Angular velocity)
 */
struct circular_traj
{
    double dt_ = 1/PUBLISH_RATE;
    double theta_ = 0.00f;
    double radius_ = 2.00f;
    double omega_ = 0.5f;
};

/**
 * dt_ (Time step/Update rate)
 * side_length_ (Length of one side of the square)
 * offset_ (Offset of origin)
 * speed_ (Linear speed)
 * segment_ (Current segment [0-3])
 * progress_ (Progress along the current segment [0.0 - 1.0])
 */
struct square_traj
{
    double dt_ = 1/PUBLISH_RATE;
    double side_length_ = 2.00f;
    double offset_ = 0.00f;
    double speed_ = 0.5f;
    int segment_ = 0;
    double progress_ = 0.00f;
};

enum class pattern
{
    HOVER = 0,
    CIRCULAR,
    SQUARE
};

void return_origin(geometry_msgs::msg::PoseStamped &pose)
{
    pose.pose.position.x = 0.00f;
    pose.pose.position.y = 0.00f;
    pose.pose.position.z = 0.00f;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, 0);
    pose.pose.orientation = tf2::toMsg(quat);
};

void hover_pattern(geometry_msgs::msg::PoseStamped &pose)
{
    pose.pose.position.x = 0.00f;
    pose.pose.position.y = 0.00f;
    pose.pose.position.z = height;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, 0);
    pose.pose.orientation = tf2::toMsg(quat);
};

void circular_pattern(geometry_msgs::msg::PoseStamped &pose, circular_traj &traj)
{
    auto multiplier = 1 + traj.theta_ / (REVOLUTION);
    pose.pose.position.x = traj.radius_ / multiplier * cos(traj.theta_);
    pose.pose.position.y = traj.radius_ / multiplier * sin(traj.theta_);
    pose.pose.position.z = height;

    // Calculate angle towards the middle (origin)
    double angle_towards_middle = atan2(0.0 - pose.pose.position.y, 0.0 - pose.pose.position.x);
    tf2::Quaternion quat;
    quat.setRPY(0, 0, angle_towards_middle);
    pose.pose.orientation = tf2::toMsg(quat);

    traj.theta_ = traj.theta_ + traj.omega_ * traj.dt_;
};

void square_pattern(geometry_msgs::msg::PoseStamped &pose, square_traj &traj)
{
    double target_x = 0.0;
    double target_y = 0.0;
    double target_yaw = 0.0; // in radians
    auto multiplier = 1 + static_cast<int>(traj.segment_ / 4);

    switch (traj.segment_ % 4)
    {
    case 0: // Move forward (along x-axis)
        target_x = -traj.offset_ + traj.progress_ * traj.side_length_;
        target_y = traj.offset_;
        target_yaw = 0.0;
        break;
    case 1: // Move sideways (along y-axis)
        target_x = -traj.offset_ + traj.side_length_;
        target_y = traj.offset_ - traj.progress_ * traj.side_length_;
        target_yaw = -M_PI_2; // 90 degrees
        break;
    case 2: // Move backward
        target_x = -traj.offset_ + (1.0 - traj.progress_) * traj.side_length_;
        target_y = traj.offset_ - traj.side_length_;
        target_yaw = M_PI; // 180 degrees
        break;
    case 3: // Move sideways (back to the start)
        target_x = -traj.offset_;
        target_y = traj.offset_ - (1.0 - traj.progress_) * traj.side_length_;
        target_yaw = M_PI_2; // -90 degrees
        break;
    }

    pose.pose.position.x = target_x / multiplier;
    pose.pose.position.y = target_y / multiplier;
    pose.pose.position.z = height;

    // Set yaw orientation
    tf2::Quaternion quat;
    quat.setRPY(0, 0, target_yaw);
    pose.pose.orientation = tf2::toMsg(quat);

    // Update trajectory state
    traj.progress_ += traj.speed_ * traj.dt_ / traj.side_length_;
    if (traj.progress_ >= 1.0)
    {
        traj.progress_ = 0.0;
        ++traj.segment_;
    }
};

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

    auto node = rclcpp::Node::make_shared("mavros_offb_node");
    node->declare_parameter("flight_pattern", rclcpp::ParameterValue(0)); // Default to HOVER
    rclcpp::Parameter flight_pattern_param = node->get_parameter("flight_pattern");
    const pattern flight_pattern = static_cast<pattern>(flight_pattern_param.as_int());
    node->declare_parameter("flight_height", rclcpp::ParameterValue(1.00)); // Default to HOVER
    height = node->get_parameter("flight_height").as_double();
    RCLCPP_INFO(node->get_logger(), "Flight Height: %.2f", height);

    switch (flight_pattern)
    {
    case pattern::HOVER:
        RCLCPP_INFO(node->get_logger(), "Flight Pattern: Hover");
        break;
    case pattern::CIRCULAR:
        RCLCPP_INFO(node->get_logger(), "Flight Pattern: Circular");
        break;
    case pattern::SQUARE:
        RCLCPP_INFO(node->get_logger(), "Flight Pattern: Square");
        break;
    default:
        RCLCPP_ERROR(node->get_logger(), "Flight Pattern Invalid!");
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
    pose.pose.position.x = 0.00f;
    pose.pose.position.y = 0.00f;
    pose.pose.position.z = 1.25f;
    pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1)); // No rotation (yaw = 0)

    RCLCPP_INFO(node->get_logger(),
                "Sending Setpoints \n pos (x=%.2f m, y=%.2f m, z=%.2f m) \n rot (x=%.2f, y=%.2f, z=%.2f, w=%.2f) ",
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
    offb_set_mode.custom_mode = "OFFBOARD";

    auto arm_cmd = mavros_msgs::srv::CommandBool::Request();
    arm_cmd.value = true;

    bool isCompleted = false;
    bool isReady = false;
    auto last_request = node->now();
    auto cir_traj = circular_traj{0.05, 0.0, 1.0, 0.5};
    auto squ_traj = square_traj{0.05, 2.0, 1.0, 0.5, 0, 0.0};

    while (rclcpp::ok())
    {
        // Set offboard mode
        if (current_state.mode != "OFFBOARD" &&
            (node->now() - last_request) > rclcpp::Duration(2, 0))
        {
            if (set_mode_client->wait_for_service(std::chrono::seconds(2)))
            {
                set_mode_client->async_send_request(
                    std::make_shared<mavros_msgs::srv::SetMode::Request>(offb_set_mode),
                    [node](rclcpp::Client<mavros_msgs::srv::SetMode>::SharedFuture response_future)
                    {
                        // Callback function
                        auto result = response_future.get();
                        if (result)
                        {
                            RCLCPP_INFO(node->get_logger(), "OFFBOARD mode set!");
                        }
                        else
                        {
                            RCLCPP_INFO(node->get_logger(), "FAILED to set OFFBOARD mode!");
                        }
                    });
                last_request = node->now();
            }
        }
        else
        {
            // Arm vehicle
            if (!current_state.armed && (node->now() - last_request) > rclcpp::Duration(2, 0))
            {
                if (arming_client->wait_for_service(std::chrono::seconds(2)))
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
        if (current_state.mode == "OFFBOARD" && current_state.armed && !isCompleted)
        {
            switch (flight_pattern)
            {
            case pattern::HOVER:
                // hover for 30 sec
                hover_pattern(pose);
                if ((node->now() - last_request) > rclcpp::Duration(30, 0))
                    isCompleted = true;
                break;
            case pattern::CIRCULAR:
                if (!isReady)
                {
                    hover_pattern(pose);
                    last_request = node->now();
                    isReady = true;
                }
                // hover first for 10 sec
                if (isReady && (node->now() - last_request) > rclcpp::Duration(10, 0))
                {
                    RCLCPP_WARN(node->get_logger(), "Traj theta = %.2f", cir_traj.theta_);
                    if (cir_traj.theta_ > (2 * REVOLUTION))
                        isCompleted = true;
                    else
                        circular_pattern(pose, cir_traj);
                }
                break;
            case pattern::SQUARE:
                if (!isReady)
                {
                    hover_pattern(pose);
                    last_request = node->now();
                    isReady = true;
                }
                // hover first for 10 sec
                if (isReady && (node->now() - last_request) > rclcpp::Duration(10, 0))
                {
                    RCLCPP_WARN(node->get_logger(), "Traj segment = %d", squ_traj.segment_);
                    if (squ_traj.segment_ > 8)
                        isCompleted = true;
                    else
                        square_pattern(pose, squ_traj);
                }
                break;
            default:
                isCompleted = true;
                break;
            }
        }

        // return to origin if still mid air
        if (current_state.armed && isCompleted && current_odom.pose.pose.position.z > 0.05)
        {
            return_origin(pose);
            RCLCPP_INFO(node->get_logger(), "Returning to Origin!");
            RCLCPP_WARN(node->get_logger(), "Altitude = %.2f m", current_odom.pose.pose.position.z);
        }
        else if (current_state.armed && isCompleted && current_odom.pose.pose.position.z < 0.05)
        {
            arm_cmd.value = false;
            if (arming_client->wait_for_service(std::chrono::seconds(2)))
            {
                arming_client->async_send_request(
                    std::make_shared<mavros_msgs::srv::CommandBool::Request>(arm_cmd),
                    [node](rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedFuture response_future)
                    {
                        // Callback function
                        auto result = response_future.get();
                        if (result)
                        {
                            RCLCPP_INFO(node->get_logger(), "Vehicle DISARMED!");
                        }
                        else
                        {
                            RCLCPP_ERROR(node->get_logger(), "FAILED to DISARM vehicle!");
                        }
                    });
            }
        }

        // have to keep publishing pose
        local_pos_pub->publish(pose);
        rclcpp::spin_some(node);
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
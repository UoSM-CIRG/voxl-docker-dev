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
    double omega_ = 0.1f;
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
    double speed_ = 0.1f;
    int segment_ = 0;
    double progress_ = 0.00f;
};

/**
 * dt_ (Time step/Update rate)
 * radius_ (Radius of the star)
 * offset_x_ (Center x-offset)
 * offset_y_ (Center y-offset)
 * angle_ (Current angle)
 * speed_ (Linear speed)
 * progress_ (Progress along current segment [0.0 - 1.0])
 * segment_ (Current line segment of the star [0-9]))
*/
struct star_traj
{
    double dt_ = 1/PUBLISH_RATE;
    double radius_ = 1.0;      
    double offset_x_ = 0.0;    
    double offset_y_ = 0.0;    
    double speed_ = 0.1f;
    double angle_ = 0.0;       
    double progress_ = 0.0;    
    int segment_ = 0;          
};


enum class pattern
{
    HOVER = 0,
    CIRCULAR,
    SQUARE,
    STAR,
    ANKH,
    CLOVER
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

constexpr double calculate_star_x(double radius, double angle, double offset_x = 0.0)
{
    double outer_radius = radius;
    double inner_radius = radius * 0.382; // Ratio based on the golden ratio

    bool outer_point = fmod(angle, (2 * M_PI / 5)) < M_PI / 5;
    double effective_radius = outer_point ? outer_radius : inner_radius;

    return effective_radius * cos(angle) + offset_x;
}

constexpr double calculate_star_y(double radius, double angle, double offset_y = 0.0)
{
    double outer_radius = radius;
    double inner_radius = radius * 0.382; 

    bool outer_point = fmod(angle, (2 * M_PI / 5)) < M_PI / 5;
    double effective_radius = outer_point ? outer_radius : inner_radius;

    return effective_radius * sin(angle) + offset_y;
}

void star_pattern(geometry_msgs::msg::PoseStamped &pose, star_traj &traj)
{
    // Move along a line segment
    double start_angle = traj.segment_ * 2 * M_PI / 10;
    double end_angle = start_angle + 2 * M_PI / 10;

    double target_x = calculate_star_x(traj.radius_, start_angle + traj.progress_ * (end_angle - start_angle), traj.offset_x_);
    double target_y = calculate_star_y(traj.radius_, start_angle + traj.progress_ * (end_angle - start_angle), traj.offset_y_);

    pose.pose.position.x = target_x;
    pose.pose.position.y = target_y;
    pose.pose.position.z = height;

    // Orientation: Face the center of the pentagram 
    double angle_towards_middle = atan2(traj.offset_y_ - pose.pose.position.y, traj.offset_x_ - pose.pose.position.x);
    tf2::Quaternion quat;
    quat.setRPY(0, 0, angle_towards_middle);
    pose.pose.orientation = tf2::toMsg(quat);

    // Update trajectory state
    traj.progress_ += traj.speed_ * traj.dt_;
    if (traj.progress_ >= 1.0) {
        traj.progress_ = 0.0;
        traj.segment_ = (traj.segment_ + 1) % 10; 
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
    node->declare_parameter("flight_height", rclcpp::ParameterValue(1.00)); // Default to 1m
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
    case pattern::STAR:
        RCLCPP_INFO(node->get_logger(), "Flight Pattern: Star");
        break;
    case pattern::ANKH:
        RCLCPP_INFO(node->get_logger(), "Flight Pattern: Ankh");
        break;
    case pattern::CLOVER:
        RCLCPP_INFO(node->get_logger(), "Flight Pattern: Clover");
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
    auto sta_traj = star_traj{0.05, 2.0, 0.0, 0.0, 0.5, 0.0, 0.0, 0};

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
            case pattern::STAR:
                if (!isReady)
                {
                    hover_pattern(pose);
                    last_request = node->now();
                    isReady = true;
                }
                // hover first for 10 sec
                if (isReady && (node->now() - last_request) > rclcpp::Duration(10, 0))
                {
                    RCLCPP_WARN(node->get_logger(), "Traj segment = %d", sta_traj.segment_);
                    if (sta_traj.segment_ > 9)
                        isCompleted = true;
                    else
                        star_pattern(pose, sta_traj);
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
/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version for ROS2 as an alternate to PX4 Pro Flight FastRTPS/uXRCE-DDS
 * @author UoSM-CIRG Janitor
 * @date 2024-05-24
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

#include <bits/stdc++.h>
#include <cmath>
#include "FlightPattern.hpp"

constexpr float PUBLISH_RATE(20.0);                 // pose publishing rate
constexpr float THRESHOLD_ORIGIN(0.1);              // threshold for origin
const std::string FLIGHT_MODE_OFFBOARD("OFFBOARD"); // offboard mode
const std::string FLIGHT_MODE_LAND("AUTO.LAND");    // land mode

// global
mavros_msgs::State current_state;
nav_msgs::Odometry current_odom;

// math utility
double calc2DVectorDistance(const nav_msgs::Odometry &odom)
{
    return std::sqrt(
        std::pow(odom.pose.pose.position.x, 2) +
        std::pow(odom.pose.pose.position.y, 2));
}

// Callback function for handling changes in the state of the vehicle
void state_cb(const mavros_msgs::State::ConstPtr msg)
{
    current_state = *msg;
}

void odom_cb(const nav_msgs::Odometry::ConstPtr msg)
{
    current_odom = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh("~");
    // Get the parameter
    int flight_pattern_;
    nh.param<int>("flight_pattern", flight_pattern_, 0);
    const auto flight_pattern = static_cast<uosm::flight_pattern::PatternFactory::PatternType>(flight_pattern_);

    int max_iter_;
    nh.param<int>("max_iter", max_iter_, 2);

    float dt;
    float radius;
    float height;
    float speed;
    float min_speed;
    float offset_x;
    float offset_y;
    float offset_z;
    float frequency;
    int ngram_vertices;
    int ngram_step;

    nh.param<float>("dt", dt, 0.05f);
    nh.param<float>("radius", radius, 0.80f);
    nh.param<float>("height", height, 1.00f);
    nh.param<float>("speed", speed, 0.30f);
    nh.param<float>("min_speed", min_speed, 0.05f);
    nh.param<float>("offset_x", offset_x, 0.00f);
    nh.param<float>("offset_y", offset_y, 0.00f);
    nh.param<float>("offset_z", offset_z, 0.00f);
    nh.param<float>("frequency", frequency, 0.00f);
    nh.param<int>("ngram_vertices", ngram_vertices, 7);
    nh.param<int>("ngram_step", ngram_step, 2);

    const uosm::flight_pattern::PatternParameters flight_params{
        dt, radius, height, speed, min_speed, offset_x, offset_y, offset_z, frequency, ngram_vertices, ngram_step, max_iter_};

    if (std::__gcd(flight_params.ngram_vertices, flight_params.ngram_step) != 1)
    {
        ROS_ERROR("[UoSM] ngram_vertices and ngram_step must be co-prime!");
        return 0;
    }

    switch (flight_pattern)
    {
    case uosm::flight_pattern::PatternFactory::PatternType::CIRCULAR:
        ROS_INFO("[UoSM] Flight Pattern: Circular");
        break;
    case uosm::flight_pattern::PatternFactory::PatternType::SPIRAL:
        ROS_INFO("[UoSM] Flight Pattern: Spiral");
        break;
    case uosm::flight_pattern::PatternFactory::PatternType::CLOUD:
        ROS_INFO("[UoSM] Flight Pattern: Cloud");
        break;
    case uosm::flight_pattern::PatternFactory::PatternType::SINE:
        ROS_INFO("[UoSM] Flight Pattern: Sine");
        break;
    case uosm::flight_pattern::PatternFactory::PatternType::NGRAM:
        ROS_INFO("[UoSM] Flight Pattern: N-Gram");
        break;
    default:
        ROS_ERROR("Flight Pattern Invalid!");
        return 0;
    }

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state", 10, state_cb);
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, odom_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(PUBLISH_RATE);

    // wait for FCU connection
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = flight_params.offset_x;
    pose.pose.position.y = flight_params.offset_y;
    pose.pose.position.z = flight_params.height;
    pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1)); // No rotation (yaw = 0)

    // send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = FLIGHT_MODE_OFFBOARD;

    mavros_msgs::SetMode land_set_mode;
    land_set_mode.request.custom_mode = FLIGHT_MODE_LAND;

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    bool isCompleted = false;
    bool isReady = false;
    float wait_period_sec = 2.0f;
    ros::Time last_request = ros::Time::now();
    auto pattern = uosm::flight_pattern::PatternFactory::createPattern(flight_pattern, flight_params);

    while (ros::ok())
    {
        if (current_state.mode != FLIGHT_MODE_OFFBOARD && current_state.mode != FLIGHT_MODE_LAND &&
            (ros::Time::now() - last_request > ros::Duration(wait_period_sec)))
        {
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("[UoSM] OFFBOARD mode set!");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if (!current_state.armed && !isCompleted &&
                (ros::Time::now() - last_request > ros::Duration(wait_period_sec)))
            {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle ARMED!");
                }
                last_request = ros::Time::now();
            }
        }

        // fly predefined pattern
        if (current_state.mode == FLIGHT_MODE_OFFBOARD && current_state.armed && !isCompleted)
        {
            if (!isReady)
            {
                pattern->hover(pose);
                last_request = ros::Time::now();
                isReady = true;
            }
            // hover first for 5 sec
            if (isReady && (ros::Time::now() - last_request) > ros::Duration(5.0))
            {
                pattern->run(pose);
                isCompleted = pattern->is_done(max_iter_);
            }
        }

        // return to origin XYZ defined in params and hover, then
        // use px4 automated land mode
        if (current_state.armed && isCompleted && current_state.mode == FLIGHT_MODE_OFFBOARD && current_odom.pose.pose.position.z >= 0.1 && calc2DVectorDistance(current_odom) >= THRESHOLD_ORIGIN)
        {
            pattern->hover(pose);
            ROS_WARN("[UoSM] Returning to hover at origin! x = %.2f m, y = %.2f m, z = %.2f m", current_odom.pose.pose.position.x, current_odom.pose.pose.position.y, current_odom.pose.pose.position.z);
        }
        else if (current_state.armed && isCompleted && current_state.mode == FLIGHT_MODE_OFFBOARD && current_state.mode != FLIGHT_MODE_LAND && calc2DVectorDistance(current_odom) < THRESHOLD_ORIGIN)
        {
            if (set_mode_client.call(land_set_mode) &&
                land_set_mode.response.mode_sent)
            {
                ROS_INFO("[UoSM] LAND mode set!");
            }
        }
        else if (!current_state.armed && isCompleted)
        {
            ROS_INFO("[UoSM] Complete!");
            return 0;
        }

        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
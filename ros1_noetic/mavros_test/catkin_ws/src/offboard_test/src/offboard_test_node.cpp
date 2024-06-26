/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

constexpr float PUBLISH_RATE(20.0);   // pose publishing rate
constexpr float REVOLUTION(2 * M_PI); // 1 full circle

//global
mavros_msgs::State current_state;
nav_msgs::Odometry current_odom;
float height; // flight height param

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
    STAR
};

void return_origin(geometry_msgs::PoseStamped &pose)
{
    pose.pose.position.x = 0.00f;
    pose.pose.position.y = 0.00f;
    pose.pose.position.z = 0.00f;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, 0);
    pose.pose.orientation = tf2::toMsg(quat);
};

void hover_pattern(geometry_msgs::PoseStamped &pose)
{
    pose.pose.position.x = 0.00f;
    pose.pose.position.y = 0.00f;
    pose.pose.position.z = height;
    tf2::Quaternion quat;
    quat.setRPY(0, 0, 0);
    pose.pose.orientation = tf2::toMsg(quat);
};

void circular_pattern(geometry_msgs::PoseStamped &pose, circular_traj &traj)
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

void square_pattern(geometry_msgs::PoseStamped &pose, square_traj &traj)
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

void star_pattern(geometry_msgs::PoseStamped &pose, star_traj &traj)
{
    // Move along a line segment
    double start_angle = traj.segment_%10 * 2 * M_PI / 10;
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
        traj.segment_++; 
    }
};

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
    int flight_pattern;
    nh.param<int>("flight_pattern", flight_pattern, static_cast<int>(pattern::HOVER)); // Default to HOVER
    nh.param<float>("flight_height", height, 1.00f); // Default flight height - 1.0m

    ROS_WARN("Flight Height = %.2f m", height);

    switch (static_cast<pattern>(flight_pattern))
    {
    case pattern::HOVER:
        ROS_INFO("Flight Pattern: Hover");
        break;
    case pattern::CIRCULAR:
        ROS_INFO("Flight Pattern: Circular");
        break;
    case pattern::SQUARE:
        ROS_INFO("Flight Pattern: Square");
        break;
    case pattern::STAR:
        ROS_INFO("Flight Pattern: Star");
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
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = height;
    pose.pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, 0, 1)); // No rotation (yaw = 0)

    // send a few setpoints before starting
    for (int i = 100; ros::ok() && i > 0; --i)
    {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    bool isCompleted = false;
    bool isReady = false;
    ros::Time last_request = ros::Time::now();
    auto cir_traj = circular_traj{0.05, 0.0, 1.0, 0.3};
    auto squ_traj = square_traj{0.05, 2.0, 1.0, 0.3, 0, 0.0};
    auto sta_traj = star_traj{0.05, 1.0, 0.0, 0.0, 0.3, 0.0, 0.0, 0};
    while (ros::ok())
    {
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(3.0)))
        {
            if (set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent)
            {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        }
        else
        {
            if (!current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(3.0)))
            {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        // fly predefined pattern
        if (current_state.mode == "OFFBOARD" && current_state.armed && !isCompleted)
        {
            switch (static_cast<pattern>(flight_pattern))
            {
            case pattern::HOVER:
                // hover for 30 sec
                hover_pattern(pose);
                if ((ros::Time::now() - last_request > ros::Duration(30.0)))
                    isCompleted = true;
                break;
            case pattern::CIRCULAR:
                if (!isReady)
                {
                    hover_pattern(pose);
                    last_request = ros::Time::now();
                    isReady = true;
                }
                // hover first for 10 sec
                if (isReady && (ros::Time::now() - last_request > ros::Duration(10.0)))
                {
                    ROS_INFO("Traj theta = %.2f", cir_traj.theta_);
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
                    last_request = ros::Time::now();
                    isReady = true;
                }
                // hover first for 10 sec
                if (isReady && (ros::Time::now() - last_request > ros::Duration(10.0)))
                {
                    ROS_INFO("Traj segment = %d", squ_traj.segment_);
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
                    last_request = ros::Time::now();
                    isReady = true;
                }
                // hover first for 10 sec
                if (isReady && (ros::Time::now() - last_request > ros::Duration(10.0)))
                {
                   ROS_INFO("Traj segment = %d", sta_traj.segment_);
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
        if (current_state.armed && isCompleted && current_odom.pose.pose.position.z > 0.1)
        {
            return_origin(pose);
            ROS_INFO("Returning to Origin!");
            ROS_WARN("Altitude = %.2f m", current_odom.pose.pose.position.z);
        }
        else if (current_state.armed && isCompleted && current_odom.pose.pose.position.z <= 0.1)
        {
            arm_cmd.request.value = false;
            if (arming_client.call(arm_cmd) &&
                arm_cmd.response.success)
            {
                ROS_INFO("Vehicle disarmed");
            }
        }

        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

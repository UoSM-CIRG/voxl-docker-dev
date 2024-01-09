/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>

mavros_msgs::State current_state;
struct circular_traj
{
    double dt_;
    double theta_;
    double radius_;
    double omega_;
};

void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    // the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    // wait for FCU connection
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0;
    pose.pose.position.y = 0;
    pose.pose.position.z = 1.5;
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

    ros::Time last_request = ros::Time::now();
    auto traj = circular_traj{0.02, 0.0, 1.0, 0.5};
    while (ros::ok())
    {
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0)))
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
                (ros::Time::now() - last_request > ros::Duration(5.0)))
            {
                if (arming_client.call(arm_cmd) &&
                    arm_cmd.response.success)
                {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        if (current_state.mode == "OFFBOARD" && current_state.armed)
        {
            pose.pose.position.x = traj.radius_ * cos(traj.theta_);
            pose.pose.position.y = traj.radius_ * sin(traj.theta_);
            pose.pose.position.z = 1.5;

            // Calculate angle towards the middle (origin)
            double angle_towards_middle = atan2(0.0 - pose.pose.position.y , 0.0 - pose.pose.position.x);
            tf2::Quaternion quat;
            quat.setRPY(0, 0, angle_towards_middle);
            pose.pose.orientation = tf2::toMsg(quat);

            traj.theta_ = traj.theta_ + traj.omega_ * traj.dt_;
            ROS_INFO("theta = %.2f", traj.theta_);
            ROS_INFO("pos x = %.2f", pose.pose.position.x);
            ROS_INFO("pos y = %.2f", pose.pose.position.y);
            ROS_INFO("yaw angle = %.2f", angle_towards_middle);
        }

        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
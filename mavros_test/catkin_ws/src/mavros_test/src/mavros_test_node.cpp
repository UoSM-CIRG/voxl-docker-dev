/*******************************************************************************
 * Copyright 2022 ModalAI Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * 4. The Software is used solely in conjunction with devices provided by
 *    ModalAI Inc.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>

#include <math.h>

#define RATE    20  // loop rate hz

mavros_msgs::State current_state;



void state_cb(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

int simple_demo()
{
    ROS_INFO("simple_demo");

    int i;

    ros::NodeHandle nh;

    ros::Subscriber state_sub           = nh.subscribe<mavros_msgs::State>
                                        ("mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub        = nh.advertise<geometry_msgs::PoseStamped>
                                        ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client    = nh.serviceClient<mavros_msgs::CommandBool>
                                        ("mavros/cmd/arming");
    ros::ServiceClient land_client      = nh.serviceClient<mavros_msgs::CommandTOL>
                                        ("mavros/cmd/land");
    ros::ServiceClient set_mode_client  = nh.serviceClient<mavros_msgs::SetMode>
                                        ("mavros/set_mode");
    ros::Publisher target_local_pub     = nh.advertise<mavros_msgs::PositionTarget>
                                        ("mavros/setpoint_raw/local", 10);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(RATE);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO("\rconnecting to FCT...");
    }

    // keep this pose constant, home position
    mavros_msgs::PositionTarget position_home;
    position_home.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    position_home.type_mask = mavros_msgs::PositionTarget::IGNORE_VX |
                              mavros_msgs::PositionTarget::IGNORE_VY |
                              mavros_msgs::PositionTarget::IGNORE_VZ |
                              mavros_msgs::PositionTarget::IGNORE_AFX |
                              mavros_msgs::PositionTarget::IGNORE_AFY |
                              mavros_msgs::PositionTarget::IGNORE_AFZ |
                              mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    position_home.position.x = 0;
    position_home.position.y = 0;
    position_home.position.z = 0;
    position_home.velocity.x = 0;
    position_home.velocity.y = 0;
    position_home.velocity.z = 0;
    position_home.acceleration_or_force.x = 0;
    position_home.acceleration_or_force.y = 0;
    position_home.acceleration_or_force.z = 0;
    position_home.yaw = 0;
    position_home.yaw_rate = 0;

    //send a few setpoints before starting
    for(i = 100; ros::ok() && i > 0; --i){
        target_local_pub.publish(position_home);
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("waiting for offboard mode");

    while(ros::ok()){
        target_local_pub.publish(position_home);
        ros::spinOnce();
        rate.sleep();
        if(current_state.mode == "OFFBOARD") break;
    }

    ROS_INFO("arming");

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    if( arming_client.call(arm_cmd) &&
        arm_cmd.response.success){
        ROS_INFO("Vehicle armed");
    }
    else {
        ROS_INFO("Vehicle failed to arm");
        return -1;
    }

    for(int i = 100; ros::ok() && i > 0; --i){
        target_local_pub.publish(position_home);
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("disarming");

    arm_cmd.request.value = false;

    if( arming_client.call(arm_cmd) &&
        arm_cmd.response.success){
        ROS_INFO("Vehicle disarmed");
    }
    else {
        ROS_INFO("Vehicle failed to disarm");
        return -1;
    }

    ROS_INFO("done");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    simple_demo();
}


/* HOW TO ARM , LAND AND CHANGE MODE

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.yaw = 0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.altitude = 0;

    if( set_mode_client.call(offb_set_mode) &&
        offb_set_mode.response.mode_sent){
        ROS_INFO("Offboard enabled");
    }

    if( arming_client.call(arm_cmd) &&
        arm_cmd.response.success){
        ROS_INFO("Vehicle armed");
    }

        ROS_INFO("tring to land");
    while (!(land_client.call(land_cmd) &&
            land_cmd.response.success)){
      //local_pos_pub.publish(pose);
      ROS_INFO("tring to land");
*/

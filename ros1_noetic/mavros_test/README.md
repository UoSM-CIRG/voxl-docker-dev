# VOXL MAVROS Test

This is a simple node to test offboard mode with mavros. It is meant to be built and run inside the roskinetic-xenial docker image.

For more detailed instructions on using this, see https://docs.modalai.com/mavros/

Remember to set the following to true at /etc/modalai/voxl-vision-hub.conf
```
"en_localhost_mavlink_udp":     true,
```  

### Helper scripts:

#### ros_environment.sh

This configures the ROS IP environment variables and is used by run_mavros.sh and run_mavros_test.sh. This is much like the ~/my_ros_env.sh script in the Yocto base image. You don't have to run this script yourself as it is called by run_mavros.sh and run_mavros_test.sh, but this is where you would make changes to match your network configuration.

#### run_mavros.sh

This will start the mavros node configured to listen for UDP packets from voxl-vision-px4 and relay them back to qGroundControl.

#### run_mavros_test.sh

This will start two ROS nodes: mavros and mavros_test. You don't need to call run_mavros.sh first. mavros_test.sh waits for a connection to PX4 through mavros then arms the drone and flys a 1m square before landing.

#### build.sh and clean.sh

These build and clean the catkin_ws and mavros_test node. Make sure to build INSIDE the roskinetic-exenial docker so it links to the roskinetic libraries and builds for arm64.

### Custom Feature

* Added ros noetic support

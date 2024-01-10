# VOXL MAVROS Test

This is a simple node to test offboard mode with mavros for ROS2. It is meant to be built and run inside the roshumble-jammy docker image.

For more info on mavros ROS2, can visit https://github.com/mavlink/mavros/blob/ros2/mavros/README.md#installation.


```bash
rosinstall_generator --format repos mavlink | tee /tmp/mavlink.repos
rosinstall_generator --format repos --upstream mavros | tee -a /tmp/mavros.repos
# alternative: latest source
# rosinstall_generator --format repos --upstream-development mavros | tee -a /tmp/mavros.repos
# For fetching all the dependencies into your ros2_ws, just add '--deps' to the above scripts
# ex: rosinstall_generator --format repos --upstream mavros --deps | tee -a /tmp/mavros.repos
vcs import src < /tmp/mavlink.repos
vcs import src < /tmp/mavros.repos
rosdep install --from-paths src --ignore-src -y
./src/mavros/mavros/scripts/install_geographiclib_datasets.sh
colcon build --symlink-install
```

### Custom Feature

* Added ros humble support
* For foxy build, need to go to 
1. /opt/ros/foxy/include/tf2_geometry_msgs and change tf2_geometry_msgs.h to tf2_geometry_msgs.hpp
2. /opt/ros/foxy/include/tf2_eigen and change tf2_eigen.h to tf2_eigen.hpp

# voxl-docker-dev

This repo is store dockerfiles to be used with ModalAI VoXL Development Drone.

## Folder Structure

- [ROS1](./ros1_noetic): Docker files and mavros offboard example for ROS1.
- [ROS2](./ros2_humble): Docker files and mavros offboard example for ROS2.

## Isaac Sim ROS2 example

![Circle In Pattern](images/circle_in.gif)
![Square In Pattern](images/square_in.gif)

## Common things to look out for

### Storage
The Voxl within the M500 unit is equipped with 4GB of RAM and 32GB of storage, with 15GB allocated for the /data partition, and the remaining space reserved for system packages. 

You can check the disk space with the command ` df -H ` 

If you need more storage space, you have the option to insert an additional SD card. Please be mindful that there are two SD card slots available. One of them is a 32GB SD card dedicated for PX4 log storage that comes pre-installed with the drone. 

Check sd card format with `lsblk or fdisk -l`

Its best to mount the external storage to /media instead of /mnt/sdcard as it is link to /data. 

You may require to format the SD card into ext for linux file system.   
``` 
  mkdir /media/sdcard 
  mount /dev/mmcblk0 /media/sdcard
``` 

### Docker
Before working with Docker, ensure you are connected to the network. Transition your Wi-Fi from softap mode to station mode, as outlined in the instructions at [Wifi Setup](https://docs.modalai.com/voxl-wifi-setup/). You have the option to configure a hotspot from your PC, providing a simple SSID and password. 

Once connected, use the following command to set up Wi-Fi: 
```
adb shell voxl-wifi station <SSID> <Password> 
```
By default, Docker stores its system image in /var/lib/docker, which may have limited available space. In such cases, it might be necessary to relocate it to a different location. The standard procedure is detailed in the guide at [Docker on voxl](https://docs.modalai.com/docker-on-voxl/). 

Docker version 1.9.0 is older and lacks the --data-root option found in more recent versions. In older Docker versions, changing the storage location isn't as straightforward with a command-line option. Nevertheless, you can still move the Docker data directory to another location by manually relocating the data and updating the Docker configuration. 
  
Here are the steps to change the Docker storage location in Docker 1.9.0: 

- Stop the Docker daemon: 
```
systemctl stop docker 
```
- Move the Docker data directory to your desired location. Assuming you want to move it to a directory on your SD card, replace /media/sdcard/docker with your chosen location. By default in /etc/fstat, the sdcard if partitioned correctly, mmcblk0p1 will be mapped to /media/sdcard: 
```
sudo mv /var/lib/docker /media/sdcard/docker 
```
- Create a symlink from the original location to the new location: 
```
sudo ln -s /media/sdcard/docker /var/lib/docker 
```
- Start the Docker daemon: 
```
systemctl start docker 
```
- Verify that Docker is now using the new data directory by running: 
```
systemctl status docker 
```
 
Following these steps, Docker should utilize the new location for its data, allowing you to continue using Docker with data stored on your SD card as usual. Continue on the guide to know how to load and run the docker. 

In some instance, you may need to rerun the voxl docker configuration 
```
voxl-configure-docker-support 
```
 
In new voxl devices, there is an issue with the default docker.service file. 
Modify the file with 
``` 
 vi /lib/systemd/system/docker.service 
```

Find the line below and remove the **–raw-logs**
```
ExecStart=/usr/bin/docker daemon –H fd:// --raw-logs 
```
Afterwards, power cycle the device and restart docker and docker-daemon service 
 
From the official doc https://docs.modalai.com/docker-on-voxl/ 
 
Move Docker image from /data to /media/sdcard 

In order to change where the root of the docker runtime points to in the docker service go to /etc/systemd/system/docker-daemon.service and change the following line so that it uses the sd card(ext4 format) in the voxl ExecStart=/usr/bin/docker daemon -g /media/sdcard/docker 

Reload the daemon: 
```
systemctl daemon-reload 
```
restart the service for the changes to take effect: 
```
systemctl restart docker-daemon.service 
```
Delete the directories that docker uses in the data partition:
```
cd /data
rm -rf containers linkgraph.db overlay tmp volumes graph network repositories-overlay trust 
```
[NOTE] The previous commands will delete any images that were previously available. 

You can now run voxl-configure-docker-support.sh and a hello world image will be created and the directories that were deleted from the /data partition will be in the /mnt/sdcard directory. 

You can verify that you have freed up the /data partition via `df -h` 

[Special Note] Not all SD cards can be used for Docker. In our lab tests a SanDisk Ultra 16GB card worked fine but a SanDisk Extreme 32GB card failed. 

### Voxl-mapper

ModalAI has a mapping interface in beta. To get it, follow the instructions below.

**Pre-requisites:** 

voxl-mapper requires depth info, the depth info can be obtained from voxl-dfs-server, the dfs is built from camera. 

Dependency chain:

voxl-mapper -> voxl-dfs-server -> voxl-camera-server 
 
- First update the system-image 

[Flash system image](https://docs.modalai.com/flash-system-image/)
[Voxl system image](https://docs.modalai.com/voxl-system-image/) 

For seeker-slam, use camera config 3 as we are using Hires rather than ToF. 

**Steps:** 

-Calibrate camera as needed [Calibrate Camera](https://docs.modalai.com/calibrate-cameras/) 
(Be patient as the re-projection error threshold is <0.5, only calibrate for stereo if using dfs, otherwise calibrate tof.) 
 
-On the voxl, go to /etc/opkg/opkg.config and change the path from modalai to dev to point to the specific sdk.  
[Pakacging on voxl](https://docs.modalai.com/packaging-on-voxl/#overview) 

For sdk 1.1 uploaded in step 1, make sure the site is http://voxl-packages.modalai.com/dists/apq8096/sdk-1.1/binary-arm64/ 

- Set voxl-wifi to station mode and run
```  
opkg update 
opkg install voxl-portal voxl-mapper  
```

[Setting up voxl-mapper](https://docs.modalai.com/setting-up-voxl-mapper-0_9/) 
[voxl-mapper gitlab](https://gitlab.com/voxl-public/voxl-sdk/services/voxl-mapper) 

- Change the configuration
``` 
/etc/modalai/voxl-vision-hub.conf   
     "offboard_mode" : "trajectory" 
/etc/modalai/voxl-mapper.conf   
     "tof_n_enable" : false 
     "depth_pipe_n_enable" : true 
```

- View in voxl-portal by accessing http://192.168.8.1 

# voxl-docker-dev

This project provides Dockerfiles and instructions for building Robot Operating System (ROS) Noetic on an aarch64 Linux Ubuntu Focal image. Additionally, it includes guidance on transferring the generated files to a VOXL device. The repository also features a MAVROS test package built using Catkin.

## 1. Getting Started

To get started with VOXL-Docker-Dev, follow these steps:

```
git clone https://github.com/UoSM-CIRG/voxl-docker-dev.git
cd voxl-docker-dev
```

Build the Docker image using the provided Dockerfiles, ideally on a aarch64 device.

```
sudo ./build-docker.sh
```

Then transfer the following two files to the voxl. Make sure you are connected to the voxl device by ADB shell. Alternatively can use scp to transfer the file.
How to setup ADB can be found at https://docs.modalai.com/setting-up-adb/
```
adb push rosnoetic-focal-v1.0.tar /data
adb push mavros_test /home/root
```

In your host PC, access the voxl device either by SSH or ADB and navigate to the directory of the '.tar' file and load the Docker image. Configure the Docker service beforehand with `voxl-configure-docker-support` if necessary.

```
cd /data
docker load -i rosnoetic-focal-v1.0.tar
```

Execute the following command to run the Docker image.
```
voxl-docker -i rosnoetic-focal:v1.0
```

The mavros_test can be found in the container at the path yoctohome/mavros_test

## 2. TODO

* Setup mpa-to-ros https://gitlab.com/voxl-public/voxl-sdk/utilities/voxl-mpa-to-ros

## 3. Reference

* https://docs.modalai.com/mavros/#how-to-run-mavros
* https://docs.modalai.com/docker-on-voxl/#running-a-docker-image
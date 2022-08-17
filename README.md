## Time-synced stereo camera for ROS and AirSim
AirSim has been known for impossible to get time-synced stereo camera [AirSim#4569](https://github.com/microsoft/AirSim/issues/4569), [AirSim#4171](https://github.com/microsoft/AirSim/issues/4171), [AirSim#2784](https://github.com/microsoft/AirSim/issues/2784).

The time delay between 2 cameras is varying around 0.05 - 0.15 seconds depends on computer specs.

This repo solves above problem using the computer vision mode inspired by [VIODE](https://github.com/kminoda/VIODE).

## Info
AirSim: v1.8.1 - Linux.

OS: Ubuntu 20.04 (ROS 1 - Noetic).

Unreal Engine: v4.27.

## Usage
1. Clone AirSim repo, copy and replace the ```CMakeList.txt``` and ```package.xml``` into ```Airsim/ros/src/airsim_ros_pkgs/```. Copy ```stereo_recording_node.cpp``` to ```Airsim/ros/src/airsim_ros_pkgs/src``` and build the code.
2. Record a rosbag with only 2 topics ```imu/Imu``` and ```odom_local_ned```.
3. Correct the path and file name of the recoreded rosbag in ```stereo_recording_node.cpp```.
4. Change the ```settings.json``` to computer vision mode.
5. Run the stereo_recording_node using ```rosrun```.

## Q&A
Contact ```hoangvietdo@sju.ac.kr``` or make an issue for discussion.

PRs are of course welcome.

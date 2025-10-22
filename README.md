# Crazyflie interface for ROS2
These ros2 packages are used to communicate with crazyflie board using ROS2.

This repo is migrated from crazyswarm2 https://imrclab.github.io/crazyswarm2/

## Dependensies
Tested on Unbuntu 24.04 and ROS2 Jazzy

# Install dependencies
```shell
sudo apt install libboost-program-options-dev libusb-1.0-0-dev
pip3 install rowan nicegui
```
If you are planning to use the CFlib backend, do:
```shell
pip3 install cflib transforms3d
sudo apt-get install ros-<DISTRO>-tf-transformations
```

## Setup workspace
```shell
mkdir -p ros2_ws/src
cd ros2_ws/src
git clone https://github.com/Minke99/CrazyROS2.git --recursive
```

Build the workspace

```shell
cd ../
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

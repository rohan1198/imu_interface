# ImuROS

- This is a basic ROS package to interface with the BNO055 IMU.

Note: Still under development

---

### Hardware setup

- The BNO055 is connected to an Arduino Nano (add diagram below).
- The Arduino Nano is connected to a Jetson Orin Nano via USB (change later to use GPIO pins instead).
- The Jetson has Ubuntu 20.04, and ROS Noetic installed on it.

---

### Software Setup

- Initialize Catkin workspace

```
mkdir -p catkin_ws_sensors/src
cd catkin_ws_sensors

catkin init
catkin config --extend /opt/ros/noetic
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin config --merge-devel

cd src
git clone git@github.com:rohan1198/imu_interface.git

cd ..

catkin build
```

### Running the Imu interface

```
source ./devel/setup.zsh

# With Python
roslaunch imu_interface imu_reader_py.launch

# With C++
roslaunch imu_interface imu_reader_cpp.launch
```

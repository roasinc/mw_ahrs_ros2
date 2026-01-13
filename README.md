# mw_ahrs_ros2

[![Licence](https://img.shields.io/badge/License-Apache_2.0-green.svg)](https://opensource.org/licenses/Apache-2.0/)
[![ubuntu22](https://img.shields.io/badge/-UBUNTU_22.04-orange?style=flat-square&logo=ubuntu&logoColor=white)](https://releases.ubuntu.com/jammy/)
[![humble](https://img.shields.io/badge/-HUMBLE-blue?style=flat-square&logo=ros)](https://docs.ros.org/en/humble/index.html)

## Overview

ROS2 driver package for MW-AHRS X1

## ROS Interface

| Topic Name | Type                           | Description           |
|------------|--------------------------------|-----------------------|
| `imu/data` | `sensor_msgs/Imu`              | IMU data              |
| `imu/rpy`  | `geometry_msgs/Vector3Stamped` | roll, pitch, yaw data |
| `imu/mag`  | `sensor_msgs/MagneticField`    | magnetic field data   |

| Service Name | Type               | Description      |
|--------------|--------------------|------------------|
| `imu/reset`  | `std_srvs/Trigger` | Reset the sensor |

## Installation

```shell
cd ~/colcon_ws/src/
git clone https://github.com/roasinc/mw_ahrs_ros2.git

cd ~/colcon_ws/
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
```

## Usage

```shell
sudo cp rules/99-mwahrs.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

ros2 launch mw_ahrs_ros2 mw_ahrs_launch.py
```

// Copyright (c) 2023, ROAS Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MW_AHRS_ROS2__MW_AHRS_DRIVER_HPP_
#define MW_AHRS_ROS2__MW_AHRS_DRIVER_HPP_

#include <atomic>
#include <cmath>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>
#include <libserial/SerialPort.h>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "realtime_tools/realtime_publisher.hpp"
#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/magnetic_field.hpp"
#include "std_srvs/srv/trigger.hpp"

class MwAhrsDriver : public rclcpp::Node
{
public:
  MwAhrsDriver(const std::string& node);

  virtual ~MwAhrsDriver();

  void start_streaming();

  void handle_reset(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                    std::shared_ptr<std_srvs::srv::Trigger::Response> resp);

  void read_loop();

private:
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_reset_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr pub_rpy_;
  rclcpp::Publisher<sensor_msgs::msg::MagneticField>::SharedPtr pub_mag_;

  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::Imu>> rp_imu_;
  std::shared_ptr<realtime_tools::RealtimePublisher<geometry_msgs::msg::Vector3Stamped>> rp_rpy_;
  std::shared_ptr<realtime_tools::RealtimePublisher<sensor_msgs::msg::MagneticField>> rp_mag_;

  std::atomic<bool> running_{ false };
  std::thread thread_;

  mutable std::mutex mutex_;
  LibSerial::SerialPort serial_;

  std::string port_;
  std::string frame_id_;
};

#endif  // MW_AHRS_ROS2__MW_AHRS_DRIVER_HPP_
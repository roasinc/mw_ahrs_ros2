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

#include "mw_ahrs_ros2/mw_ahrs_driver.hpp"

constexpr char eol = '\n';
constexpr size_t timeout = 1000;
constexpr size_t data_size = 12;

MwAhrsDriver::MwAhrsDriver(const std::string& node) : Node(node)
{
  this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
  this->declare_parameter<std::string>("frame_id", "imu_link");

  this->get_parameter("port", port_);
  this->get_parameter("frame_id", frame_id_);

  srv_reset_ = this->create_service<std_srvs::srv::Trigger>(
      "imu/reset", std::bind(&MwAhrsDriver::handle_reset, this, std::placeholders::_1, std::placeholders::_2));

  pub_imu_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data", 1);
  pub_rpy_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>("imu/rpy", 1);
  pub_mag_ = this->create_publisher<sensor_msgs::msg::MagneticField>("imu/mag", 1);

  rp_imu_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::Imu>>(pub_imu_);
  rp_imu_->msg_.header.frame_id = frame_id_;
  rp_imu_->msg_.orientation_covariance = { 0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025 };
  rp_imu_->msg_.angular_velocity_covariance = { 0.02, 0, 0, 0, 0.02, 0, 0, 0, 0.02 };
  rp_imu_->msg_.linear_acceleration_covariance = { 0.04, 0, 0, 0, 0.04, 0, 0, 0, 0.04 };

  rp_rpy_ = std::make_shared<realtime_tools::RealtimePublisher<geometry_msgs::msg::Vector3Stamped>>(pub_rpy_);
  rp_rpy_->msg_.header.frame_id = frame_id_;

  rp_mag_ = std::make_shared<realtime_tools::RealtimePublisher<sensor_msgs::msg::MagneticField>>(pub_mag_);
  rp_mag_->msg_.header.frame_id = frame_id_;

  try
  {
    serial_.Open(port_);
  }
  catch (LibSerial::OpenFailed& e)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to open serial port [%s]: %s", port_.c_str(), e.what());
    throw std::runtime_error("Failed to open serial port: " + port_);
  }

  serial_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
  serial_.SetParity(LibSerial::Parity::PARITY_NONE);
  serial_.SetStopBits(LibSerial::StopBits::STOP_BITS_1);
  serial_.FlushIOBuffers();
  rclcpp::sleep_for(std::chrono::milliseconds(100));

  start_streaming();
  running_.store(true);
  thread_ = std::thread(&MwAhrsDriver::read_loop, this);
}

MwAhrsDriver::~MwAhrsDriver()
{
  running_.store(false);

  if (thread_.joinable())
    thread_.join();

  std::lock_guard<std::mutex> lock(mutex_);
  if (serial_.IsOpen())
    serial_.Close();
}

void MwAhrsDriver::start_streaming()
{
  std::lock_guard<std::mutex> lock(mutex_);

  serial_.Write("ss=15\r\n");  // accel + gyro + angle + mag
  serial_.Write("sp=10\r\n");  // 10ms
}

void MwAhrsDriver::handle_reset(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                std::shared_ptr<std_srvs::srv::Trigger::Response> resp)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!serial_.IsOpen())
    {
      resp->success = false;
      resp->message = "Serial port is not open.";
      return;
    }

    serial_.Write("rst\r\n");
  }

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  start_streaming();

  resp->success = true;
  resp->message = "Reset completed.";
}

void MwAhrsDriver::read_loop()
{
  while (running_.load() && rclcpp::ok())
  {
    std::string line;

    try
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (serial_.IsOpen())
      {
        serial_.ReadLine(line, eol, timeout);
      }
      else
      {
        rclcpp::sleep_for(std::chrono::milliseconds(10));
        continue;
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to read: %s", e.what());
      continue;
    }

    if (line.empty())
      continue;

    if (!line.empty() && line.back() == '\r')
      line.pop_back();

    std::vector<double> data;
    data.reserve(data_size);

    std::istringstream iss(line);
    bool overflow = false;
    double v = 0.0;

    while (iss >> v)
    {
      if (data.size() >= data_size)
      {
        overflow = true;
        break;
      }

      data.push_back(v);
    }

    if (overflow || data.size() != data_size)
      continue;

    try
    {
      const double ax = data[0] * 9.80665;
      const double ay = data[1] * 9.80665;
      const double az = data[2] * 9.80665;

      const double gx = data[3] * M_PI / 180.0;
      const double gy = data[4] * M_PI / 180.0;
      const double gz = data[5] * M_PI / 180.0;

      const double roll = data[6] * M_PI / 180.0;
      const double pitch = data[7] * M_PI / 180.0;
      const double yaw = data[8] * M_PI / 180.0;

      tf2::Quaternion q;
      q.setRPY(roll, pitch, yaw);

      const double mx = data[9] * 1e-6;
      const double my = data[10] * 1e-6;
      const double mz = data[11] * 1e-6;

      const auto stamp = this->now();

      if (rp_rpy_ && rp_rpy_->trylock())
      {
        rp_rpy_->msg_.header.stamp = stamp;
        rp_rpy_->msg_.vector.x = roll;
        rp_rpy_->msg_.vector.y = pitch;
        rp_rpy_->msg_.vector.z = yaw;
        rp_rpy_->unlockAndPublish();
      }

      if (rp_imu_ && rp_imu_->trylock())
      {
        rp_imu_->msg_.header.stamp = stamp;

        rp_imu_->msg_.linear_acceleration.x = ax;
        rp_imu_->msg_.linear_acceleration.y = ay;
        rp_imu_->msg_.linear_acceleration.z = az;

        rp_imu_->msg_.angular_velocity.x = gx;
        rp_imu_->msg_.angular_velocity.y = gy;
        rp_imu_->msg_.angular_velocity.z = gz;

        rp_imu_->msg_.orientation.x = q.x();
        rp_imu_->msg_.orientation.y = q.y();
        rp_imu_->msg_.orientation.z = q.z();
        rp_imu_->msg_.orientation.w = q.w();

        rp_imu_->unlockAndPublish();
      }

      if (rp_mag_ && rp_mag_->trylock())
      {
        rp_mag_->msg_.header.stamp = stamp;
        rp_mag_->msg_.magnetic_field.x = mx;
        rp_mag_->msg_.magnetic_field.y = my;
        rp_mag_->msg_.magnetic_field.z = mz;
        rp_mag_->unlockAndPublish();
      }
    }
    catch (const std::exception& e)
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Failed to parse: %s", e.what());
      continue;
    }
  }
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MwAhrsDriver>("mw_ahrs_driver_node");
  auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  executor->add_node(node);
  executor->spin();

  rclcpp::shutdown();
  return 0;
}
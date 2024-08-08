// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_TRANSPORT__TYPE__IMU_HPP_
#define QRB_ROS_TRANSPORT__TYPE__IMU_HPP_

#include "qrb_sensor_client/sensor_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/type_adapter.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace qrb_ros::transport::type
{

struct Imu
{
  std_msgs::msg::Header header;
  std::shared_ptr<sensors_event_t> acceleration{ nullptr };
  std::shared_ptr<sensors_event_t> gyro{ nullptr };
};

}  // namespace qrb_ros::transport::type

template <>
struct rclcpp::TypeAdapter<qrb_ros::transport::type::Imu, sensor_msgs::msg::Imu>
{
  using is_specialized = std::true_type;
  using custom_type = qrb_ros::transport::type::Imu;
  using ros_message_type = sensor_msgs::msg::Imu;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination);
  static void convert_to_custom(const ros_message_type & source, custom_type & destination);
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(qrb_ros::transport::type::Imu, sensor_msgs::msg::Imu);

#endif  // QRB_ROS_TRANSPORT__TYPE__IMU_HPP_

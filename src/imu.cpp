// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_transport_imu_type/imu.hpp"

void rclcpp::TypeAdapter<qrb_ros::transport::type::Imu,
    sensor_msgs::msg::Imu>::convert_to_ros_message(const custom_type & source,
    ros_message_type & destination)
{
  RCLCPP_DEBUG(rclcpp::get_logger("qrb_ros_transport"), "convert_to_ros_message");

  destination.header = source.header;
  if (source.acceleration == nullptr || source.gyro == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("qrb_ros_transport"), "imu: source data is null");
    return;
  }

  destination.linear_acceleration.x = source.acceleration->acceleration.x;
  destination.linear_acceleration.y = source.acceleration->acceleration.y;
  destination.linear_acceleration.z = source.acceleration->acceleration.z;
  destination.angular_velocity.x = source.gyro->gyro.x;
  destination.angular_velocity.y = source.gyro->gyro.y;
  destination.angular_velocity.z = source.gyro->gyro.z;
}

void rclcpp::TypeAdapter<qrb_ros::transport::type::Imu, sensor_msgs::msg::Imu>::convert_to_custom(
    const ros_message_type & source,
    custom_type & destination)
{
  RCLCPP_DEBUG(rclcpp::get_logger("qrb_ros_transport"), "convert_to_custom");

  destination.header = source.header;

  auto acc_data = std::make_shared<sensors_event_t>();
  auto gyro_data = std::make_shared<sensors_event_t>();

  acc_data->acceleration.x = source.linear_acceleration.x;
  acc_data->acceleration.y = source.linear_acceleration.y;
  acc_data->acceleration.z = source.linear_acceleration.z;

  gyro_data->gyro.x = source.angular_velocity.x;
  gyro_data->gyro.y = source.angular_velocity.y;
  gyro_data->gyro.z = source.angular_velocity.z;

  destination.acceleration = acc_data;
  destination.gyro = gyro_data;
}

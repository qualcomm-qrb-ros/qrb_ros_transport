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

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination)
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

  static void convert_to_custom(const ros_message_type & source, custom_type & destination)
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
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(qrb_ros::transport::type::Imu, sensor_msgs::msg::Imu);

#endif  // QRB_ROS_TRANSPORT__TYPE__IMU_HPP_

// Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_TRANSPORT_POINT_CLOUD2_TYPE__POINT_CLOUD2_HPP_
#define QRB_ROS_TRANSPORT_POINT_CLOUD2_TYPE__POINT_CLOUD2_HPP_

#include "lib_mem_dmabuf/dmabuf.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/type_adapter.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

namespace qrb_ros::transport::type
{

struct PointCloud2
{
  std_msgs::msg::Header header;
  uint32_t height;
  uint32_t width;
  std::vector<sensor_msgs::msg::PointField> fields;
  bool is_bigendian;
  uint32_t point_step;
  uint32_t row_step;
  bool is_dense;

  std::shared_ptr<lib_mem_dmabuf::DmaBuffer> data_fd{ nullptr };
};

}  // namespace qrb_ros::transport::type

template <>
struct rclcpp::TypeAdapter<qrb_ros::transport::type::PointCloud2, sensor_msgs::msg::PointCloud2>
{
  using is_specialized = std::true_type;
  using custom_type = qrb_ros::transport::type::PointCloud2;
  using ros_message_type = sensor_msgs::msg::PointCloud2;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination);
  static void convert_to_custom(const ros_message_type & source, custom_type & destination);
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(qrb_ros::transport::type::PointCloud2,
    sensor_msgs::msg::PointCloud2);

#endif  // QRB_ROS_TRANSPORT_POINT_CLOUD2_TYPE__POINT_CLOUD2_HPP_

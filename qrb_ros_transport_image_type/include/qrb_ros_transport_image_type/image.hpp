// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_TRANSPORT_IMAGE_TYPE__IMAGE_HPP_
#define QRB_ROS_TRANSPORT_IMAGE_TYPE__IMAGE_HPP_

#include <cstring>
#include <memory>

#include "lib_mem_dmabuf/dmabuf.hpp"
#include "qrb_ros_transport_image_type/image_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/type_adapter.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace qrb_ros::transport::type
{

struct Image {
  std_msgs::msg::Header header;
  int width{ 0 };
  int height{ 0 };
  std::string encoding{ "" };
  std::shared_ptr<lib_mem_dmabuf::DmaBuffer> dmabuf{ nullptr };
};

}  // namespace qrb_ros::transport::type

template <>
struct rclcpp::TypeAdapter<qrb_ros::transport::type::Image, sensor_msgs::msg::Image>
{
  using is_specialized = std::true_type;
  using custom_type = qrb_ros::transport::type::Image;
  using ros_message_type = sensor_msgs::msg::Image;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination);
  static void convert_to_custom(const ros_message_type & source, custom_type & destination);
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(qrb_ros::transport::type::Image,
    sensor_msgs::msg::Image);

#endif  // QRB_ROS_TRANSPORT_IMAGE_TYPE__IMAGE_HPP_

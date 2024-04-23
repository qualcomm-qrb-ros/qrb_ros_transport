// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_TRANSPORT__TYPE__IMAGE_HPP_
#define QRB_ROS_TRANSPORT__TYPE__IMAGE_HPP_

#include <cstring>
#include <memory>

#include "lib_mem_dmabuf/dmabuf.hpp"
#include "qrb_ros_transport/image_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/type_adapter.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace qrb_ros::transport::type
{

struct Image
{
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

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination)
  {
    RCLCPP_DEBUG(rclcpp::get_logger("qrb_ros_transport"), "convert_to_ros_message");

    if (!qrb_ros::transport::image_utils::is_support_encoding(source.encoding)) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("qrb_ros_transport"),
          "image: encoding " << source.encoding << " not support");
      return;
    }

    destination.header = source.header;
    destination.width = source.width;
    destination.height = source.height;
    destination.encoding = source.encoding;

    auto stride = qrb_ros::transport::image_utils::get_image_stride(source.width, source.encoding);
    destination.step = stride;

    // read image data from dmabuf
    destination.data.resize(qrb_ros::transport::image_utils::get_image_align_size(
        source.width, source.height, source.encoding));

    if (!qrb_ros::transport::image_utils::read_image_from_dmabuf(source.dmabuf,
            (char *)destination.data.data(), destination.width, destination.height,
            destination.step, destination.encoding, true)) {
      RCLCPP_ERROR(rclcpp::get_logger("qrb_ros_transport"), "image: read from dmabuf failed");
      return;
    }
  }

  static void convert_to_custom(const ros_message_type & source, custom_type & destination)
  {
    RCLCPP_DEBUG(rclcpp::get_logger("qrb_ros_transport"), "convert_to_custom");

    if (!qrb_ros::transport::image_utils::is_support_encoding(source.encoding)) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("qrb_ros_transport"),
          "image: encoding " << source.encoding << " not support");
      return;
    }

    destination.header = source.header;
    destination.width = source.width;
    destination.height = source.height;
    destination.encoding = source.encoding;

    // save image data to dmabuf
    auto buf_size = qrb_ros::transport::image_utils::get_image_align_size(
        source.width, source.height, source.encoding);
    auto dmabuf = lib_mem_dmabuf::DmaBuffer::alloc(buf_size, "/dev/dma_heap/system");
    if (dmabuf == nullptr) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("qrb_ros_transport"),
          "image: alloc dmabuf failed, size: " << buf_size);
      return;
    }

    if (!qrb_ros::transport::image_utils::save_image_to_dmabuf(dmabuf, source.data.data(),
            source.width, source.height, source.step, source.encoding, true)) {
      RCLCPP_ERROR_STREAM(rclcpp::get_logger("qrb_ros_transport"), "image: save to dmabuf failed");
      return;
    }
    destination.dmabuf = dmabuf;
  }
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(qrb_ros::transport::type::Image,
    sensor_msgs::msg::Image);

#endif  // QRB_ROS_TRANSPORT__TYPE__IMAGE_HPP_

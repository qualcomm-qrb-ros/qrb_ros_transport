// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_transport_image_type/image.hpp"

void rclcpp::TypeAdapter<qrb_ros::transport::type::Image,
    sensor_msgs::msg::Image>::convert_to_ros_message(const custom_type & source,
    ros_message_type & destination)
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
          (char *)destination.data.data(), destination.width, destination.height, destination.step,
          destination.encoding, true)) {
    RCLCPP_ERROR(rclcpp::get_logger("qrb_ros_transport"), "image: read from dmabuf failed");
    return;
  }

  // remove extra height alignment
  auto real_size = destination.height * destination.step;
  if (source.encoding == "nv12") {
    real_size *= 1.5;
  }
  destination.data.resize(real_size);
}

void rclcpp::TypeAdapter<qrb_ros::transport::type::Image,
    sensor_msgs::msg::Image>::convert_to_custom(const ros_message_type & source,
    custom_type & destination)
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
    RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("qrb_ros_transport"), "image: alloc dmabuf failed, size: " << buf_size);
    return;
  }

  if (!qrb_ros::transport::image_utils::save_image_to_dmabuf(dmabuf, source.data.data(),
          source.width, source.height, source.step, source.encoding, true)) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("qrb_ros_transport"), "image: save to dmabuf failed");
    return;
  }
  destination.dmabuf = dmabuf;
}

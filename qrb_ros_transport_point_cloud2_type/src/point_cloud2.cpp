// Copyright (c) 2025 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_transport_point_cloud2_type/point_cloud2.hpp"

void rclcpp::TypeAdapter<qrb_ros::transport::type::PointCloud2,
    sensor_msgs::msg::PointCloud2>::convert_to_ros_message(const custom_type & source,
    ros_message_type & destination)
{
  RCLCPP_DEBUG(rclcpp::get_logger("qrb_ros_transport"), "convert_to_ros_message");

  destination.header = source.header;
  destination.width = source.width;
  destination.height = source.height;
  destination.fields = source.fields;
  destination.is_bigendian = source.is_bigendian;
  destination.point_step = source.point_step;
  destination.row_step = source.row_step;
  destination.is_dense = source.is_dense;

  destination.data.resize(source.data_fd->size());

  if (source.data_fd == nullptr || source.data_fd->fd() <= 0) {
    RCLCPP_ERROR(rclcpp::get_logger("qrb_ros_transport"), "point_cloud2: dmabuf is null");
    return;
  }

  if (!(source.data_fd->map() && source.data_fd->sync_start())) {
    RCLCPP_ERROR(rclcpp::get_logger("qrb_ros_transport"), "point_cloud2: dmabuf mmap failed");
    return;
  }

  std::memcpy(destination.data.data(), source.data_fd->addr(), source.data_fd->size());

  if (!(source.data_fd->sync_start() && source.data_fd->unmap())) {
    RCLCPP_ERROR(rclcpp::get_logger("qrb_ros_transport"), "point_cloud2: dmabuf unmap failed");
    return;
  }
}

void rclcpp::TypeAdapter<qrb_ros::transport::type::PointCloud2,
    sensor_msgs::msg::PointCloud2>::convert_to_custom(const ros_message_type & source,
    custom_type & destination)
{
  RCLCPP_DEBUG(rclcpp::get_logger("qrb_ros_transport"), "convert_to_custom");

  destination.header = source.header;
  destination.width = source.width;
  destination.height = source.height;
  destination.fields = source.fields;
  destination.is_bigendian = source.is_bigendian;
  destination.point_step = source.point_step;
  destination.row_step = source.row_step;
  destination.is_dense = source.is_dense;

  // save point_cloud2 data to dmabuf
  auto dmabuf = lib_mem_dmabuf::DmaBuffer::alloc(source.data.size(), "/dev/dma_heap/system");
  if (dmabuf == nullptr) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("qrb_ros_transport"),
        "point_cloud2: alloc dmabuf failed, size: " << source.data.size());
    return;
  }

  if (!dmabuf->set_data((char *)source.data.data(), source.data.size())) {
    RCLCPP_ERROR(rclcpp::get_logger("qrb_ros_transport"), "point_cloud2: save to dmabuf failed");
    return;
  }
  dmabuf->unmap();

  destination.data_fd = dmabuf;
}

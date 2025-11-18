// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_transport_image_type/image_utils.hpp"

#include <iostream>
#include <map>

#include "rclcpp/rclcpp.hpp"

namespace qrb_ros::transport::image_utils
{
#define ALIGN_WIDTH 128
#define ALIGN_HEIGHT 32
#define ALIGN_TOTAL 4096

std::map<std::string, float> supported_encodings = {
  { sensor_msgs::image_encodings::RGB8, 3 },
  { "nv12", 1.5 },
};

int align(int x, int align_size)
{
  if (x >> 2 == 0) {
    return (((x) + (align_size)-1) & (~((align_size)-1)));
  }
  if (x <= align_size) {
    return align_size;
  }
  return std::ceil((double)(x) / align_size) * align_size;
}

int align_width(int width)
{
  return align(width, ALIGN_WIDTH);
}

int align_height(int height)
{
  return align(height, ALIGN_HEIGHT);
}

int align_total_size(int size)
{
  return align(size, ALIGN_TOTAL);
}

int get_image_align_size(int width, int height, const std::string & encoding)
{
  if (!is_support_encoding(encoding)) {
    throw std::runtime_error("unsupported encoding " + encoding);
  }
  auto bbp = bytes_per_pixel(encoding);
  int size = 0;
  if (encoding == sensor_msgs::image_encodings::RGB8) {
    size = align(width, 256) * align_height(height) * bbp;
  } else {
    size = align_width(width) * align_height(height) * bbp;
  }
  return align_total_size(size);
}

bool is_support_encoding(const std::string & encoding)
{
  return supported_encodings.find(encoding) != supported_encodings.end();
}

float bytes_per_pixel(const std::string & encoding)
{
  if (!is_support_encoding(encoding)) {
    throw std::runtime_error("unsupported encoding " + encoding);
  }
  return supported_encodings.at(encoding);
}

int get_image_stride(int width, const std::string & encoding)
{
  if (!is_support_encoding(encoding)) {
    throw std::runtime_error("unsupported encoding " + encoding);
  }
  if (encoding == sensor_msgs::image_encodings::RGB8) {
    return align_width(width) * bytes_per_pixel(encoding);
  }
  if (encoding == "nv12") {
    return align_width(width);
  }
  return -1;
}

bool save_image_to_dmabuf(std::shared_ptr<lib_mem_dmabuf::DmaBuffer> dmabuf,
    const void * data,
    int width,
    int height,
    int src_step,
    const std::string & encoding,
    bool need_align)
{
  if (!is_support_encoding(encoding)) {
    RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("qrb_ros_transport"), "encoding: " << encoding << " not support");
    return false;
  }
  if (!dmabuf->map() || !dmabuf->sync_start()) {
    RCLCPP_ERROR(rclcpp::get_logger("qrb_ros_transport"), "dmabuf mmap failed");
    return false;
  }

  if (!need_align) {
    memcpy(dmabuf->addr(), data, src_step * height * bytes_per_pixel(encoding));
  } else {
    if (encoding == sensor_msgs::image_encodings::RGB8) {
      int line_size = std::ceil(align_width(width) * bytes_per_pixel(encoding));
      for (int i = 0; i < height; i++) {
        memcpy((char *)dmabuf->addr() + i * line_size, (char *)data + i * src_step,
            width * bytes_per_pixel(encoding));
      }
    } else if (encoding == "nv12") {
      // copy Y channel data
      for (int i = 0; i < height; i++) {
        memcpy((char *)dmabuf->addr() + i * align_width(width), (char *)data + i * src_step, width);
      }
      auto offset = align_width(width) * align_height(height);
      // copy UV channel data
      for (int i = 0; i < (height + 1) / 2; i++) {
        memcpy((char *)dmabuf->addr() + offset + i * align_width(width),
            (char *)data + (height + i) * src_step, width);
      }
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("qrb_ros_transport"), "encoding not support");
      return false;
    }
  }

  if (!dmabuf->sync_end() || !dmabuf->unmap()) {
    RCLCPP_ERROR(rclcpp::get_logger("qrb_ros_transport"), "dmabuf unmap failed");
    return false;
  };
  return true;
}

bool read_image_from_dmabuf(std::shared_ptr<lib_mem_dmabuf::DmaBuffer> dmabuf,
    char * dst,
    int width,
    int height,
    int dst_step,
    const std::string & encoding,
    bool need_unalign)
{
  if (!is_support_encoding(encoding)) {
    RCLCPP_ERROR_STREAM(
        rclcpp::get_logger("qrb_ros_transport"), "encoding: " << encoding << " not support");
    return false;
  }

  if (!dmabuf->map() || !dmabuf->sync_start()) {
    RCLCPP_ERROR(rclcpp::get_logger("qrb_ros_transport"), "dmabuf mmap failed");
    return false;
  }

  if (!need_unalign) {
    memcpy(dst, dmabuf->addr(), dst_step * height * bytes_per_pixel(encoding));
  } else {
    if (encoding == sensor_msgs::image_encodings::RGB8) {
      int line_size = std::ceil(align_width(width) * bytes_per_pixel(encoding));
      for (int i = 0; i < height; i++) {
        memcpy(dst + i * dst_step, (char *)dmabuf->addr() + i * line_size, dst_step);
      }
    } else if (encoding == "nv12") {
      // copy Y channel
      for (int i = 0; i < height; i++) {
        memcpy(dst + i * dst_step, (char *)dmabuf->addr() + i * align_width(width), width);
      }
      // copy UV channel
      auto offset = align_width(width) * align_height(height);
      for (int i = 0; i < (height + 1) / 2; i++) {
        memcpy(dst + (height + i) * dst_step,
            (char *)dmabuf->addr() + offset + i * align_width(width), width);
      }
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("qrb_ros_transport"), "encoding not support");
      return false;
    }
  }

  if (!dmabuf->sync_end() || !dmabuf->unmap()) {
    RCLCPP_ERROR(rclcpp::get_logger("qrb_ros_transport"), "dmabuf unmap failed");
    return false;
  };
  return true;
}

}  // namespace qrb_ros::transport::image_utils

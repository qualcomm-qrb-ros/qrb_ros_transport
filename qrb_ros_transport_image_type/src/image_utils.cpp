// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_transport_image_type/image_utils.hpp"

#include <iostream>
#include <map>

#include "rclcpp/rclcpp.hpp"

namespace qrb_ros::transport::image_utils
{
#define ALIGN_TOTAL 4096

#define ALIGN_WIDTH_RGB8 256
#define ALIGN_HEIGHT_RGB8 1

#define ALIGN_WIDTH_NV12 256
#define ALIGN_HEIGHT_NV12 1

std::map<std::string, float> supported_encodings = {
  { sensor_msgs::image_encodings::RGB8, 3 },
  { "nv12", 1.5 },
};

int align(int x, int align_size)
{
  if (x <= align_size) {
    return align_size;
  }
  return ((x + align_size - 1) / align_size) * align_size;
}

int align_width(int width)
{
  return align_width(width, "nv12");
}

int align_height(int height)
{
  return align_height(height, "nv12");
}

int align_width(int width, const std::string & encoding)
{
  if (!is_support_encoding(encoding)) {
    throw std::runtime_error("unsupported encoding " + encoding);
  }
  if (encoding == sensor_msgs::image_encodings::RGB8) {
    return align(width, ALIGN_WIDTH_RGB8);
  }
  if (encoding == "nv12") {
    return align(width, ALIGN_WIDTH_NV12);
  }
  return -1;
}

int align_height(int height, const std::string & encoding)
{
  if (!is_support_encoding(encoding)) {
    throw std::runtime_error("unsupported encoding " + encoding);
  }
  if (encoding == sensor_msgs::image_encodings::RGB8) {
    return align(height, ALIGN_HEIGHT_RGB8);
  }
  if (encoding == "nv12") {
    return align(height, ALIGN_HEIGHT_NV12);
  }
  return -1;
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
  auto size = align_width(width, encoding) * align_height(height, encoding) * bbp;
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
    return align_width(width, encoding) * bytes_per_pixel(encoding);
  }
  if (encoding == "nv12") {
    return align_width(width, encoding);
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

  if (encoding == sensor_msgs::image_encodings::RGB8) {
    int line_size = std::ceil(align_width(width, encoding) * bytes_per_pixel(encoding));
    if (!need_align) {
      memcpy(dmabuf->addr(), data, height * line_size);
    } else {
      for (int i = 0; i < height; i++) {
        memcpy((char *)dmabuf->addr() + i * line_size, (char *)data + i * src_step,
            width * bytes_per_pixel(encoding));
      }
    }
  } else if (encoding == "nv12") {
    if (!need_align) {
      memcpy(dmabuf->addr(), data, get_image_align_size(width, height, "nv12"));
    } else {
      // copy Y channel data
      for (int i = 0; i < height; i++) {
        memcpy((char *)dmabuf->addr() + i * align_width(width, encoding),
            (char *)data + i * src_step, width);
      }
      auto offset = align_width(width, encoding) * align_height(height, encoding);
      // copy UV channel data
      for (int i = 0; i < (height + 1) / 2; i++) {
        memcpy((char *)dmabuf->addr() + offset + i * align_width(width, encoding),
            (char *)data + (height + i) * src_step, width);
      }
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("qrb_ros_transport"), "encoding not support");
    return false;
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

  if (encoding == sensor_msgs::image_encodings::RGB8) {
    int line_size = std::ceil(align_width(width, encoding) * bytes_per_pixel(encoding));
    if (!need_unalign) {
      memcpy(dst, dmabuf->addr(), height * line_size);
    } else {
      for (int i = 0; i < height; i++) {
        memcpy(dst + i * dst_step, (char *)dmabuf->addr() + i * line_size, dst_step);
      }
    }
  } else if (encoding == "nv12") {
    if (!need_unalign) {
      memcpy(dst, dmabuf->addr(), get_image_align_size(width, height, "nv12"));
    } else {
      // copy Y channel
      for (int i = 0; i < height; i++) {
        memcpy(
            dst + i * dst_step, (char *)dmabuf->addr() + i * align_width(width, encoding), width);
      }
      // copy UV channel
      auto offset = align_width(width, encoding) * align_height(height, encoding);
      for (int i = 0; i < (height + 1) / 2; i++) {
        memcpy(dst + (height + i) * dst_step,
            (char *)dmabuf->addr() + offset + i * align_width(width, encoding), width);
      }
    }
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("qrb_ros_transport"), "encoding not support");
    return false;
  }

  if (!dmabuf->sync_end() || !dmabuf->unmap()) {
    RCLCPP_ERROR(rclcpp::get_logger("qrb_ros_transport"), "dmabuf unmap failed");
    return false;
  };
  return true;
}

}  // namespace qrb_ros::transport::image_utils

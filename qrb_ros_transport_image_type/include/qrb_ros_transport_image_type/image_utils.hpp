// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_TRANSPORT_IMAGE_TYPE__IMAGE_UTILS_HPP_
#define QRB_ROS_TRANSPORT_IMAGE_TYPE__IMAGE_UTILS_HPP_

#include "lib_mem_dmabuf/dmabuf.hpp"
#include "sensor_msgs/image_encodings.hpp"

namespace qrb_ros::transport::image_utils
{

int align(int x, int align_size);

[[deprecated("Use align_width(width, encoding) instead.")]] int align_width(int width);

[[deprecated("Use align_height(height, encoding) instead.")]] int align_height(int height);

int align_width(int width, const std::string & encoding);

int align_height(int height, const std::string & encoding);

int align_total_size(int size);

int get_image_align_size(int width, int height, const std::string & encoding);

bool is_support_encoding(const std::string & encoding);

float bytes_per_pixel(const std::string & encoding);

int get_image_stride(int width, const std::string & encoding);

bool save_image_to_dmabuf(std::shared_ptr<lib_mem_dmabuf::DmaBuffer> dmabuf,
    const void * data,
    int width,
    int height,
    int src_step,
    const std::string & encoding,
    bool need_align = false);

bool read_image_from_dmabuf(std::shared_ptr<lib_mem_dmabuf::DmaBuffer> dmabuf,
    char * dst,
    int width,
    int height,
    int dst_step,
    const std::string & encoding,
    bool need_unalign = false);

}  // namespace qrb_ros::transport::image_utils

#endif  // QRB_ROS_TRANSPORT_IMAGE_TYPE__IMAGE_UTILS_HPP_

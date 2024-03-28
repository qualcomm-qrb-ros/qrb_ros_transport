// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_transport/type/image.hpp"

#include "dmabuf_transport/dmabuf.hpp"
#include "opencv2/imgproc/imgproc.hpp"

namespace qrb_ros
{
namespace transport
{
namespace type
{

void Image::get_ros_data(uint8_t* dst) const
{
  int height = get_ros_message().height;
  int width = get_ros_message().width;
  auto buffer = get_dma_buf();
  buffer->map();
  buffer->sync_start();
  uint8_t* src = (uint8_t*)buffer->addr();

  cv::Mat nv12(height * 1.5, width, CV_8UC1);

  // remove align
  for (int i = 0; i < height; i++) {
    const uint8_t* tmp_src = src + i * align_width_;
    uint8_t* tmp_dst = nv12.data + i * width;
    memcpy(tmp_dst, tmp_src, width);
  }
  for (int i = 0; i < height / 2; i++) {
    const uint8_t* tmp_src = src + (i + align_height_) * align_width_;
    uint8_t* tmp_dst = nv12.data + (i + height) * width;
    memcpy(tmp_dst, tmp_src, width);
  }

  // convert nv12 to bgr8
  cv::Mat rgb8(height, width, CV_8UC3, dst);
  cv::cvtColor(nv12, rgb8, cv::COLOR_YUV2BGR_NV12);

  buffer->sync_end();
  buffer->unmap();
}

void Image::save_ros_data(const uint8_t* src) const
{
  int height = this->get_ros_message().height;
  int width = this->get_ros_message().width;

  // convert bgr8 to yuv420
  uint8_t* tmp_src = const_cast<uint8_t*>(src);
  cv::Mat rgb8(height, width, CV_8UC3, tmp_src);
  cv::Mat yuv420(height * 1.5, width, CV_8UC1);
  cv::cvtColor(rgb8, yuv420, cv::COLOR_BGR2YUV_I420);

  // save data to nv12
  auto buffer = get_dma_buf();
  buffer->map();
  buffer->sync_start();

  uint8_t* nv12_buffer = (uint8_t*)buffer->addr();
  // copy y channel data
  for (int i = 0; i < height; i++) {
    memcpy(nv12_buffer + i * align_width_, yuv420.data + i * width, width);
  }

  // copy u,v channel data
  int u_channel_offset = height * width;
  int align_u_channel_offset = align_height_ * align_width_;
  int v_channel_offset = u_channel_offset / 4;
  int half_width = width / 2;
  int half_height = ((align_height_ >> 6) + 1) << 5;

  for (int j = 0; j < half_height; j++) {
    for (int i = 0; i < half_width; i++) {
      int index = u_channel_offset + j * half_width + i;
      nv12_buffer[align_u_channel_offset + j * align_width_ + 2 * i] = yuv420.data[index];
      nv12_buffer[align_u_channel_offset + j * align_width_ + 2 * i + 1] =
          yuv420.data[index + v_channel_offset];
    }
  }

  buffer->sync_end();
  buffer->unmap();
}

}  // namespace type
}  // namespace transport
}  // namespace qrb_ros

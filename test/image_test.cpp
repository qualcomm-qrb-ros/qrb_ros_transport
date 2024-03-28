// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_transport/type/image.hpp"

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("image_test");
  auto pub = node->create_publisher<qrb_ros::transport::type::Image>("image", 10);

  sensor_msgs::msg::Image msg;
  msg.header = std_msgs::msg::Header();
  msg.width = 1920;
  msg.height = 1080;
  msg.encoding = "nv12";

  // save image data to dmabuf
  auto heap = "/dev/dma_heap/qcom,system";
  uint32_t align_width = ALIGN(msg.width, ALIGN_WIDTH);
  uint32_t align_height = ALIGN(msg.height, ALIGN_HEIGHT);
  // for y channel
  uint32_t size = align_width * align_height;
  // for u,v channels
  size += align_height % 64 == 0 ? align_height * align_width * 0.5 :
                                   (((align_height >> 6) + 1) << 5) * align_width;
  auto dmabuf = dmabuf_transport::DmaBuffer::alloc(size, heap);
  if (dmabuf == nullptr) {
    RCLCPP_ERROR(rclcpp::get_logger("image_test"), "dma buffer alloc failed");
    return -1;
  }

  qrb_ros::transport::type::Image typed_msg;
  // ros message not include real data
  typed_msg.set_ros_message(msg);
  typed_msg.set_dma_buf(dmabuf);
  typed_msg.set_align_height(align_height);
  typed_msg.set_align_width(align_width);

  // publish typed message
  pub->publish(typed_msg);

  rclcpp::shutdown();

  return 0;
}

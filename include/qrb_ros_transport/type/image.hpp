// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ROS_TRANSPORT__TYPE__IMAGE_HPP_
#define QRB_ROS_TRANSPORT__TYPE__IMAGE_HPP_

#include <cstring>
#include <memory>

#include "dmabuf_transport/dmabuf_type_adapter.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/type_adapter.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/image.hpp"

#define ALIGN(x, y) (((x) + (y)-1) & (~((y)-1)))
#define ALIGN_HEIGHT 32
#define ALIGN_WIDTH 128

namespace qrb_ros
{
namespace transport
{
namespace type
{

struct Image : public dmabuf_transport::DmaBufTypeAdapter<sensor_msgs::msg::Image>
{
  explicit Image() = default;
  ~Image() {}

  uint32_t get_align_width() const { return align_width_; }
  void set_align_width(const uint32_t& width) { align_width_ = width; }

  uint32_t get_align_height() const { return align_height_; }
  void set_align_height(const uint32_t& height) { align_height_ = height; }

  void set_message(const sensor_msgs::msg::Image& message)
  {
    header = message.header;
    sensor_msgs::msg::Image msg;
    msg.height = message.height;
    msg.width = message.width;
    msg.step = message.step;
    msg.encoding = message.encoding;
    set_ros_message(msg);
  }

  void get_ros_data(uint8_t* dst) const;
  void save_ros_data(const uint8_t* src) const;

  std_msgs::msg::Header header;

private:
  uint32_t align_height_;
  uint32_t align_width_;
};

}  // namespace type
}  // namespace transport
}  // namespace qrb_ros

template <>
struct rclcpp::TypeAdapter<qrb_ros::transport::type::Image, sensor_msgs::msg::Image>
{
  using is_specialized = std::true_type;
  using custom_type = qrb_ros::transport::type::Image;
  using ros_message_type = sensor_msgs::msg::Image;

  static void convert_to_ros_message(const custom_type& source, ros_message_type& destination)
  {
    destination.header = source.header;
    // copy message informations, not include data
    destination = source.get_ros_message();
    destination.encoding = "bgr8";
    // copy message data
    int height = source.get_ros_message().height;
    int width = source.get_ros_message().width;
    int size = height * width * 3;
    destination.data.resize(size);
    source.get_ros_data(destination.data.data());
  }

  static void convert_to_custom(const ros_message_type& source, custom_type& destination)
  {
    if (!sensor_msgs::image_encodings::isColor(source.encoding)) {
      RCLCPP_ERROR(rclcpp::get_logger("qrb_ros_image_type_adapter"),
          "convert_to_custom failed because source is not color");
      return;
    }
    auto heap = "/dev/dma_heap/qcom,system";
    uint32_t align_height = ALIGN(source.height, ALIGN_HEIGHT);
    uint32_t align_width = ALIGN(source.width, ALIGN_WIDTH);
    uint32_t size = align_width * align_height;
    size += align_height % 64 == 0 ? align_height * align_width * 0.5 :
                                     (((align_height >> 6) + 1) << 5) * align_width;
    auto dmabuf = dmabuf_transport::DmaBuffer::alloc(size, heap);
    if (dmabuf == nullptr) {
      RCLCPP_ERROR(rclcpp::get_logger("qrb_ros_image_type_adapter"), "dma buffer alloc failed");
      return;
    }
    destination.set_dma_buf(dmabuf);
    destination.set_message(source);
    destination.set_align_height(align_height);
    destination.set_align_width(align_width);
    destination.save_ros_data(source.data.data());
  }
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(qrb_ros::transport::type::Image,
    sensor_msgs::msg::Image);

#endif  // QRB_ROS_TRANSPORT__TYPE__IMAGE_HPP_

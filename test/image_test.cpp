// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_transport_image_type/image.hpp"

#include <fstream>

// nv12 image, 1920x1080, 3133440 bytes
#define NV12_FILE_PATH "/data/src.yuv"

using namespace qrb_ros::transport::image_utils;

std::unique_ptr<char[]>
mock_image_data(const std::string & path, int width, int height, const std::string & encoding)
{
  int size = get_image_align_size(width, height, encoding);
  auto data = std::make_unique<char[]>(size);

  std::ifstream ifs(path, std::ios::in | std::ios::binary);
  if (ifs.is_open()) {
    ifs.read(data.get(), size);
  } else {
    std::cerr << "open file: " << path << "failed" << std::endl;
  }
  ifs.close();
  return data;
}

class PubNode : public rclcpp::Node
{
public:
  explicit PubNode() : Node("image_test_pub")
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "image_test_pub start");

    int width = 1920;
    int height = 1080;

    // create publisher
    rclcpp::QoS qos{ 10 };
    qos.transient_local();
    pub_ = this->create_publisher<qrb_ros::transport::type::Image>("image", qos);

    int count = 10;
    while (count-- > 0) {
      // create message
      auto msg = std::make_unique<qrb_ros::transport::type::Image>();
      msg->header = std_msgs::msg::Header();
      msg->width = width;
      msg->height = height;
      msg->encoding = "nv12";

      // alloc dmabuf
      auto size = get_image_align_size(width, height, "nv12");

      auto dmabuf = lib_mem_dmabuf::DmaBuffer::alloc(size, "/dev/dma_heap/system");
      if (dmabuf == nullptr) {
        RCLCPP_ERROR(rclcpp::get_logger("image_test"), "dma buffer alloc failed");
        return;
      }
      // save image data to dmabuf
      auto data = mock_image_data(NV12_FILE_PATH, width, height, "nv12");
      auto step = get_image_stride(width, "nv12");
      if (!save_image_to_dmabuf(dmabuf, data.get(), width, height, step, "nv12")) {
        RCLCPP_ERROR(rclcpp::get_logger("image_test"), "dma buffer write failed");
        return;
      }
      msg->dmabuf = dmabuf;

      // publish image
      RCLCPP_INFO_STREAM(this->get_logger(), "publish image fd: " << msg->dmabuf->fd());
      pub_->publish(std::move(msg));

      rclcpp::sleep_for(std::chrono::seconds(1));
    }
  }

private:
  rclcpp::Publisher<qrb_ros::transport::type::Image>::SharedPtr pub_;
};

class SubNode : public rclcpp::Node
{
public:
  explicit SubNode() : Node("image_test_sub")
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "image_test_sub start");

    rclcpp::QoS qos{ 10 };
    qos.transient_local();

    sub_ = this->create_subscription<qrb_ros::transport::type::Image>(
        "image", qos, [this](const std::shared_ptr<qrb_ros::transport::type::Image> msg) {
          RCLCPP_INFO_STREAM(this->get_logger(), "got message: image encoding: " << msg->encoding);
          RCLCPP_INFO_STREAM(this->get_logger(), "image data fd: " << msg->dmabuf->fd());

          auto size = qrb_ros::transport::image_utils::get_image_align_size(
              msg->width, msg->height, msg->encoding);
          auto data = std::make_unique<char[]>(size);

          qrb_ros::transport::image_utils::read_image_from_dmabuf(msg->dmabuf, data.get(),
              msg->width, msg->height,
              qrb_ros::transport::image_utils::get_image_stride(msg->width, msg->encoding),
              msg->encoding, false);

          std::ofstream file("/data/dump", std::ios::binary);
          if (file.is_open()) {
            file.write(data.get(), size);
            file.close();
          } else {
            RCLCPP_INFO(this->get_logger(), "dump to file failed");
          }
        });
  }

private:
  rclcpp::Subscription<qrb_ros::transport::type::Image>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

#ifdef TEST_PUBLISHER
  rclcpp::spin(std::make_shared<PubNode>());
#endif

#ifdef TEST_SUBSCRIBER
  rclcpp::spin(std::make_shared<SubNode>());
#endif

  rclcpp::shutdown();

  return 0;
}

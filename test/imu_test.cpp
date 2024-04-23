// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_transport/type/imu.hpp"

class PubNode : public rclcpp::Node
{
public:
  explicit PubNode() : Node("imu_test_pub")
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "imu_test_pub start");

    // create publisher
    rclcpp::QoS qos{ 10 };
    qos.transient_local();
    pub_ = this->create_publisher<qrb_ros::transport::type::Imu>("imu", qos);

    int count = 10;
    while (count-- > 0) {
      // create message
      auto msg = std::make_unique<qrb_ros::transport::type::Imu>();
      msg->header = std_msgs::msg::Header();
      msg->header.frame_id = std::to_string(10 - count);

      // mock imu data
      auto acc_data = std::make_shared<sensors_event_t>();
      acc_data->acceleration.x = 10 - count;

      auto gyro_data = std::make_shared<sensors_event_t>();
      gyro_data->gyro.x = 10 - count;

      msg->acceleration = acc_data;
      msg->gyro = gyro_data;

      // publish imu
      RCLCPP_INFO_STREAM(this->get_logger(), "publish imu data: " << msg->header.frame_id);
      pub_->publish(std::move(msg));

      rclcpp::sleep_for(std::chrono::seconds(1));
    }
  }

private:
  rclcpp::Publisher<qrb_ros::transport::type::Imu>::SharedPtr pub_;
};

class SubNode : public rclcpp::Node
{
public:
  explicit SubNode() : Node("imu_test_sub")
  {
    RCLCPP_INFO_STREAM(this->get_logger(), "imu_test_sub start");

    rclcpp::QoS qos{ 10 };
    qos.transient_local();

    sub_ = this->create_subscription<qrb_ros::transport::type::Imu>(
        "imu", qos, [this](const std::shared_ptr<qrb_ros::transport::type::Imu> msg) {
          RCLCPP_INFO_STREAM(
              this->get_logger(), "got message: imu frame: " << msg->header.frame_id);
          RCLCPP_INFO_STREAM(this->get_logger(), "imu data: " << msg->acceleration->acceleration.x);
        });
  }

private:
  rclcpp::Subscription<qrb_ros::transport::type::Imu>::SharedPtr sub_;
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

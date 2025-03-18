// Copyright (c) 2024 Qualcomm Innovation Center, Inc. All rights reserved.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#include "qrb_ros_transport_image_type/image_utils.hpp"

#include "gtest/gtest.h"

TEST(image_utils, alignment)
{
  ASSERT_EQ(qrb_ros::transport::image_utils::align(9, 16), 16);
  ASSERT_EQ(qrb_ros::transport::image_utils::align(9, 10), 10);
  ASSERT_EQ(qrb_ros::transport::image_utils::align(9, 5), 10);
  ASSERT_EQ(qrb_ros::transport::image_utils::align_width(100), 128);
  ASSERT_EQ(qrb_ros::transport::image_utils::align_height(30), 32);
  ASSERT_EQ(qrb_ros::transport::image_utils::align_total_size(4000), 4096);
}

TEST(image_utils, is_support_encoding)
{
  ASSERT_TRUE(
      qrb_ros::transport::image_utils::is_support_encoding(sensor_msgs::image_encodings::RGB8));
  ASSERT_TRUE(qrb_ros::transport::image_utils::is_support_encoding("nv12"));
  ASSERT_FALSE(qrb_ros::transport::image_utils::is_support_encoding("yuv420"));
}

TEST(image_utils, bytes_per_pixel)
{
  ASSERT_EQ(
      3, qrb_ros::transport::image_utils::bytes_per_pixel(sensor_msgs::image_encodings::RGB8));
  ASSERT_EQ(1.5, qrb_ros::transport::image_utils::bytes_per_pixel("nv12"));
}

TEST(image_utils, get_image_align_size)
{
  int width = 600;
  int height = 400;

  int rgb8_size = 640 * 416 * 3;
  ASSERT_EQ(rgb8_size, qrb_ros::transport::image_utils::get_image_align_size(
                           width, height, sensor_msgs::image_encodings::RGB8));

  // Y channel + UV channel + alignment to 4096
  int nv12_size = 640 * 416 * 1 + (640 * 416 * 0.5) + 2048;  // 401,408
  ASSERT_EQ(
      nv12_size, qrb_ros::transport::image_utils::get_image_align_size(width, height, "nv12"));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

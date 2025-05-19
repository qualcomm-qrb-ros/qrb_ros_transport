# Changelog for package qrb_ros_transport

## 1.3.0-jazzy (2025-05-19)

- Add end to end test components ([#6](https://github.com/qualcomm-qrb-ros/qrb_ros_transport/pull/6))
- Add PointCloud2 transport support ([#8](https://github.com/qualcomm-qrb-ros/qrb_ros_transport/pull/8))
- Add QIRP SDK build checkers ([#13](https://github.com/qualcomm-qrb-ros/qrb_ros_transport/pull/13))
- Add commit message checkers ([#24](https://github.com/qualcomm-qrb-ros/qrb_ros_transport/pull/24))
- Fix type header export issue ([#5](https://github.com/qualcomm-qrb-ros/qrb_ros_transport/pull/5))
- Fix RGB8 image dmabuf allocation size ([#11](https://github.com/qualcomm-qrb-ros/qrb_ros_transport/pull/11))
- Fix dmabuf not unmap when not align ([#17](https://github.com/qualcomm-qrb-ros/qrb_ros_transport/pull/17))
- Fix pcl dependencies for test ([#19](https://github.com/qualcomm-qrb-ros/qrb_ros_transport/pull/19))
- Contributors: Peng Wang, Zhaoyuan Cheng, Jiaxing Shi

## 1.2.0 (2024-10-26)

- Separate Image and IMU types to standalone packages
- Contributors: Peng Wang

## 1.1.0 (2024-07-12)

- Add IMU transport support
- Refactor Image type, add `image_utils` for image alignment
- Contributers: Peng Wang

## 1.0.0 (2024-03-28)

- Initial version release for Humble
- Contributors: Peng Wang

# QRB ROS Transport

## Overview

[QRB ROS Transport](https://github.com/qualcomm-qrb-ros/qrb_ros_transport) is designed for zero-copy transporting ROS messages on Qualcomm robotics platforms.

 It is implemented based on [REP 2007](https://ros.org/reps/rep-2007.html), which provides interfaces to define methods for serializing custom types and/or using those types in intra-process communication without conversion.

## Getting Started

### Build

For the Qualcomm QCLinux platform, we provide two ways to build this package.

<details>
<summary>On-Device Compilation with Docker</summary>

1. Set up the QCLinux Docker environment following the [QRB ROS Docker Setup](https://github.com/qualcomm-qrb-ros/qrb_ros_docker?tab=readme-ov-file#quickstart).

2. Clone and build the source code:

    ```bash
    git clone https://github.com/qualcomm-qrb-ros/lib_mem_dmabuf.git
    git clone https://github.com/qualcomm-qrb-ros/qrb_ros_imu.git
    git clone https://github.com/qualcomm-qrb-ros/qrb_ros_transport.git

    colcon build --packages-skip qrb_ros_transport_test
    ```

</details>

<details><summary>Cross Compilation with QIRP SDK</summary>

1. Set up the QIRP SDK environment: Refer to [QRB ROS Documents: Getting Started](https://qualcomm-qrb-ros.github.io/main/getting_started/environment_setup.html).

2. Create a workspace and clone the source code:

    ```bash
    mkdir -p <qirp_decompressed_workspace>/qirp-sdk/ros_ws
    cd <qirp_decompressed_workspace>/qirp-sdk/ros_ws

    git clone https://github.com/qualcomm-qrb-ros/qrb_ros_transport.git
    ```

3. Build the source code with QIRP SDK:

    ```bash
    colcon build --merge-install --packages-skip qrb_ros_transport_test --cmake-args ${CMAKE_ARGS}
    ```

4. Install ROS package to device

   ```bash
   cd install
   tar czvf qrb_ros_transport.tar.gz include lib share
   scp qrb_ros_transport.tar.gz root@[ip-addr]:~
   ssh root@[ip-addr]
   (ssh) mount -o remount,rw /usr
   (ssh) tar --no-same-owner -zxf ~/qrb_ros_transport.tar.gz -C /usr/
   ```

</details>

### Usage

This section shows how to use `qrb_ros_transport` in your projects, here take `qrb_ros::transport::type::Image` as an example.

Add dependencies in your `package.xml`:

```xml
<depend>qrb_ros_transport_image_type</depend>
```

Use `ament_cmake_auto` to find dependencies in your `CMakeLists.txt`:

```cmake
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
```

<details open><summary>Using adapted types in your ROS node</summary>

```c++
#include "qrb_ros_transport_image_type/image.hpp"

// Create message
auto msg = std::make_unique<qrb_ros::transport::type::Image>();
msg->header = std_msgs::msg::Header();
msg->width = width;
msg->height = height;
msg->encoding = "nv12";

// Allocate dmabuf for message
auto dmabuf = lib_mem_dmabuf::DmaBuffer::alloc(size, "/dev/dma_heap/system");
// ... set data to dmabuf
msg->dmabuf = dmabuf;

// Publish message
pub->publish(std::move(msg));
```
</details>

For more details, check out the documentation at [qualcomm-qrb-ros.github.io](https://qualcomm-qrb-ros.github.io/).

## Supported Types

The following table lists the currently supported types:

| QRB ROS Transport Type          | ROS Interfaces          |
| ------------------------------- | ----------------------- |
| [qrb_ros::transport::type::Image](./qrb_ros_transport_image_type/include/qrb_ros_transport_image_type/image.hpp) | [sensor_msgs::msg::Image](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Image.msg) |
| [qrb_ros::transport::type::Imu](./qrb_ros_transport_imu_type/include/qrb_ros_transport_imu_type/imu.hpp) | [sensor_msgs::msg::Imu](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Imu.msg) |
| [qrb_ros::transport::type::PointCloud2](./qrb_ros_transport_point_cloud2_type/include/qrb_ros_transport_point_cloud2_type/point_cloud2.hpp) | [sensor_msgs::msg::PointCloud2](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/PointCloud2.msg) |

## Contributing

We would love to have you as a part of the QRB ROS community. Whether you are helping us fix bugs, proposing new features, improving our documentation, or spreading the word, please refer to our [contribution guidelines](./CONTRIBUTING.md) and [code of conduct](./CODE_OF_CONDUCT.md).

- Bug report: If you see an error message or encounter failures, please create a [bug report](../../issues)
- Feature Request: If you have an idea or if there is a capability that is missing and would make development easier and more robust, please submit a [feature request](../../issues)

## Authors

* **Peng Wang** - *Maintainer* - [@penww](https://github.com/penww)

See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.

## License

Project is licensed under the [BSD-3-Clause License](https://spdx.org/licenses/BSD-3-Clause.html). See [LICENSE](./LICENSE) for the full license text.

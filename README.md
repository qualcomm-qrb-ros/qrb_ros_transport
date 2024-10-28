# QRB ROS Transport

## Overview

[QRB ROS Transport](https://github.com/quic-qrb-ros/qrb_ros_transport) is designed for hardware-acceleration friendly transporting of messages on Qualcomm robotics platforms. It uses type adaption to make message data zero-copy between different ROS nodes, and different hardwares. It includes adapted types for Qualcomm robotics platforms.

[Type Adaptation Feature (REP 2007)](https://ros.org/reps/rep-2007.html) has enabled on ROS 2 Humble. This interface will allow us to define methods for serializing directly to the user requested type, and/or using that type in intra-process communication without ever converting it.

qrb_ros_transport is based on [lib_mem_dmabuf](https://github.com/quic-qrb-ros/lib_mem_dmabuf), it is open sourced and apply to all platforms based on Linux.

## Getting Started

### Prerequisites

- Linux kernel version 5.12 and later, for kernel dma-buf support.
- ROS 2 Humble and later, for type adaption support.

### Cross Compile with QCLINUX SDK

Setup QCLINUX SDK environments:
- Reference [QRB ROS Documents: Getting Started](https://quic-qrb-ros.github.io/getting_started/environment_setup.html)

Create workspace in QCLINUX SDK environment and clone source code

```bash
mkdir -p <qirp_decompressed_workspace>/qirp-sdk/ros_ws
cd <qirp_decompressed_workspace>/qirp-sdk/ros_ws

git clone https://github.com/quic-qrb-ros/lib_mem_dmabuf.git
git clone https://github.com/quic-qrb-ros/qrb_ros_imu.git
git clone https://github.com/quic-qrb-ros/qrb_ros_transport.git
```

Build source code with QCLINUX SDK

```bash
export AMENT_PREFIX_PATH="${OECORE_TARGET_SYSROOT}/usr;${OECORE_NATIVE_SYSROOT}/usr"
export PYTHONPATH=${PYTHONPATH}:${OECORE_TARGET_SYSROOT}/usr/lib/python3.10/site-packages

colcon build --merge-install --cmake-args \
  -DPython3_ROOT_DIR=${OECORE_TARGET_SYSROOT}/usr \
  -DPython3_NumPy_INCLUDE_DIR=${OECORE_TARGET_SYSROOT}/usr/lib/python3.10/site-packages/numpy/core/include \
  -DPYTHON_SOABI=cpython-310-aarch64-linux-gnu -DCMAKE_STAGING_PREFIX=$(pwd)/install \
  -DCMAKE_PREFIX_PATH=$(pwd)/install/share \
  -DBUILD_TESTING=OFF
```

### Use `qrb_ros_transport` in your package

Add dependencies in your package.xml

```xml
<depend>qrb_ros_transport_image_type</depend>
```
Use ament_cmake_auto to find dependencies in your CMakeLists.txt

```cmake
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
```

Using adapted types in your ROS node

```c++
#include "qrb_ros_transport_image_type/image.hpp"

// create message
auto msg = std::make_unique<qrb_ros::transport::type::Image>();
msg->header = std_msgs::msg::Header();
msg->width = width;
msg->height = height;
msg->encoding = "nv12";

// alloc dmabuf for message
auto dmabuf = lib_mem_dmabuf::DmaBuffer::alloc(size, "/dev/dma_heap/system");
// ... set data to dmabuf
msg->dmabuf = dmabuf;

// publish message
pub->publish(std::move(msg));
```

## Supported Types

The following table lists current supported types:

| QRB ROS Transport Type          | ROS Interface           |
| ------------------------------- | ----------------------- |
| [qrb_ros::transport::type::Image](./qrb_ros_transport_image_type/include/qrb_ros_transport_image_type/image.hpp) | [sensor_msgs::msg::Image](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Image.msg) |
| [qrb_ros::transport::type::Imu](./qrb_ros_transport_imu_type/include/qrb_ros_transport_imu_type/imu.hpp) | [sensor_msgs::msg::Imu](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Imu.msg) |

## Supported Platforms

This package is designed and tested to be compatible with ROS 2 Humble running on Qualcomm RB3 gen2.

| Hardware                                                     | Software          |
| ------------------------------------------------------------ | ----------------- |
| [Qualcomm RB3 gen2](https://www.qualcomm.com/developer/hardware/rb3-gen-2-development-kit) | LE.QCROBOTICS.1.0 |

## Contributing

We would love to have you as a part of the QRB ROS community. Whether you are helping us fix bugs, proposing new features, improving our documentation, or spreading the word, please refer to our [contribution guidelines](./CONTRIBUTING.md) and [code of conduct](./CODE_OF_CONDUCT.md).

- Bug report: If you see an error message or encounter failures, please create a [bug report](../../issues)
- Feature Request: If you have an idea or if there is a capability that is missing and would make development easier and more robust, please submit a [feature request](../../issues)

## Authors

* **Peng Wang** - *Maintainer* - [penww](https://github.com/penww)

See also the list of [contributors](https://github.com/your/project/contributors) who participated in this project.

## License

Project is licensed under the [BSD-3-clause License](https://spdx.org/licenses/BSD-3-Clause.html). See [LICENSE](./LICENSE) for the full license text.

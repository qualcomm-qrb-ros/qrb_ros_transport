# QRB ROS Transport

## Overview

[QRB ROS Transport](https://github.com/quic-qrb-ros/qrb_ros_transport) is designed for hardware-acceleration friendly transporting of messages on Qualcomm robotics platforms. It uses type adaption to make message data zero-copy between different ROS nodes, and different hardwares. It includes adapted types for Qualcomm robotics platforms.

[Type Adaptation Feature (REP 2007)](https://ros.org/reps/rep-2007.html) has enabled on ROS 2 Humble. This interface will allow us to define methods for serializing directly to the user requested type, and/or using that type in intra-process communication without ever converting it.

qrb_ros_transport is based on [dmabuf_transport](https://github.com/quic-qrb-ros/dmabuf_transport), it is open sourced and apply to all platforms based on Linux.

## System Requirements

- Linux kernel version 5.12 and later, for kernel dma-buf support.
- ROS 2 Humble and later, for type adaption support.

## Quickstart

1. Clone `dmabuf_transport` and this repository under `${QRB_ROS_WS}/src`

   ```bash
   cd ${QRB_ROS_WS}/src
   ```

   ```bash
   git clone https://github.com/quic-qrb-ros/dmabuf_transport.git
   ```

   ```
   git clone https://github.com/quic-qrb-ros/qrb_ros_transport.git
   ```

2. Add dependencies in your package.xml

   ```xml
   <depend>qrb_ros_transport</depend>
   ```
3. Add dependencies in your CMakeLists.txt

   ```cmake
   find_package(dmabuf_transport REQUIRED)
   find_package(qrb_ros_transport REQUIRED)

   ament_target_dependencies(${PROJECT_NAME}
      # ...
      dmabuf_transport
      qrb_ros_transport
   )
   ```

3. Using adapted types in your ROS node

   ```c++
   #include "qrb_ros_transport/type/image.hpp"

   auto typed_msg = std::make_shared<qrb_ros::transport::type::Image>();
   pub_.publish(typed_msg);
   ```

## Adapted Types

The following table lists current supported types:

| QRB ROS Transport Type          | ROS Interface           |
| ------------------------------- | ----------------------- |
| [qrb_ros::transport::type::Image](./include/qrb_ros_transport/type/image.hpp) | [sensor_msgs::msg::Image](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Image.msg) |

## Supported Platforms

This package is designed and tested to be compatible with ROS 2 Humble running on Qualcom RB3 gen2.

| Hardware                        | Software                |
| ------------------------------- | ----------------------- |
| RB3 gen2                        | LE.QCROBOTICS.1.0       |

## Resources

- [ROS2 Type Adaption](https://ros.org/reps/rep-2007.html)
- [Linux dma-buf documentation](https://docs.kernel.org/driver-api/dma-buf.html)
- [dmabuf_transport](https://github.com/quic-qrb-ros/dmabuf_transport)

## Contributions

Thanks for your interest in contributing to qrb_ros_transport! Please read our [Contributions Page](CONTRIBUTING.md) for more information on contributing features or bug fixes. We look forward to your participation!

## License

qrb_ros_transport is licensed under the BSD 3-clause "New" or "Revised" License. 

Check out the [LICENSE](LICENSE) for more details.

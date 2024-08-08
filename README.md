# QRB ROS Transport

## Overview

[QRB ROS Transport](https://github.com/quic-qrb-ros/qrb_ros_transport) is designed for hardware-acceleration friendly transporting of messages on Qualcomm robotics platforms. It uses type adaption to make message data zero-copy between different ROS nodes, and different hardwares. It includes adapted types for Qualcomm robotics platforms.

[Type Adaptation Feature (REP 2007)](https://ros.org/reps/rep-2007.html) has enabled on ROS 2 Humble. This interface will allow us to define methods for serializing directly to the user requested type, and/or using that type in intra-process communication without ever converting it.

qrb_ros_transport is based on [lib_mem_dmabuf](https://github.com/quic-qrb-ros/lib_mem_dmabuf), it is open sourced and apply to all platforms based on Linux.

## System Requirements

- Linux kernel version 5.12 and later, for kernel dma-buf support.
- ROS 2 Humble and later, for type adaption support.

## Code Sync and Build

Currently, we only support build with QCLINUX SDK.

1. Setup QCLINUX SDK environments follow this document: [Set up the cross-compile environment](https://docs.qualcomm.com/bundle/publicresource/topics/80-65220-2/develop-your-first-application_6.html?product=1601111740013072&facet=Qualcomm%20Intelligent%20Robotics%20(QIRP)%20Product%20SDK&state=releasecandidate)

2. Create `ros_ws` directory in `<qirp_decompressed_workspace>/qirp-sdk/`

     ```bash
     mkdir -p <qirp_decompressed_workspace>/qirp-sdk/ros_ws
     ```

3. Clone this repository and dependencies under `<qirp_decompressed_workspace>/qirp-sdk/ros_ws`

     ```bash
     cd <qirp_decompressed_workspace>/qirp-sdk/ros_ws
     git clone https://github.com/quic-qrb-ros/lib_mem_dmabuf.git
     git clone https://github.com/quic-qrb-ros/qrb_ros_imu.git
     git clone https://github.com/quic-qrb-ros/qrb_ros_transport.git
     ```

4. Build projects

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

## Zero Copy Transport with qrb_ros_transport

1. Add dependencies in your package.xml

   ```xml
   <depend>qrb_ros_transport</depend>
   ```
2. Use ament_cmake_auto to find dependencies in your CMakeLists.txt

   ```cmake
   find_package(ament_cmake_auto REQUIRED)
   ament_auto_find_build_dependencies()
   ```
3. Using adapted types in your ROS node

   ```c++
   #include "qrb_ros_transport/type/image.hpp"

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
| [qrb_ros::transport::type::Image](./include/qrb_ros_transport/type/image.hpp) | [sensor_msgs::msg::Image](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Image.msg) |
| [qrb_ros::transport::type::Imu](./include/qrb_ros_transport/type/imu.hpp) | [sensor_msgs::msg::Imu](https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/msg/Imu.msg) |

## Supported Platforms

This package is designed and tested to be compatible with ROS 2 Humble running on Qualcomm RB3 gen2.

| Hardware                                                     | Software          |
| ------------------------------------------------------------ | ----------------- |
| [Qualcomm RB3 gen2](https://www.qualcomm.com/developer/hardware/rb3-gen-2-development-kit) | LE.QCROBOTICS.1.0 |

## Resources

- [ROS2 Type Adaption](https://ros.org/reps/rep-2007.html): ROS 2 new feature to implement zero copy transport.
- [Linux dma-buf](https://docs.kernel.org/driver-api/dma-buf.html): Linux kernel subsystem for sharing buffers for hardware (DMA) access across multiple device drivers and subsystems, and for synchronizing asynchronous hardware access
- [lib_mem_dmabuf](https://github.com/quic-qrb-ros/lib_mem_dmabuf): Library for access and interact with Linux DMA heaps.

## Contributions

Thanks for your interest in contributing to qrb_ros_transport! Please read our [Contributions Page](CONTRIBUTING.md) for more information on contributing features or bug fixes. We look forward to your participation!

## License

qrb_ros_transport is licensed under the BSD 3-clause "New" or "Revised" License.

Check out the [LICENSE](LICENSE) for more details.

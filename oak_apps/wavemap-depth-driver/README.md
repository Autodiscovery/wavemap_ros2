# Wavemap Depth Driver — OAK 4 D Wide Standalone App

DepthAI v3 standalone app for the **OAK 4 D Wide** camera (RVC4) that publishes
depth images and RGB images over ROS 2 topics for use with wavemap's
`depth_image_topic` input.

## Published ROS 2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/oak/rgb/image_raw` | `sensor_msgs/msg/Image` | RGB camera stream |
| `/oak/rgb/camera_info` | `sensor_msgs/msg/CameraInfo` | RGB intrinsics (use for wavemap config) |
| `/oak/stereo/image_raw` | `sensor_msgs/msg/Image` | Aligned depth image (16UC1, mm) |
| `/oak/stereo/camera_info` | `sensor_msgs/msg/CameraInfo` | Depth intrinsics |
| `/oak/imu/data` | `sensor_msgs/msg/Imu` | IMU data |

## Deployment

### Prerequisites

- OAK 4 D Wide camera with RVC4 compute module
- `oakctl` CLI installed on the host: `pip install oakctl`
- Host and camera on the same network

### Configure Camera IP

The OAK camera's IP is managed via `oakctl`. To set or discover the IP:

```bash
# Discover cameras on the network
oakctl device list

# Connect to a specific camera by IP
oakctl connect <CAMERA_IP>
```

> **Note:** The camera IP is assigned by your network's DHCP server or can
> be configured as static via the RVC4's network settings through `oakctl`.

### Deploy & Run

```bash
cd oak_apps/wavemap-depth-driver
oakctl app run .
```

### Verify Topics on Host

```bash
source /opt/ros/jazzy/setup.bash
# If cross-distro (camera runs Kilted), use CycloneDDS:
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 topic list | grep oak
ros2 topic hz /oak/stereo/image_raw
```

## Wavemap Configuration

Use the camera intrinsics from `/oak/rgb/camera_info` to configure wavemap:

```bash
ros2 topic echo /oak/stereo/camera_info --once
```

Then update your wavemap YAML config's `projection_model` section with the
actual `fx`, `fy`, `cx`, `cy`, `width`, and `height` values.

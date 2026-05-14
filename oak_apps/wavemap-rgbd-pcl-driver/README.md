# Wavemap RGB-D Point Cloud Driver — OAK 4 D Wide Standalone App

DepthAI v3 standalone app for the **OAK 4 D Wide** camera (RVC4) that publishes
RGB-D point clouds over ROS 2 topics for use with wavemap's `pointcloud_topic`
input.

## Published ROS 2 Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/oak/rgbd/points` | `sensor_msgs/msg/PointCloud2` | Colored depth point cloud |

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
cd oak_apps/wavemap-rgbd-pcl-driver
oakctl app run .
```

### Verify Topics on Host

```bash
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ros2 topic list | grep oak
ros2 topic hz /oak/rgbd/points
```

## Wavemap Configuration

Use the point cloud topic directly in your wavemap config:

```yaml
inputs:
  - type: pointcloud_topic
    topic_name: /oak/rgbd/points
    topic_type: pointcloud2
    measurement_integrator_names:
      - oak_pointcloud
    max_wait_for_pose: 1.0
```

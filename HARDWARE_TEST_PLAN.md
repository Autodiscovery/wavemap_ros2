# Hardware-in-the-Loop Test Plan

**Wavemap ROS 2 Jazzy — Livox MID-360 + OAK 4 D Wide (RVC4)**

## Overview

This test plan validates the end-to-end functionality of the wavemap ROS 2 port using real hardware:

| Hardware | Role | Connection |
|----------|------|------------|
| **Livox MID-360** | 3D LiDAR point cloud input | Ethernet (192.168.123.120) |
| **OAK 4 D Wide** (RVC4) | Depth image + RGB point cloud input | USB-C / Ethernet via DepthAI v3 |
| **Host PC** | ROS 2 Jazzy node execution | Ubuntu 24.04 |

---

## 1. Prerequisites

### 1.1 System Setup

```bash
# ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Wavemap workspace
cd ~/wavemap_ws && source install/setup.bash

# CycloneDDS (recommended for cross-device topics)
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```

### 1.2 Livox MID-360 Setup

```bash
# Verify LiDAR connectivity
ping 192.168.123.120

# Launch Livox ROS 2 driver
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

**Expected topics:**
```
/livox/lidar    (sensor_msgs/msg/PointCloud2 or livox_ros_driver2/msg/CustomMsg)
/livox/imu      (sensor_msgs/msg/Imu)
```

### 1.3 OAK 4 D Wide (RVC4) Setup

The OAK camera runs DepthAI v3 as an on-device app using `oakctl`. Two driver configurations are available:

#### Option A: Basic Driver (RGB + Depth Image)

Based on [ros-driver-basic](https://github.com/luxonis/oak-examples/tree/main/apps/ros/ros-driver-basic):
```bash
cd oak-examples/apps/ros/ros-driver-basic
oakctl app run .
```

**Expected topics:**
```
/oak/rgb/image_raw       (sensor_msgs/msg/Image)
/oak/stereo/image_raw    (sensor_msgs/msg/Image)   ← depth input for wavemap
/oak/imu/data            (sensor_msgs/msg/Imu)
```

#### Option B: RGB Point Cloud Driver

Based on [ros-driver-rgb-pcl](https://github.com/luxonis/oak-examples/tree/main/apps/ros/ros-driver-rgb-pcl):
```bash
cd oak-examples/apps/ros/ros-driver-rgb-pcl
oakctl app run .
```

**Expected topics:**
```
/oak/rgbd/points    (sensor_msgs/msg/PointCloud2)   ← pointcloud input for wavemap
```

---

## 2. Test Cases

### TC-01: Build Verification

| Step | Action | Expected Result |
|------|--------|----------------|
| 1 | `colcon build` (clean) | All 5 wavemap packages build without errors |
| 2 | `colcon test --packages-select wavemap_ros2_conversions` | All 8 tests pass |
| 3 | `ros2 pkg list \| grep wavemap` | Lists: `wavemap`, `wavemap_msgs`, `wavemap_ros2_conversions`, `wavemap_ros2`, `wavemap_all` |

---

### TC-02: Livox MID-360 — Point Cloud Mapping

**Config:** `wavemap_livox_mid360.yaml`

| Step | Action | Expected Result |
|------|--------|----------------|
| 1 | Start Livox driver | `/livox/lidar` topic publishing at ~10 Hz |
| 2 | Start odometry source (e.g., FAST-LIO) | TF `odom → livox_frame` being published |
| 3 | `ros2 launch wavemap_ros2 livox_mid360.launch.py` | Node starts without errors |
| 4 | Wait 10 seconds | Map data being published on `/wavemap/map` |
| 5 | `ros2 topic echo /wavemap/map --once` | Valid `wavemap_msgs/msg/Map` message received |
| 6 | `ros2 topic hz /wavemap/map` | Publishing at configured rate (~0.5 Hz for `once_every: 2.0`) |
| 7 | `ros2 service call /wavemap/save_map wavemap_msgs/srv/FilePath "{file_path: '/tmp/test_map.wvm'}"` | Map file saved successfully |
| 8 | Verify file exists: `ls -la /tmp/test_map.wvm` | Non-zero file size |

**Pass criteria:** Map publishes continuously with no crashes for ≥ 60 seconds.

---

### TC-03: OAK 4 D Wide — Depth Image Mapping

**Config:** Custom `wavemap_oak4d_depth.yaml` (see below)

| Step | Action | Expected Result |
|------|--------|----------------|
| 1 | Start OAK basic driver: `oakctl app run .` | `/oak/stereo/image_raw` publishing |
| 2 | Verify depth topic: `ros2 topic hz /oak/stereo/image_raw` | Publishing at ≥ 15 Hz |
| 3 | Start TF publisher for camera extrinsics | TF `odom → oak_frame` available |
| 4 | Launch wavemap with depth config | Node starts, subscribes to depth image |
| 5 | Wait 10 seconds | Map data published on `/wavemap/map` |
| 6 | `ros2 topic echo /wavemap/pointcloud --once` | Valid PointCloud2 with occupied cells |

**Pass criteria:** Depth images integrated into map without errors for ≥ 60 seconds.

---

### TC-04: OAK 4 D Wide — Point Cloud Mapping

**Config:** Custom `wavemap_oak4d_pcl.yaml` (see below)

| Step | Action | Expected Result |
|------|--------|----------------|
| 1 | Start OAK RGB-PCL driver: `oakctl app run .` | `/oak/rgbd/points` publishing |
| 2 | Verify PCL topic: `ros2 topic hz /oak/rgbd/points` | Publishing at ≥ 10 Hz |
| 3 | Start TF publisher for camera extrinsics | TF `odom → oak_frame` available |
| 4 | Launch wavemap with PCL config | Node starts, subscribes to pointcloud |
| 5 | Wait 10 seconds | Map data published on `/wavemap/map` |
| 6 | Monitor CPU/memory: `top -p $(pgrep ros_server)` | CPU ≤ 200%, RSS ≤ 2 GB |

**Pass criteria:** Point clouds integrated into map without errors for ≥ 60 seconds.

---

### TC-05: Multi-Sensor Fusion (Livox + OAK)

**Config:** Custom `wavemap_mid360_oak4d.yaml` (see below)

| Step | Action | Expected Result |
|------|--------|----------------|
| 1 | Start Livox driver + odometry | `/livox/lidar` and TF publishing |
| 2 | Start OAK RGB-PCL driver | `/oak/rgbd/points` publishing |
| 3 | Launch wavemap with multi-sensor config | Node starts, subscribes to both inputs |
| 4 | Wait 30 seconds | Map contains data from both sensors |
| 5 | Save map and verify | File contains fused occupancy data |
| 6 | Monitor memory for 5 minutes | No memory leaks (RSS stable ± 10%) |

**Pass criteria:** Both sensor inputs fused into single map for ≥ 5 minutes.

---

### TC-06: Rosbag Record and Replay

| Step | Action | Expected Result |
|------|--------|----------------|
| 1 | Record 30s bag: `ros2 bag record /livox/lidar /oak/rgbd/points /tf /tf_static -d 30` | Bag directory created |
| 2 | Stop all live drivers | No topics publishing |
| 3 | Process bag: `ros2 launch wavemap_ros2 rosbag_processor.launch.py rosbag_path:=<bag_dir> config_file:=<config>` | Processor runs to completion |
| 4 | Verify output map | Non-empty map file produced |

**Pass criteria:** Rosbag processor completes without errors and produces a valid map.

---

### TC-07: Map Operations

| Step | Action | Expected Result |
|------|--------|----------------|
| 1 | Start wavemap with `crop_map` operation configured | Node starts |
| 2 | Move the robot > `radius` meters from origin | Blocks outside radius pruned |
| 3 | Verify `/wavemap/pointcloud` | Point cloud only contains nearby cells |
| 4 | Call `save_map` service | Map saved |
| 5 | Call `load_map` service with saved file | Map restored |

---

## 3. Example Configurations

### wavemap_oak4d_depth.yaml

```yaml
map:
  type: hashed_chunked_wavelet_octree
  min_cell_width: 0.02
  tree_height: 9

measurement_integrators:
  depth_camera:
    projection_model:
      type: pinhole_camera_projector
      width: 640
      height: 480
      fx: 500.0
      fy: 500.0
      cx: 320.0
      cy: 240.0
    integration_method:
      type: hashed_chunked_wavelet_integrator
      min_range: 0.1
      max_range: 5.0

inputs:
  - type: depth_image_topic
    topic_name: /oak/stereo/image_raw
    measurement_integrator_names:
      - depth_camera
    depth_scale_factor: 1000
    max_wait_for_pose: 1.0

map_operations:
  - type: publish_map
    once_every: 2.0
    topic: map
  - type: publish_pointcloud
    once_every: 2.0
    topic: pointcloud
```

### wavemap_oak4d_pcl.yaml

```yaml
map:
  type: hashed_chunked_wavelet_octree
  min_cell_width: 0.02
  tree_height: 9

measurement_integrators:
  oak_pointcloud:
    projection_model:
      type: spherical_projector
      elevation:
        num_cells: 480
        min_angle: -0.52
        max_angle: 0.52
      azimuth:
        num_cells: 640
        min_angle: -0.78
        max_angle: 0.78
    integration_method:
      type: hashed_chunked_wavelet_integrator
      min_range: 0.1
      max_range: 5.0

inputs:
  - type: pointcloud_topic
    topic_name: /oak/rgbd/points
    topic_type: pointcloud2
    measurement_integrator_names:
      - oak_pointcloud
    max_wait_for_pose: 1.0

map_operations:
  - type: publish_map
    once_every: 2.0
    topic: map
  - type: publish_pointcloud
    once_every: 2.0
    topic: pointcloud
```

### wavemap_mid360_oak4d.yaml

```yaml
map:
  type: hashed_chunked_wavelet_octree
  min_cell_width: 0.05
  tree_height: 9

measurement_integrators:
  lidar:
    projection_model:
      type: spherical_projector
      elevation:
        num_cells: 32
        min_angle: -0.13
        max_angle: 0.92
      azimuth:
        num_cells: 1800
        min_angle: -3.14159
        max_angle: 3.14159
    integration_method:
      type: hashed_chunked_wavelet_integrator
      min_range: 0.1
      max_range: 20.0
  depth_camera:
    projection_model:
      type: pinhole_camera_projector
      width: 640
      height: 480
      fx: 500.0
      fy: 500.0
      cx: 320.0
      cy: 240.0
    integration_method:
      type: hashed_chunked_wavelet_integrator
      min_range: 0.1
      max_range: 5.0

inputs:
  - type: pointcloud_topic
    topic_name: /livox/lidar
    topic_type: livox
    measurement_integrator_names:
      - lidar
    max_wait_for_pose: 1.0
  - type: depth_image_topic
    topic_name: /oak/stereo/image_raw
    measurement_integrator_names:
      - depth_camera
    depth_scale_factor: 1000
    sensor_frame_id: oak_frame
    max_wait_for_pose: 1.0

map_operations:
  - type: publish_map
    once_every: 2.0
    topic: map
  - type: publish_pointcloud
    once_every: 2.0
    topic: pointcloud
  - type: crop_map
    radius: 15.0
    body_frame: base_link
```

---

## 4. Pass / Fail Summary

| Test Case | Description | Status |
|-----------|-------------|--------|
| TC-01 | Build verification | ☐ |
| TC-02 | Livox MID-360 point cloud mapping | ☐ |
| TC-03 | OAK 4D depth image mapping | ☐ |
| TC-04 | OAK 4D point cloud mapping | ☐ |
| TC-05 | Multi-sensor fusion (Livox + OAK) | ☐ |
| TC-06 | Rosbag record and replay | ☐ |
| TC-07 | Map operations (crop, save, load) | ☐ |

---

## 5. Environment Notes

- **RMW**: CycloneDDS recommended for OAK camera cross-device topic discovery
- **OAK firmware**: DepthAI v3 via `oakctl` on RVC4 compute module
- **TF**: An external odometry source (e.g., FAST-LIO, visual odometry) is required for all tests
- **Camera calibration**: The pinhole parameters in the example configs are placeholders — replace with actual intrinsics from `/oak/rgb/camera_info`

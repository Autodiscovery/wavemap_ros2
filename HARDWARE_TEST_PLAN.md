# Hardware-in-the-Loop Test Plan

**Wavemap ROS 2 Jazzy — Livox MID-360 + OAK 4 D Wide (RVC4)**

## Overview

This test plan validates the end-to-end functionality of the wavemap ROS 2 port using real hardware:

| Hardware | Role | Connection | Default IP |
|----------|------|------------|-----------|
| **Livox MID-360** | 3D LiDAR point cloud input | Ethernet | `192.168.1.137` (LiDAR) / `192.168.1.50` (host) |
| **OAK 4 D Wide** (RVC4) | Depth image + RGB point cloud input | Ethernet / USB-C | Discovered via `oakctl` |
| **Host PC** | ROS 2 Jazzy node execution | Ubuntu 24.04 | — |

---

## 1. Network & IP Configuration

### 1.1 Livox MID-360 IP Configuration

The LiDAR IP is configured in `config/sensor_configs/MID360_config.json`:

```
src/wavemap/interfaces/ros2/wavemap_ros2/config/sensor_configs/MID360_config.json
```

**Key fields to edit:**

```jsonc
{
  "MID360": {
    "host_net_info": {
      "cmd_data_ip": "192.168.1.50",      // ← Host PC IP on the LiDAR subnet
      "push_msg_ip": "192.168.1.50",
      "point_data_ip": "192.168.1.50",
      "imu_data_ip": "192.168.1.50"
    }
  },
  "lidar_configs": [
    {
      "ip": "192.168.1.137"                // ← LiDAR IP address
    }
  ]
}
```

**To change the LiDAR IP** (e.g., to `192.168.123.120` on the Unitree subnet):

```bash
# Edit the config
nano src/wavemap/interfaces/ros2/wavemap_ros2/config/sensor_configs/MID360_config.json

# Change:
#   "ip": "192.168.123.120"          (LiDAR address)
#   All host IPs to "192.168.123.1"  (Host address on same subnet)
```

**Verify LiDAR connectivity:**

```bash
ping 192.168.1.137        # Or your configured LiDAR IP
```

### 1.2 OAK 4 D Wide Camera IP Configuration

The OAK camera with RVC4 runs as a standalone networked device managed by `oakctl`:

```bash
# Install oakctl
pip install oakctl

# Discover all cameras on the network
oakctl device list

# Connect to a specific camera
oakctl connect <CAMERA_IP>

# Check camera status
oakctl device info
```

**Network modes:**

| Mode | Description | Configuration |
|------|-------------|---------------|
| **DHCP** (default) | Camera gets IP from router/switch | Automatic — use `oakctl device list` to find |
| **Static IP** | Fixed IP on a specific subnet | Configure via `oakctl` device settings |
| **USB-C** | Direct USB connection | No IP needed — `oakctl` auto-detects |

**To set a static IP** (e.g., for `192.168.123.122`):

```bash
oakctl connect <current_ip>
oakctl device network set --ip 192.168.123.122 --netmask 255.255.255.0 --gateway 192.168.123.1
```

### 1.3 Host Network Configuration

Ensure the host PC has interfaces on both sensor subnets:

```bash
# Example: Add a static IP for the LiDAR subnet
sudo ip addr add 192.168.1.50/24 dev eth0

# Example: Add a static IP for the OAK camera subnet (if different)
sudo ip addr add 192.168.123.1/24 dev eth0

# Verify
ip addr show eth0
```

---

## 2. Prerequisites

### 2.1 System Packages

```bash
# ROS 2 Jazzy
source /opt/ros/jazzy/setup.bash

# Wavemap workspace
cd ~/wavemap_ws && source install/setup.bash

# CycloneDDS (required for cross-device/cross-distro ROS 2 topic discovery)
sudo apt install ros-jazzy-rmw-cyclonedds-cpp
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# OAK camera CLI
pip install oakctl
```

### 2.2 Deploy OAK Standalone Apps

Two pre-built standalone apps are provided in `oak_apps/`:

#### App 1: Depth + RGB Driver (for `depth_image_topic` input)

```bash
cd oak_apps/wavemap-depth-driver
oakctl app run .
```

Published topics:
- `/oak/rgb/image_raw` — RGB image
- `/oak/stereo/image_raw` — Aligned depth image (16UC1, mm scale)
- `/oak/stereo/camera_info` — Depth camera intrinsics
- `/oak/imu/data` — IMU data

#### App 2: RGB-D Point Cloud Driver (for `pointcloud_topic` input)

```bash
cd oak_apps/wavemap-rgbd-pcl-driver
oakctl app run .
```

Published topics:
- `/oak/rgbd/points` — Colored 3D point cloud

### 2.3 Launch Livox MID-360 Driver

```bash
# Verify connectivity
ping 192.168.1.137

# Launch
ros2 launch livox_ros_driver2 msg_MID360_launch.py
```

Published topics:
- `/livox/lidar` — `PointCloud2` or `CustomMsg`
- `/livox/imu` — IMU data

---

## 3. Test Cases

### TC-01: Build Verification

| Step | Action | Expected Result |
|------|--------|----------------|
| 1 | `colcon build` (clean) | All 5 wavemap packages build without errors |
| 2 | `colcon test --packages-select wavemap_ros2_conversions` | All 8 tests pass |
| 3 | `ros2 pkg list \| grep wavemap` | Lists: `wavemap`, `wavemap_msgs`, `wavemap_ros2_conversions`, `wavemap_ros2`, `wavemap_all` |

---

### TC-02: OAK Depth Driver Deployment

| Step | Action | Expected Result |
|------|--------|----------------|
| 1 | `oakctl device list` | Camera discovered with IP |
| 2 | `cd oak_apps/wavemap-depth-driver && oakctl app run .` | App builds and starts on camera |
| 3 | `ros2 topic list \| grep oak` | `/oak/rgb/image_raw`, `/oak/stereo/image_raw`, `/oak/imu/data` visible |
| 4 | `ros2 topic hz /oak/stereo/image_raw` | Publishing at ≥ 15 Hz |
| 5 | `ros2 topic echo /oak/stereo/camera_info --once` | Intrinsics (fx, fy, cx, cy) printed |

---

### TC-03: OAK Point Cloud Driver Deployment

| Step | Action | Expected Result |
|------|--------|----------------|
| 1 | `cd oak_apps/wavemap-rgbd-pcl-driver && oakctl app run .` | App builds and starts on camera |
| 2 | `ros2 topic list \| grep oak` | `/oak/rgbd/points` visible |
| 3 | `ros2 topic hz /oak/rgbd/points` | Publishing at ≥ 10 Hz |
| 4 | `ros2 topic echo /oak/rgbd/points --once \| head -5` | Valid PointCloud2 header |

---

### TC-04: Livox MID-360 — Point Cloud Mapping

**Config:** `wavemap_livox_mid360.yaml`
**LiDAR IP:** Edit `config/sensor_configs/MID360_config.json` → `lidar_configs[0].ip`

| Step | Action | Expected Result |
|------|--------|----------------|
| 1 | Start Livox driver | `/livox/lidar` topic publishing at ~10 Hz |
| 2 | Start odometry source (e.g., FAST-LIO) | TF `odom → livox_frame` being published |
| 3 | `ros2 launch wavemap_ros2 livox_mid360.launch.py` | Node starts without errors |
| 4 | Wait 10 seconds | Map data being published on `/wavemap/map` |
| 5 | `ros2 topic echo /wavemap/map --once` | Valid `wavemap_msgs/msg/Map` message received |
| 6 | `ros2 topic hz /wavemap/map` | Publishing at configured rate (~0.5 Hz) |
| 7 | `ros2 service call /wavemap/save_map wavemap_msgs/srv/FilePath "{file_path: '/tmp/test_map.wvm'}"` | Map file saved |
| 8 | `ls -la /tmp/test_map.wvm` | Non-zero file size |

**Pass criteria:** Map publishes continuously with no crashes for ≥ 60 seconds.

---

### TC-05: OAK 4 D Wide — Depth Image Mapping

**Config:** `wavemap_oak4d_depth.yaml` (see Section 5)
**Camera setup:** Deploy `wavemap-depth-driver` app first (TC-02)

| Step | Action | Expected Result |
|------|--------|----------------|
| 1 | Verify `/oak/stereo/image_raw` publishing | ≥ 15 Hz |
| 2 | Read intrinsics: `ros2 topic echo /oak/stereo/camera_info --once` | Copy fx/fy/cx/cy into config |
| 3 | Start TF publisher for camera extrinsics | TF `odom → oak-d-base-frame` available |
| 4 | Launch wavemap with depth config | Node starts, subscribes to depth image |
| 5 | Wait 10 seconds | Map data published on `/wavemap/map` |
| 6 | `ros2 topic echo /wavemap/pointcloud --once` | Valid PointCloud2 with occupied cells |

**Pass criteria:** Depth images integrated into map without errors for ≥ 60 seconds.

---

### TC-06: OAK 4 D Wide — Point Cloud Mapping

**Config:** `wavemap_oak4d_pcl.yaml` (see Section 5)
**Camera setup:** Deploy `wavemap-rgbd-pcl-driver` app first (TC-03)

| Step | Action | Expected Result |
|------|--------|----------------|
| 1 | Verify `/oak/rgbd/points` publishing | ≥ 10 Hz |
| 2 | Start TF publisher for camera extrinsics | TF `odom → oak-d-base-frame` available |
| 3 | Launch wavemap with PCL config | Node starts, subscribes to pointcloud |
| 4 | Wait 10 seconds | Map data published on `/wavemap/map` |
| 5 | Monitor CPU/memory: `top -p $(pgrep ros_server)` | CPU ≤ 200%, RSS ≤ 2 GB |

**Pass criteria:** Point clouds integrated into map without errors for ≥ 60 seconds.

---

### TC-07: Multi-Sensor Fusion (Livox + OAK)

**Config:** `wavemap_mid360_oak4d.yaml` (see Section 5)

| Step | Action | Expected Result |
|------|--------|----------------|
| 1 | Start Livox driver + odometry | `/livox/lidar` and TF publishing |
| 2 | Deploy OAK depth driver (TC-02) | `/oak/stereo/image_raw` publishing |
| 3 | Launch wavemap with multi-sensor config | Node subscribes to both inputs |
| 4 | Wait 30 seconds | Map contains data from both sensors |
| 5 | Save map and verify size | File contains fused occupancy data |
| 6 | Monitor memory for 5 minutes | No memory leaks (RSS stable ± 10%) |

**Pass criteria:** Both sensor inputs fused into single map for ≥ 5 minutes.

---

### TC-08: Rosbag Record and Replay

| Step | Action | Expected Result |
|------|--------|----------------|
| 1 | Record 30s bag: `ros2 bag record /livox/lidar /oak/rgbd/points /tf /tf_static -d 30` | Bag directory created |
| 2 | Stop all live drivers | No topics publishing |
| 3 | Process bag: `ros2 launch wavemap_ros2 rosbag_processor.launch.py rosbag_path:=<bag_dir> config_file:=<config>` | Processor runs to completion |
| 4 | Verify output map | Non-empty map file produced |

**Pass criteria:** Rosbag processor completes without errors and produces a valid map.

---

### TC-09: Map Operations

| Step | Action | Expected Result |
|------|--------|----------------|
| 1 | Start wavemap with `crop_map` operation configured | Node starts |
| 2 | Move the robot > `radius` meters from origin | Blocks outside radius pruned |
| 3 | Verify `/wavemap/pointcloud` | Point cloud only contains nearby cells |
| 4 | Call `save_map` service | Map saved |
| 5 | Call `load_map` service with saved file | Map restored |

---

## 4. IP Address Quick Reference

| Device | Config File | Field | Default |
|--------|------------|-------|---------|
| **Livox MID-360** (LiDAR IP) | `config/sensor_configs/MID360_config.json` | `lidar_configs[0].ip` | `192.168.1.137` |
| **Livox MID-360** (Host IP) | `config/sensor_configs/MID360_config.json` | `host_net_info.cmd_data_ip` (and others) | `192.168.1.50` |
| **OAK 4 D Wide** (Camera IP) | Managed by `oakctl` | `oakctl device list` / `oakctl connect` | DHCP assigned |
| **OAK 4 D Wide** (Static IP) | Managed by `oakctl` | `oakctl device network set --ip <IP>` | — |

---

## 5. Example Wavemap Configurations

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
      # Replace with actual intrinsics from /oak/stereo/camera_info
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
    depth_scale_factor: 1000       # OAK depth is in mm (16UC1)
    sensor_frame_id: oak-d-base-frame
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
    sensor_frame_id: oak-d-base-frame
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
      # Replace with actual intrinsics from /oak/stereo/camera_info
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
    sensor_frame_id: oak-d-base-frame
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

## 6. Pass / Fail Summary

| Test Case | Description | Status |
|-----------|-------------|--------|
| TC-01 | Build verification | ☐ |
| TC-02 | OAK depth driver deployment | ☐ |
| TC-03 | OAK point cloud driver deployment | ☐ |
| TC-04 | Livox MID-360 point cloud mapping | ☐ |
| TC-05 | OAK 4D depth image mapping | ☐ |
| TC-06 | OAK 4D point cloud mapping | ☐ |
| TC-07 | Multi-sensor fusion (Livox + OAK) | ☐ |
| TC-08 | Rosbag record and replay | ☐ |
| TC-09 | Map operations (crop, save, load) | ☐ |

---

## 7. Environment Notes

- **RMW**: CycloneDDS is **required** for OAK camera topic discovery (camera runs Kilted, host runs Jazzy)
- **OAK firmware**: DepthAI v3 via `oakctl` on RVC4 — apps are in `oak_apps/` directory
- **TF**: An external odometry source (e.g., FAST-LIO, visual odometry) is required for all mapping tests
- **Camera calibration**: The pinhole parameters in the example configs are placeholders — replace with actual intrinsics from `/oak/stereo/camera_info`
- **Livox driver**: The `livox_ros_driver2` package must be in the colcon workspace (included at `src/livox_ros_driver2`)

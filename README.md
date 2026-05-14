# Wavemap ROS 2

> **This project is a ROS 2 Jazzy port of [wavemap](https://github.com/ethz-asl/wavemap) by ETH Zürich ASL.**
> All credit for the core wavelet-based volumetric mapping framework goes to the original authors.
> Please cite their work when using this software for research (see [Citation](#citation) below).

<a href="https://github.com/ethz-asl/wavemap/blob/main/LICENSE"><img src="https://img.shields.io/badge/License-BSD%203-blue?logo=bsd" alt="License"/></a>
![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-blue?logo=ros&logoColor=white)

## Hierarchical, multi-resolution volumetric mapping for ROS 2

Wavemap achieves state-of-the-art memory and computational efficiency by combining Haar wavelet compression and a coarse-to-fine measurement integration scheme. Advanced measurement models allow it to attain exceptionally high recall rates on challenging obstacles like thin objects.

The framework is very flexible and supports several data structures, measurement integration methods, and sensor models out of the box. The ROS 2 interface can easily be configured to fuse multiple sensor inputs — such as a LiDAR configured with a range of 20m and several depth cameras up to a resolution of 1cm — into a single multi-resolution occupancy grid map.

### Supported Sensors

| Sensor | Input Type | Topic Type |
|--------|-----------|------------|
| **Livox MID-360** | `livox` / `pointcloud2` | `livox_ros_driver2::msg::CustomMsg` or `sensor_msgs::msg::PointCloud2` |
| **OAK-D (DepthAI v3)** | `depth_image` | `sensor_msgs::msg::Image` (depth) |
| **OAK-D (DepthAI v3)** | `pointcloud2` | `sensor_msgs::msg::PointCloud2` (RGB-D point cloud) |
| **Ouster OS0/OS1** | `ouster` / `pointcloud2` | `sensor_msgs::msg::PointCloud2` |
| **Any depth camera** | `depth_image` | `sensor_msgs::msg::Image` |

---

## Installation

### Prerequisites

- **Ubuntu 24.04** with [ROS 2 Jazzy](https://docs.ros.org/en/jazzy/Installation.html)
- `colcon` build tools: `sudo apt install python3-colcon-common-extensions`

### Build from Source

```bash
# Create workspace
mkdir -p ~/wavemap_ws/src && cd ~/wavemap_ws/src
git clone https://github.com/Autodiscovery/wavemap_ros2.git

# Install dependencies
cd ~/wavemap_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
source /opt/ros/jazzy/setup.bash
colcon build
source install/setup.bash
```

### Packages

| Package | Description |
|---------|-------------|
| `wavemap` | Ament wrapper for the core C++ library |
| `wavemap_msgs` | ROS 2 message and service definitions |
| `wavemap_ros2_conversions` | Map ↔ message conversion utilities |
| `wavemap_ros2` | Main mapping node, input handlers, and map operations |
| `wavemap_all` | Metapackage that installs everything |

---

## Quick Start

### Online Mapping with a LiDAR

```bash
# Start the wavemap server with a config file
ros2 launch wavemap_ros2 wavemap_server.launch.py \
    config_file:=$(ros2 pkg prefix wavemap_ros2)/share/wavemap_ros2/config/wavemap_livox_mid360.yaml
```

### Online Mapping with Livox MID-360 + Static TFs

```bash
ros2 launch wavemap_ros2 livox_mid360.launch.py
```

### Offline Rosbag Processing

```bash
ros2 launch wavemap_ros2 rosbag_processor.launch.py \
    config_file:=/path/to/config.yaml \
    rosbag_path:=/path/to/rosbag_directory
```

---

## Configuration

Wavemap is configured via YAML files. All parameters from the original wavemap are supported. Example configs are installed to `share/wavemap_ros2/config/`:

| Config File | Sensors |
|------------|---------|
| `wavemap_livox_mid360.yaml` | Livox MID-360 only |
| `wavemap_livox_mid360_azure_kinect.yaml` | MID-360 + Azure Kinect |
| `wavemap_livox_mid360_pico_flexx.yaml` | MID-360 + Pico Flexx |
| `wavemap_livox_mid360_pico_monstar.yaml` | MID-360 + Pico Monstar |
| `wavemap_ouster_os0.yaml` | Ouster OS0 only |
| `wavemap_ouster_os0_pico_monstar.yaml` | Ouster OS0 + Pico Monstar |
| `wavemap_ouster_os1.yaml` | Ouster OS1 only |
| `wavemap_panoptic_mapping_rgbd.yaml` | RGB-D (panoptic) |

### Example Config Structure

```yaml
map:
  type: hashed_chunked_wavelet_octree
  min_cell_width: 0.05

measurement_integrators:
  lidar:
    projection_model:
      type: spherical_projector
    integration_method:
      type: hashed_chunked_wavelet_integrator

inputs:
  - type: pointcloud_topic
    topic_name: /livox/lidar
    topic_type: livox
    measurement_integrator_names:
      - lidar

map_operations:
  - type: publish_map
    once_every: 2.0
    topic: map
```

---

## ROS 2 Topics & Services

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/wavemap/map` | `wavemap_msgs/msg/Map` | Serialized occupancy map |
| `/wavemap/pointcloud` | `sensor_msgs/msg/PointCloud2` | Occupied cells as a point cloud |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/wavemap/save_map` | `wavemap_msgs/srv/FilePath` | Save map to disk |
| `/wavemap/load_map` | `wavemap_msgs/srv/FilePath` | Load map from disk |

---

## Citation

This project is built on the wavemap framework by [ETH Zürich ASL](https://github.com/ethz-asl/wavemap). Please cite the original paper when using this software for research:

```bibtex
@INPROCEEDINGS{reijgwart2023wavemap,
    author = {Reijgwart, Victor and Cadena, Cesar and Siegwart, Roland and Ott, Lionel},
    journal = {Robotics: Science and Systems. Online Proceedings},
    title = {Efficient volumetric mapping of multi-scale environments using wavelet-based compression},
    year = {2023-07},
}
```

> Reijgwart, V., Cadena, C., Siegwart, R., & Ott, L. (2023). Efficient volumetric mapping of multi-scale environments using wavelet-based compression. *Proceedings of Robotics: Science and Systems XIX*. https://doi.org/10.15607/RSS.2023.XIX.065

<details>
<summary>Abstract</summary>
<br>
Volumetric maps are widely used in robotics due to their desirable properties in applications such as path planning, exploration, and manipulation. Constant advances in mapping technologies are needed to keep up with the improvements in sensor technology, generating increasingly vast amounts of precise measurements. Handling this data in a computationally and memory-efficient manner is paramount to representing the environment at the desired scales and resolutions. In this work, we express the desirable properties of a volumetric mapping framework through the lens of multi-resolution analysis. This shows that wavelets are a natural foundation for hierarchical and multi-resolution volumetric mapping. Based on this insight we design an efficient mapping system that uses wavelet decomposition. The efficiency of the system enables the use of uncertainty-aware sensor models, improving the quality of the maps. Experiments on both synthetic and real-world data provide mapping accuracy and runtime performance comparisons with state-of-the-art methods on both RGB-D and 3D LiDAR data. The framework is open-sourced to allow the robotics community at large to explore this approach.
</details>

---

## License

BSD 3-Clause — see [LICENSE](LICENSE) for details.

## Acknowledgements

- **Original wavemap framework**: [ethz-asl/wavemap](https://github.com/ethz-asl/wavemap)
- **Livox ROS 2 driver**: [Livox-SDK/livox_ros_driver2](https://github.com/Livox-SDK/livox_ros_driver2)
- **DepthAI ROS**: [luxonis/depthai-ros](https://github.com/luxonis/depthai-ros) and [oak-examples](https://github.com/luxonis/oak-examples)

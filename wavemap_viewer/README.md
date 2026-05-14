# Wavemap Viewer

A real-time 3D web viewer for wavemap occupancy maps over ROS 2.

## Features

- **Live Point Cloud** — Subscribes to `/wavemap/pointcloud` and renders in 3D
- **Save/Load Maps** — Calls wavemap ROS 2 services (`save_map`, `load_map`)
- **Color Modes** — Height gradient, occupancy, or solid color
- **Camera Presets** — Top-down, front, and free orbit
- **Zero Install** — Pure HTML/JS, no build step needed

## Quick Start

### 1. Install rosbridge

```bash
sudo apt install ros-jazzy-rosbridge-suite
```

### 2. Launch rosbridge

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### 3. Open the viewer

```bash
# Option A: Open directly in browser
firefox wavemap_viewer/index.html

# Option B: Serve with Python
cd wavemap_viewer
python3 -m http.server 8080
# Then open http://localhost:8080
```

### 4. Connect

1. Enter the rosbridge URL (default: `ws://localhost:9090`)
2. Click **Connect**
3. The viewer auto-subscribes to `/wavemap/pointcloud`

### 5. Save a Map

1. Enter the save path (e.g., `/tmp/my_map.wvm`)
2. Click **Save Map**
3. The viewer calls the `/wavemap/save_map` service

## Architecture

```
Browser (Three.js)  ←──WebSocket──→  rosbridge  ←──ROS 2──→  wavemap_ros2
                                                               ├─ /wavemap/pointcloud
                                                               ├─ /wavemap/save_map
                                                               └─ /wavemap/load_map
```

## Dependencies

All loaded from CDN — no `npm install` needed:
- [Three.js r128](https://threejs.org/)
- [OrbitControls](https://threejs.org/docs/#examples/en/controls/OrbitControls)
- [roslibjs 1.4.1](https://github.com/RobotWebTools/roslibjs)

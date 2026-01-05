# realsense-depth-human-detection-ros2
ROS 2 (Humble) Python package for Intel RealSense D456: real-time human detection + distance estimation using YOLO and depth frames. Includes Dockerfile for GPU-enabled laptops and launchable ROS2 node


**ROS 2 (Humble) Python package for Intel RealSense D456 — human detection + metric distance estimation (YOLOv11)**


> Minimal, modular perception node that reads color + depth from an Intel RealSense D456, runs YOLOv11 on RGB to detect people, samples depth at each detection to produce metric distances, and publishes typed ROS2 messages for downstream use (SLAM, planning, avoidance).


## Quick links
- Purpose: camera-first perception for people + distance.
- Docker base: `dustynv/ros:humble-desktop-l4t-r36.4.0` (Jetson-aware).
- Detector: YOLOv11 (Ultralytics API).


## Getting started (copy-paste)


1. Build Docker image (from repo root where `Dockerfile` is located):



# Build and run
```bash
docker build -t intel-depth-detection .


# on jetson
docker run -it --rm \
  --runtime nvidia \
  --network host \
  --privileged \
  -v /dev:/dev \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  intel-depth-detection

# or on non-jetson device

docker run -it --rm \
  --gpus all \
  --network host \
  --privileged \
  -v /dev:/dev \
  intel-depth-detection

# inside container (if not auto-sourced)

source /ros_entrypoint.sh
source /ros2_ws/install/setup.bash || true
cd /ros2_ws
colcon build --symlink-install
source install/setup.bash

# verify msg
ros2 interface show intel_depth_detection/msg/HumanDetection

# launch node
ros2 launch intel_depth_detection realsense_yolo.launch.py
# ros2 run intel_depth_detection realsense_node
## or

# inspect detections?
ros2 topic echo /intel_depth/human_detection
# or
# ros2 topic echo /intel_depth/humans -n 5



# logs?
rm -rf build install log

```

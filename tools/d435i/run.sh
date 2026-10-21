#!/usr/bin/env bash
# Build (first time) and start the ROS 2 container with GPU, USB and GUI access.
set -e
cd "$(dirname "$0")"

docker build -t mining-ros .
xhost +local:docker >/dev/null

docker run -it --rm --name mining-ros \
  --net=host --privileged --gpus all \
  -e DISPLAY="$DISPLAY" -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /dev:/dev \
  -v "$PWD":/ws \
  mining-ros

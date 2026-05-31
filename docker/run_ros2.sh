#!/bin/bash

# Build the image if it doesn't exist yet
if ! docker image inspect rover_image > /dev/null 2>&1; then
  echo "Building rover_image for the first time."
  docker build -t rover_image ~/Desktop/rover/docker/
fi

xhost +localhost
export DISPLAY=host.docker.internal:0

docker run -it --rm \
  --name rover_ros2 \
  -e DISPLAY=host.docker.internal:0 \
  -e ROS_DOMAIN_ID=42 \
  -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
  -v ~/Desktop/rover:/home/ubuntu/rover_ws \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  rover_image \
  bash
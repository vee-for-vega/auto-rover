FROM osrf/ros:humble-desktop-full

RUN apt-get update && apt-get install -y \
  ros-humble-rmw-cyclonedds-cpp \
  ros-humble-nav2-bringup \
  ros-humble-robot-localization \
  ros-humble-joy \
  && rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc
RUN echo "source /home/ubuntu/rover_ws/install/setup.bash 2>/dev/null || true" >> /root/.bashrc
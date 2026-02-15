# Minimal ROS2 reproducible environment
FROM ros:humble-ros-base@sha256:REPLACE_WITH_DIGEST

SHELL ["/bin/bash", "-c"]
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-rclcpp \
    ros-humble-rclpy \
    && rm -rf /var/lib/apt/lists/*
      

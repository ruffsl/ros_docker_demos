# This is a Dockerfile for ros:rviz
FROM ros:ros-nvidia
MAINTAINER ruffsl roxfoxpox@gmail.com

# install rviz
RUN apt-get update && apt-get install -y \
    ros-indigo-rviz \
    && rm -rf /var/lib/apt/lists/*

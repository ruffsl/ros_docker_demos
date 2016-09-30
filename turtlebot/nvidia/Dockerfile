# This is a Dockerfile for turtlebot_simulator
FROM osrf/ros:kinetic-desktop-full

# install turtlebot simulator
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-turtlebot* \
    && rm -rf /var/lib/apt/lists/*

# Getting models from[http://gazebosim.org/models/]. This may take a few seconds.
RUN gzserver --verbose --iters 1 /opt/ros/${ROS_DISTRO}/share/turtlebot_gazebo/worlds/playground.world

# install custom launchfile
ADD my_turtlebot_simulator.launch /

# Add Nvidia display support by including nvidia-docker hooks
LABEL com.nvidia.volumes.needed="nvidia_driver"
ENV PATH /usr/local/nvidia/bin:${PATH}
ENV LD_LIBRARY_PATH /usr/local/nvidia/lib:/usr/local/nvidia/lib64:${LD_LIBRARY_PATH}

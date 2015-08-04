# This is a Dockerfile for turtlebot_simulator
FROM ros:indigo
MAINTAINER ruffsl roxfoxpox@gmail.com

# install turtlebot simulator and teleop packages
RUN apt-get update && apt-get install -y \
    ros-indigo-turtlebot \
    ros-indigo-turtlebot-apps \
    ros-indigo-turtlebot-interactions \
    ros-indigo-turtlebot-simulator \
    ros-indigo-kobuki-ftdi \
    ros-indigo-rocon-remocon \
    ros-indigo-rocon-qt-library \
    ros-indigo-ar-track-alvar-msgs \
    && rm -rf /var/lib/apt/lists/*

# # install GUI packages
# RUN apt-get update && apt-get install -y \
#     binutils \
#     mesa-utils \
#     module-init-tools \
#     x-window-system\
#     && rm -rf /var/lib/apt/lists/*

# install nvidia drivers
ADD nvidia-driver.run /tmp/nvidia-driver.run
RUN sh /tmp/nvidia-driver.run -a -N --ui=none --no-kernel-module \
	&& rm /tmp/nvidia-driver.run

# install custom launchfile
ADD my_turtlebot_simulator.launch /tmp/my_turtlebot_simulator.launch

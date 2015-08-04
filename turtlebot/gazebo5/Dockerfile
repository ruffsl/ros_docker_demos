# This is a Dockerfile for gazebo6
FROM gazebo:gzserver6
MAINTAINER ruffsl roxfoxpox@gmail.com

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

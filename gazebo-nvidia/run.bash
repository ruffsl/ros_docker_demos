#!/bin/bash

xhost +local:root
GPU=0

nvidia-docker \
  run \
	-it \
	--env="DISPLAY" \
	--env="QT_X11_NO_MITSHM=1" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  gazebo:gazebo-nvidia gazebo

xhost -local:root

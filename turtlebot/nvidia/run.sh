xhost +local:root

# Run container with necessary Xorg and GPU mounts
nvidia-docker run -it \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --volume=/tmp/.X11-unix:/tmp/.X11-unix \
  ros:turtlebot-nvidia \
  roslaunch my_turtlebot_simulator.launch

xhost -local:root

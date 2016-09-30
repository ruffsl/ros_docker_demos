xhost +local:root

# Run container with necessary Xorg and DRI mounts
docker run -it \
  --env="DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --device=/dev/dri:/dev/dri \
  --volume=/tmp/.X11-unix:/tmp/.X11-unix \
  ros:turtlebot-intel \
  roslaunch my_turtlebot_simulator.launch

xhost -local:root

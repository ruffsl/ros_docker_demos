# This is a Dockerfile for ros:ros-openni
FROM ros:indigo-perception
MAINTAINER ruffsl roxfoxpox@gmail.com

# install packages
RUN apt-get update && apt-get install -q -y \
    ros-indigo-openni-launch

EXPOSE 11311
ENTRYPOINT ["bash", "-c"]
CMD ["bash"]

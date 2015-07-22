# Development Environment
In this tutorial , we'll go over how to use docker as a tool to build and share development environments. We'll use the official ROS Docker Hub image to create catkin workspace, build a demo project, and then commit and share our container.

## Workspace

To set up our catkin workspace, we'll want to start with a system already configured with a ROS installation.

> We'll just launch an interactive session using the official ROS image from Docker. We can download the image from the web and launch the container in one command:

    docker run -it --name=catkin_ws ros:indigo bash

> This direct your terminal to a bash session inside the container. This was done use the default entrypoint that sources the ROS's setup.bash. To build ROS packages and not just run them, we'll need install some basic compiler tools:

    apt-get update
    apt-get install -y build-essential

> We'll need to make and initialize our catkin workspace:

    mkdir -p ~/catkin_ws/src
    cd ~/catkin_ws/src
    catkin_init_workspace

> Then clone the tutorial package and build from source. We'll ignore the turtlesim for now as it requires additional build dependencies such Qt, not to mention other graphical runtime dependencies such as a running X server:

    git clone https://github.com/ros/ros_tutorials.git
    touch ros_tutorials/turtlesim/CATKIN_IGNORE

> Then just kick off the build process and watch it go:

    cd ~/catkin_ws/
    catkin_make

> Ok, lets add our catkin workspace setup to the entry point just below to line for ROS so we can easily launch our built binaries later:

    sed -i \
        '/source "\/opt\/ros\/$ROS_DISTRO\/setup.bash"/a source "\/root\/catkin_ws\/devel\/setup.bash"' \
        /ros_entrypoint.sh

## Committing
Now we can save the state of our container to an image for later use. This takes diff how the container has changed as compared to it's derived image.

> We can exit and commit our stoped container to an image with some notes on what we did:

    exit
    docker commit \
        --author="Dr. Foo Bar" \
        --message="My glorious build was successful!
        catkin workspace configured with built ros_tutorials" \
        catkin_ws \
        ros:indigo-foo-dev

> We can then see our image among our list stored by the docker daemon:

    docker images | grep indigo-foo-dev

> We can even introspect it further and examine the finer details about our image and its history:

    docker inspect ros:indigo-foo-dev

> Perhaps we should also check if our build is working before we try and share it. Go ahead and follow the multicontainer demo, but use the image we just created instead of the one that uses the debian packages. Come back here when you are finished and know everything is working.

## Sharing

Now that we've built and tested our development image, lets try and share it with others. We'll upload our image to Docker Hub, then simply tell our colleagues where to find it.

> To upload our image, we'll need to tag it with our username for Docker Hub's registry and then push. Lets use `foobar` here, but replace this with your own.

    docker tag ros:indigo-foo-dev foobar/ros:indigo-foo-dev
    docker push foobar/ros:indigo-foo-dev

> This will prompt you for your Docker Hub credentials, and then upload you image to a a tag named `indigo-foo-dev` under your own `ros` repository.

> Once we've uploaded our image, we can point people to the registry so they can pull, run and alter the images themselves and share fixes or improvements with you. Note however that images are layered diffs at the binary level, and can grow size on disk if many changes are committed and stacked over time.  

> So although images are handy to share runtime errors and patched fixes, and the diff you share are small, they're probably not something you'd should definitely hack on. Dockerfile defines how to cleanly build an image of your application, and should be the target from where to store your improvement once you've nailed them down.

> In this case, we would be able to link to our repo using the URL:

    https://hub.docker.com/u/foobar/ros

> or pull the latest image of the tag and  start a container directly using a similar docker command we started with

    docker pull foobar/ros:indigo-foo-dev
    docker
    docker run -it --name=catkin_ws foobar/ros:indigo-foo-dev
    docker bash

> And so the development cycle continues!

# Multi-Container ROS nodes
In this tutorial we'll use Docker to deploy ROS nodes onto a separate multiple containers on a single host and connect them together on a virtual network.

## Dependencies
Here is a breakdown of the dependencies across our demo application:

#### Local
* [Docker](https://www.docker.com/)
* [Docker Compose](https://docs.docker.com/compose/)

#### Image
* [ROS](https://registry.hub.docker.com/_/ros/)

## Setup

### Local
For our local setup, we'll need to install Docker along with the other Docker tools so that we can create and run multiple containers to deploy our setup. Fallow the links above for installation guides for you platform.

## Image
For our image setup, we'll need to build an image from the Docker Hub's official ROS repo to include the necessary demo packages we'll be using.
> From within this demo directory, we can build our needed demo images:

    docker build --tag ros:ros-tutorials ros-tutorials/.

## Deploy

### Creating a network
If we want our all ROS nodes to easily talk to each other, we'll can use a virtual network to connect the separate containers. Same is possible for connecting containers from separate hosts, this however is a bit more advanced and will addressed in a later tutorial.

> To create a new network `foo`, we use the network command:

    docker network create foo

Now that we have a network, we can create services. Services advertise there location on the network, making it easy to resolve the location/address of the service specific container. We'll use this make sure our ROS nodes can find and connect to our ROS `master`.

> To create a container for the ROS master and advertise it's service:

    docker run -it --rm\
        --publish-service=master.foo \
        --name master \
        ros:ros-tutorials \
        roscore

> Now you can see that master is running and is ready manage our other ROS nodes. To add our `talker` node, we'll need to point the relevant environment variable to the master service:

    docker run -it \
        --publish-service=talker.foo \
        --env ROS_HOSTNAME=talker \
        --env ROS_MASTER_URI=http://master:11311 \
        --name talker \
        ros:ros-tutorials \
        rosrun roscpp_tutorials talker

> Then in a new terminal, run the `listener` node similarly:

    docker run -it \
        --publish-service=listener.foo \
        --env ROS_HOSTNAME=listener \
        --env ROS_MASTER_URI=http://master:11311 \
        --name listener \
        ros:ros-tutorials \
        rosrun roscpp_tutorials listener

> Alright! You should see `listener` is now echoing each message the `talker` broadcasting. You can then list the containers and see something like this:

    $ docker service ls
    SERVICE ID          NAME                NETWORK             CONTAINER
    67ce73355e67        listener            foo                 a62019123321
    917ee622d295        master              foo                 f6ab9155fdbe
    7f5a4748fb8d        talker              foo                 e0da2ee7570a

> And for the services:

    $ docker ps
    CONTAINER ID        IMAGE               COMMAND                CREATED              STATUS              PORTS               NAMES
    a62019123321        ros:ros-tutorials   "/ros_entrypoint.sh    About a minute ago   Up About a minute   11311/tcp           listener
    e0da2ee7570a        ros:ros-tutorials   "/ros_entrypoint.sh    About a minute ago   Up About a minute   11311/tcp           talker
    f6ab9155fdbe        ros:ros-tutorials   "/ros_entrypoint.sh    About a minute ago   Up About a minute   11311/tcp           master

### Introspection
Ok, now that we see the two nodes are communicating, let get inside one of the containers and do some introspection what exactly the topics are:

    docker exec -it master bash
    source /ros_entrypoint.sh

> If we then use `rostopic` to list published message topics, we should see something like this:

    $ rostopic list
    /chatter
    /rosout
    /rosout_agg

### Tear down
To tear down the structure we've made, we just need to stop the containers and remove the services.

> We can stop and remove the containers using the names we gave them:

    docker stop master talker listener
    docker rm master talker listener

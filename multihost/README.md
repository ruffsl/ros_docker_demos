# Multi-Host ROS nodes
In this tutorial we'll use Docker to deploy ROS nodes onto a swarm of multiple seperate hosts and connect them together on a vertual network.

## Dependencies
Here is a breakdown of the dependencies across our demo application:

#### Local
* [Docker](https://www.docker.com/)
* [Docker Compose](https://docs.docker.com/compose/)
* [Docker Machine](https://docs.docker.com/machine/)
* [Docker Swarm](https://docs.docker.com/swarm/)
* [VirtualBox](https://www.virtualbox.org/)

## Setup

### Local
For our local setup, we'll need to install Docker along with the other Docker tools and VirtualBox so that we can create and run multiple docker engiens to deploy our multi-host setup. If you wish to run the demo on trulely isolated remotly accesable machines, you can change the driver from vertualbox to that for your prefered cloud provider, like AWS or Digital Ocean. The general instructions for this demo still hold, it is only out of conveniounce in creating a harmless learning enviorment that we use the VirtualBox driver for creating local VMs instead.

## Deployment
So get things started we'll use docker-machine to create our aws GPU instance for us with our desired configuration, and designating it as our swam master. Then we'll launch gzserver and gzweb on the remote instance and attach a new network to the running container. Once the server is running, we should be able point your web browser to the remote instance's external address and see our simulation's interface. Finally we'll add our local docker engine to the swarm cluster and we'll start gzclient in a locally running container attached to the same network allowing gzclient to connect to gzserver.

### Making a remote machine
We'll need to use our AWS credentials, so add them to your shell session as environmental variables:
```shell
export AWS_ACCESS_KEY_ID=####################
export AWS_SECRET_ACCESS_KEY=########################################
```

> Now create our AWS GPU instance and swarm master
* what region the VM should be started
 * `us-west-2`
* as well as what zone in the region's site (region specific)
 * `b`
* we'll specify what VM image to use (region specific). Use the aws image to enable graphical hardware acceleration
 * `ami-6dd8d85d`
* we'll specify what hardware to use (region specific). Use the GPU cluster for rendering images
 * `g2.2xlarge`
* security group same as default, docker-machine, but with added http=80 + gzweb=7681 inbound. Default being i.e. ssh=22 + dockerPort=2376 + swarmPort=3376 inbound
 * `sg-3515d051`
* Virtual Private Cloud network corresponding to the used security group
 * `vpc-e2eb6787`  

>Use docker_macine to make aws instance as swarm master

```shell
docker-machine -D create \
    --driver amazonec2 \
    --amazonec2-vpc-id vpc-722ea217 \
    --amazonec2-region us-west-2 \
    --amazonec2-zone b \
    --amazonec2-ami ami-77dbdb47 \
    --amazonec2-instance-type g2.2xlarge \
    --swarm \
    --swarm-master \
    swarm-master
```

### Starting gzserver and gzweb
Now we'll point docker client to the swarm master node:
```shell
eval "$(docker-machine env swarm-master)"
```
And then launch the gzserver and gzweb services using the compose file from inside this demo directory
```shell
docker-compose -f gzweb5Nvidia.yml up
```

### Loading gzweb
**TODO** For some reason, docker-machine always wants to create a security group, never use the one given to it. So just let it make it's own group named `Docker+Machine`, and then edit that security group from the AWS console to allow for the http=80 + gzweb=7681 inbound rules.

Then point your browser to the AWS external address.

### Creating a network

### Connecting our local machine

### Starting gzclient


## Tear down





> Create a `local` machine, this could represent our local workstation

    docker-machine create \
        -d virtualbox \
        local \
        --virtualbox-boot2docker-url=http://sirile.github.io/files/boot2docker-1.8.iso

> Now lets point our docker commands at this `local` docker engien and tell it to create a swarm

    eval "$(docker-machine env local)"
    docker run swarm create

> This will lastly print a token. Save it and use it to create our `swarm-master`

    export SID=<token-srting-here>
    docker-machine create \
        -d virtualbox \
        --virtualbox-boot2docker-url=http://sirile.github.io/files/boot2docker-1.8.iso \
        --swarm \
        --swarm-master \
        --swarm-discovery token://$SID \
        swarm-master

> Now we'll add two nodes to our swarm with the same token

    docker-machine create \
        -d virtualbox \
        --virtualbox-boot2docker-url=http://sirile.github.io/files/boot2docker-1.8.iso \
        --swarm \
        --swarm-discovery token://$SID \
        swarm-node-00

    docker-machine create \
        -d virtualbox \
        --virtualbox-boot2docker-url=http://sirile.github.io/files/boot2docker-1.8.iso \
        --swarm \
        --swarm-discovery token://$SID \
        swarm-node-01

> Ok, we finally have our massive swarm of 3 nodes and one local machine. Now lets direct our atention to the swarm master and see whats going on

    eval "$(docker-machine env --swarm swarm-master)"
    docker info
>


    docker network create foo

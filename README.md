# asclinic-system

Software for running the robot that is the focus of the Autonomous Systems Clinic subject.

# Docker instructions

Use the `ros:noetic-ros-core` image as the base. Because we need to run the
image as a single session, we'll need to install `tmux` too. To set up the image,
we'll build a docker image with that as the base. Run from the root of the repo:

```
docker build . -f Dockerfile -t ros-image
```

Then start up the image by using:

```
docker-compose run ros-runner
```

## Removing the image

```
docker-compose rm
```



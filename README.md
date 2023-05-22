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

## Running the type-hint remover

Python3.8, which is used by ROS1, isn't compatible with all of the type hint
options supported by more modern python. If these are used, you won't be able
to run your code. The `strip-hints` module is useful to remove these hints.

To run it, run `catkin_ws/src/asclinic_pkg/src/docker-strip-hints.bash` from
the directory that your files are in.

## Removing the image

```
docker-compose rm
```



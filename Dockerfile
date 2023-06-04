FROM ros:noetic-ros-base

# install required packages for compilation, and tmux
RUN apt update \
    && apt install -y \
        gpiod \
        libgpiod-dev \
        libgpiod-doc \
        ros-noetic-rosbridge-server \
        ros-noetic-tf \
        tmux \
    && apt clean \
    && rm -rf /var/lib/apt/lists/*

# auto-source the setup file
RUN echo "source /root/asclinic/catkin_ws/devel/setup.bash" >> ~/.bashrc

COPY strip-hints /root/strip-hints


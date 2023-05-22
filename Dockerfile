FROM ros:noetic-ros-core

RUN apt update && \
    apt install -y tmux && \
    apt clean && \
    rm -rf /var/lib/apt/lists/*



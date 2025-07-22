# Use official ROS 2 Humble base image with Python 3 support
FROM osrf/ros:humble-desktop-full

# Install system dependencies
RUN apt-get update && \
    apt-get install -y \
        python3-colcon-common-extensions \
        python3-pip \
        libtiff-dev \
        build-essential \
        libpython3-dev \
        net-tools \
        iproute2 \
        ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Install pysoem via pip (requires SOEM library)
RUN pip3 install pysoem

# Create a directory for the ROS package
WORKDIR /ros2_ws

# Copy your ROS 2 package(s) into the workspace
COPY ./bota_ethercat ./src

RUN . /opt/ros/humble/setup.sh && \
    colcon build

SHELL ["/bin/bash", "-c"]
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# Default command to source setup and run your node
CMD . /opt/ros/humble/setup.bash && \
    . /ros2_ws/install/setup.bash 

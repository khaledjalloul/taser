FROM nvidia/cuda:12.8.1-cudnn-runtime-ubuntu24.04

###############################################
# Initial Setup

ARG USERNAME
ENV USERNAME=${USERNAME}

# Set required environment variables and shell
ENV PLATFORM=amd64
ENV SHELL=/bin/bash
SHELL ["/bin/bash", "-c"]

# Set up docker user
RUN apt update && apt install -y sudo
RUN useradd -m --shell /bin/bash ${USERNAME}
RUN echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Install essentials
RUN apt update
RUN apt install -y \
    curl \
    nano \
    # Cmake
    build-essential \
    cmake \
    # C++
    gdb \
    # Git
    git \
    git-lfs \
    # GUI support
    libxrandr2

###############################################
# ROS 2 Jazzy

# Set UTF-8 locale
RUN apt update && apt install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Add ROS 2 Jazzy repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Jazzy
RUN apt update
RUN apt install -y software-properties-common
RUN add-apt-repository -y universe

ENV DEBIAN_FRONTEND=noninteractive
RUN apt install -y \
    ros-dev-tools \
    ros-jazzy-desktop \
    python3-colcon-clean

# Set up rosdep
RUN rosdep init

# Add ROS sourcing to bashrc
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/${USERNAME}/.bashrc

# Fix Rviz GUI 
ENV LIBGL_DRI3_DISABLE=1

###############################################
# Isaac Sim

# Copy Isaac Sim folder from its image because downloading and unzipping manually fails in docker due to large file size
RUN mkdir /workspaces && chown ${USERNAME} /workspaces

USER ${USERNAME}

COPY --from=nvcr.io/nvidia/isaac-sim:5.0.0 /isaac-sim /workspaces/isaacsim

# Create aliases for python and pip to use those of Isaac Sim
RUN printf ' \
    alias omni_python="/workspaces/isaacsim/python.sh" \n \
    alias isaacsim="/workspaces/isaacsim/isaac-sim.sh" \n \
    ' >> /home/${USERNAME}/.bashrc

###############################################
# Isaac Lab

ENV TERM=xterm
RUN git clone https://github.com/isaac-sim/IsaacLab.git /workspaces/isaaclab
RUN ln -s /workspaces/isaacsim /workspaces/isaaclab/_isaac_sim
RUN /workspaces/isaaclab/isaaclab.sh --install

###############################################
# Install dependencies

USER root

# Install Osqp and OsqpEigen
WORKDIR /tmp
RUN git clone --recursive --branch v0.6.3 https://github.com/osqp/osqp
RUN mkdir -p osqp/build
RUN cd osqp/build && cmake -G "Unix Makefiles" -DCMAKE_INSTALL_PREFIX:PATH=/usr ..
RUN cd osqp/build && make && make install

RUN git clone --branch v0.8.1 https://github.com/robotology/osqp-eigen.git
RUN mkdir -p osqp-eigen/build
RUN cd osqp-eigen/build && cmake -DCMAKE_INSTALL_PREFIX:PATH=/usr ..
RUN cd osqp-eigen/build && make && make install
ENV OsqpEigen_DIR=/usr

# Install rosdep dependencies to cache them in the image
RUN apt install -y \
    ros-jazzy-controller-manager \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-velocity-controllers \
    ros-jazzy-joint-state-broadcaster \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-robot-state-publisher \
    ros-jazzy-rviz2 \
    ros-jazzy-xacro

# Install personal CLI
RUN echo "deb [trusted=yes] https://khaledjalloul.github.io/debian-repository unstable main" | tee /etc/apt/sources.list.d/khaledjalloul.list
RUN apt update
RUN apt install -y khaled-cli

###############################################
# Install Python packages

RUN /workspaces/isaacsim/python.sh -m pip install --root-user-action=ignore --upgrade pip

USER ${USERNAME}

RUN /workspaces/isaacsim/python.sh -m pip install --no-input \
    cvxpy

###############################################
# Finalization

# Create the cache folders if not existing and handle correct permissions
RUN sudo mkdir -p /home/${USERNAME}/.cache && sudo chown -R ${USERNAME} /home/${USERNAME}/.cache
RUN sudo mkdir -p /workspaces/isaacsim/kit && sudo chown -R ${USERNAME} /workspaces/isaacsim/kit

WORKDIR /workspaces/taser_ws

# Copy the source code
COPY ./src/ /workspaces/taser_ws/src
RUN sudo chown -R ${USERNAME} .

# Install additional dependencies
RUN rosdep update
RUN . /opt/ros/jazzy/setup.bash && rosdep install --from-paths src --ignore-src -ry 

RUN echo "source /workspaces/taser_ws/install/local_setup.bash" >> /home/${USERNAME}/.bashrc

# RUN git clone https://github.com/isaac-sim/IsaacSim-ros_workspaces.git /workspaces/IsaacSim-ros_workspaces
# RUN cd /workspaces/IsaacSim-ros_workspaces/jazzy_ws && . /opt/ros/jazzy/setup.bash && rosdep install --from-paths src --ignore-src -ry 

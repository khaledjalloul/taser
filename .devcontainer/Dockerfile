ARG BASE_IMAGE=${BASE_IMAGE:-12.8.1-cudnn-runtime-ubuntu24.04}
ARG ISAAC_SIM_VERSION=${ISAAC_SIM_VERSION:-5.0.0}

FROM nvcr.io/nvidia/isaac-sim:${ISAAC_SIM_VERSION} AS isaac-sim

FROM ${BASE_IMAGE}

###############################################
# Initial Setup

ARG USERNAME
ENV USERNAME=${USERNAME}

ARG ROS_DISTRO=${ROS_DISTRO:-jazzy}
ENV ROS_DISTRO=${ROS_DISTRO}

ENV ISAAC_SIM_VERSION=${ISAAC_SIM_VERSION}

# Set required environment variables and shell
ENV PLATFORM=amd64
ENV SHELL=/bin/bash
SHELL ["/bin/bash", "-c"]

# Rename default ubuntu user to USERNAME only if USERNAME is set
RUN apt update && apt install -y sudo
RUN if [ -n "$USERNAME" ]; then \
    set -eux; \
    # Check if user 'ubuntu' exists and rename to $USERNAME if so
    if id -u ubuntu >/dev/null 2>&1; then \
    # Rename group
    groupmod -n "${USERNAME}" ubuntu \
    # Rename user, move home to match
    && usermod -l "${USERNAME}" -d "/home/${USERNAME}" -m ubuntu \
    # Ensure the user's primary group is the renamed one
    && usermod -g "${USERNAME}" "${USERNAME}" \
    # Fix sudoers/subuid/subgid if they reference 'ubuntu'
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers \
    && [ -f /etc/subuid ] && sed -i "s/^ubuntu:/${USERNAME}:/" /etc/subuid || true \
    && [ -f /etc/subgid ] && sed -i "s/^ubuntu:/${USERNAME}:/" /etc/subgid || true; \
    else \
    # Create the user if it does not exist yet
    useradd -m --shell /bin/bash ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers; \
    fi; \
    fi

# Install apt packages
RUN apt update
RUN apt install -y \
    # Essentials
    curl \
    nano \
    # Python
    python3-pip \
    python3-venv \
    # Cmake
    build-essential \
    cmake \
    # C++
    gdb \
    # Git
    git \
    git-lfs \
    # GUI support
    libxrandr2 \
    libxcb-cursor0

###############################################
# ROS 2

# Set UTF-8 locale
RUN apt update && apt install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Add ROS 2 repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2
RUN apt update
RUN apt install -y software-properties-common
RUN add-apt-repository -y universe

ENV DEBIAN_FRONTEND=noninteractive
RUN apt install -y \
    ros-dev-tools \
    ros-${ROS_DISTRO}-desktop \
    python3-colcon-clean

# Set up rosdep
RUN rosdep init

USER ${USERNAME:-root}

# Add ROS sourcing to bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ${HOME}/.bashrc

# Fix Rviz GUI 
ENV LIBGL_DRI3_DISABLE=1

###############################################
# Isaac Sim

# Create the workspaces folder with proper permissions
RUN sudo mkdir /workspaces
RUN if [ -n "$USERNAME" ]; then sudo chown -R ${USERNAME} /workspaces; fi

# Copy Isaac Sim folder from its image because downloading and unzipping manually fails in docker due to large file size
COPY --chown=${USERNAME:-root} --from=isaac-sim /isaac-sim /workspaces/isaacsim

# Create aliases for python and pip to use those of Isaac Sim
RUN printf ' \
    alias omni_python="/workspaces/isaacsim/python.sh" \n \
    alias isaacsim="/workspaces/isaacsim/isaac-sim.sh" \n \
    ' >> ${HOME}/.bashrc

###############################################
# Isaac Lab

ENV TERM=xterm
RUN git clone https://github.com/isaac-sim/IsaacLab.git /workspaces/isaaclab
RUN ln -s /workspaces/isaacsim /workspaces/isaaclab/_isaac_sim
RUN /workspaces/isaaclab/isaaclab.sh --install

###############################################
# Python Environment

# Upgrade omni_python pip
RUN sudo /workspaces/isaacsim/python.sh -m pip install --root-user-action=ignore --upgrade pip

# Upgrade install dependencies
RUN python3 -m pip install -U pip setuptools wheel build packaging scikit-build-core pyproject-metadata

USER ${USERNAME:-root}

# Unset python interactive startup script created by vscode
RUN echo "unset PYTHONSTARTUP" >> ${HOME}/.bashrc

###############################################
# Install Dependencies

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
    ros-${ROS_DISTRO}-controller-manager \
    ros-${ROS_DISTRO}-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
    ros-${ROS_DISTRO}-velocity-controllers \
    ros-${ROS_DISTRO}-joint-state-broadcaster \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-robot-state-publisher \
    ros-${ROS_DISTRO}-rviz2 \
    ros-${ROS_DISTRO}-xacro

# Install personal CLI
RUN echo "deb [trusted=yes] https://khaledjalloul.github.io/debian-repository unstable main" | tee /etc/apt/sources.list.d/khaledjalloul.list
RUN apt update
RUN apt install -y khaled-cli

###############################################
# Finalization

USER ${USERNAME:-root}
ENV WORKSPACE=/workspaces/taser
WORKDIR ${WORKSPACE}

# Install project dependencies
COPY --chown=${USERNAME:-root} ./pyproject.toml ${WORKSPACE}
COPY --chown=${USERNAME:-root} ./src/taser_cpp ${WORKSPACE}/src/taser_cpp
RUN mkdir -p ${WORKSPACE}/src/taser
RUN python3 -m pip install --verbose -e .

# Update environment variables for the Python bindings
ENV LD_LIBRARY_PATH=${LD_LIBRARY_PATH}:/home/kjalloul/.local/lib/python3.10/site-packages/lib
ENV CMAKE_PREFIX_PATH=${CMAKE_PREFIX_PATH}:/home/kjalloul/.local/lib/python3.10/site-packages

# RUN git clone https://github.com/isaac-sim/IsaacSim-ros_workspaces.git /workspaces/IsaacSim-ros_workspaces
# RUN cd /workspaces/IsaacSim-ros_workspaces/${ROS_DISTRO}_ws && . /opt/ros/${ROS_DISTRO}/setup.bash && rosdep install --from-paths src --ignore-src -ry 

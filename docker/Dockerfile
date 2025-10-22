ARG ISAACSIM_VERSION=${ISAACSIM_VERSION:-4.5.0}
FROM nvcr.io/nvidia/isaac-sim:${ISAACSIM_VERSION} AS base

###############################################
# Initial Setup

ARG USERNAME
ENV USERNAME=${USERNAME}

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
    nano \
    bash-completion \
    # Python
    python3-pip \
    # C++
    gdb \
    # Git
    git-lfs \
    # Python Qt GUI dependencies
    libxrandr2 \
    libxcb-cursor0

USER ${USERNAME:-root}

# Enable bash completion
RUN echo ". /etc/bash_completion" >> ${HOME}/.bashrc

###############################################
# Isaac Sim

RUN sudo mkdir /workspace
RUN if [ -n "$USERNAME" ]; then \
    sudo chown -R ${USERNAME} /isaac-sim; \
    sudo chown -R ${USERNAME} /workspace; \
    fi

# Create Isaac Sim workspace folder
RUN ln -s /isaac-sim /workspace/isaacsim

# Create aliases for omni_python and isaacsim
RUN printf '%s\n' \
    "alias omni_python=\"/workspace/isaacsim/python.sh\"" \
    "alias isaacsim=\"/workspace/isaacsim/isaac-sim.sh\"" \
    >> ${HOME}/.bashrc

###############################################
# Isaac Lab (Installed manually to get latest version)

ENV TERM=xterm
RUN git clone https://github.com/isaac-sim/IsaacLab.git /workspace/isaaclab
RUN ln -s /workspace/isaacsim /workspace/isaaclab/_isaac_sim
RUN /workspace/isaaclab/isaaclab.sh --install

###############################################
# Isaac Sim 4.5.0 Specific - Install ROS 2 Humble

FROM base AS isaacsim-4.5.0

USER root

# Set UTF-8 locale
RUN apt update && apt install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
ENV LANG=en_US.UTF-8

# Fix timezone to UTC
RUN ln -sf /usr/share/zoneinfo/Etc/UTC /etc/localtime
RUN echo "Etc/UTC" | tee /etc/timezone >/dev/null

# Add ROS 2 repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2
RUN apt update
RUN apt install -y software-properties-common
RUN add-apt-repository -y universe

ENV DEBIAN_FRONTEND=noninteractive
RUN apt install -y --allow-downgrades \
    # Fixes RVIZ installation
    libbrotli1=1.0.9-2build6 \ 
    ros-dev-tools \
    ros-humble-desktop \
    python3-colcon-clean

USER ${USERNAME:-root}

# Add ROS sourcing to bashrc
RUN echo "source /opt/ros/humble/setup.bash" >> ${HOME}/.bashrc

USER root

# Install rosdep dependencies
RUN rosdep init
COPY --chown=${USERNAME:-root} ./src/taser_ros/package.xml /workspace/taser/src/taser_ros/package.xml
RUN rosdep update
RUN ROS_DISTRO=humble rosdep install --from-paths /workspace/taser/src --ignore-src -r -y

# Create symlink for default python
RUN ln -sf /usr/bin/python3 /usr/bin/python

###############################################
# Isaac Sim 5.0.0 Specific

FROM base AS isaacsim-5.0.0

# Enable Isaac Sim internal rclpy library 
ENV ROS_DISTRO=humble
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/workspace/isaacsim/exts/isaacsim.ros2.bridge/humble/lib

# Create symlink for default python
RUN sudo ln -sf /workspace/isaacsim/kit/python/bin/python3 /usr/bin/python

# Install pybind11 for Isaac Sim's python 3.11 since the global python is 3.10
RUN python -m pip install pybind11
ENV pybind11_DIR=/workspace/isaacsim/kit/python/lib/python3.11/site-packages/pybind11/share/cmake/pybind11

###############################################
# Install Dependencies

ARG ISAACSIM_VERSION
FROM isaacsim-${ISAACSIM_VERSION} AS final

USER root

# Remove OS conflicting dependencies
RUN apt remove -y \
    python3-blinker \
    python3-matplotlib

# Install personal CLI
RUN echo "deb [trusted=yes] https://khaledjalloul.github.io/debian-repository unstable main" | tee /etc/apt/sources.list.d/khaledjalloul.list
RUN apt update
RUN apt install -y khaled-cli

###############################################
# Finalization

USER ${USERNAME:-root}
WORKDIR /workspace/taser

# Unset python interactive startup script created by vscode
RUN echo "unset PYTHONSTARTUP" >> ${HOME}/.bashrc

# Upgrade install dependencies
RUN python -m pip install -U pip setuptools wheel build packaging scikit-build-core pyproject-metadata

# Install project
COPY --chown=${USERNAME:-root} ./pyproject.toml /workspace/taser
COPY --chown=${USERNAME:-root} ./install.bash /workspace/taser
COPY --chown=${USERNAME:-root} ./src/taser_cpp /workspace/taser/src/taser_cpp
RUN bash ./install.bash

ENTRYPOINT [ ]

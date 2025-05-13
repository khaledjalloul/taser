FROM mcr.microsoft.com/vscode/devcontainers/base:ubuntu-24.04 AS ros-jazzy

ENV DEBIAN_FRONTEND=noninteractive

# Install ROS 2 Jazzy
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update
RUN apt install -y software-properties-common
RUN add-apt-repository -y universe
RUN apt install -y \
    ros-dev-tools \
    ros-jazzy-desktop \
    python3-colcon-clean

RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/vscode/.bashrc
RUN echo "source /workspace/colcon_ws/install/local_setup.bash" >> /home/vscode/.bashrc

# Install apt packages
RUN apt install -y \
    # Python
    python3.12 \
    python3-virtualenv \
    # C++
    cmake \
    clang-format \
    gdb \
    # GUI support
    libxrandr2

USER vscode

# Create Python virtual environment
RUN virtualenv /home/vscode/venv
RUN echo 'export PYTHONPATH=$PYTHONPATH:/home/vscode/venv/lib/python3.12/site-packages' >> /home/vscode/venv/bin/activate

# Install Python packages
RUN /home/vscode/venv/bin/pip install --no-input \
    cvxpy \
    matplotlib \
    numpy \
    # TODO: Find out why
    catkin_pkg

USER root

# Set up apt repositories
RUN echo "deb [trusted=yes] https://khaledjalloul.github.io/debian-repository unstable main" | tee /etc/apt/sources.list.d/khaledjalloul.list
RUN apt update

# Set up rosdep
RUN mkdir -p /etc/ros/rosdep/sources.list.d && echo "yaml file:///workspace/colcon_ws/src/rosdep.yaml" | tee /etc/ros/rosdep/sources.list.d/1-custom-deps.list
RUN rosdep init

# Install Osqp, OsqpEigen, and Clipper2
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

RUN git clone https://github.com/AngusJohnson/Clipper2.git
RUN mkdir -p Clipper2/CPP/build
RUN cd Clipper2/CPP/build && cmake .. && make && make install

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
RUN apt install -y khaled-cli

########################################

FROM nvidia/cuda:11.8.0-devel-ubuntu22.04 AS isaaclab

# Environment variables for Isaac Sim and Isaac Lab
ENV DEBIAN_FRONTEND=noninteractive
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV OMNI_KIT_ACCEPT_EULA=YES
ENV OMNI_KIT_ALLOW_ROOT=1
ENV TERM=xterm

# Install essentials
RUN apt update
RUN apt install -y \
    curl \
    sudo \
    # Python
    python3.10 \
    python3-virtualenv \
    # Cmake
    build-essential \
    cmake \
    # Git
    git \
    git-lfs

# Set up vscode user
RUN addgroup devcontainer
RUN useradd --gid devcontainer --uid 1000 -m -s /bin/bash vscode
RUN echo "vscode ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

USER vscode

# Create Python virtual environment
RUN virtualenv /home/vscode/venv
RUN echo 'export PYTHONPATH=$PYTHONPATH:/home/vscode/venv/lib/python3.10/site-packages' >> /home/vscode/venv/bin/activate

# Install Isaac Sim
RUN /home/vscode/venv/bin/pip install torch==2.5.1 torchvision==0.20.1 --index-url https://download.pytorch.org/whl/cu118
RUN /home/vscode/venv/bin/pip install 'isaacsim[all,extscache]==4.5.0' --extra-index-url https://pypi.nvidia.com

# Install Isaac Lab
RUN cd /home/vscode && git clone https://github.com/isaac-sim/IsaacLab.git
RUN . /home/vscode/venv/bin/activate && \
    /home/vscode/IsaacLab/isaaclab.sh --install

USER root

# Install apt packages
RUN echo "deb [trusted=yes] https://khaledjalloul.github.io/debian-repository unstable main" | tee /etc/apt/sources.list.d/khaledjalloul.list
RUN apt update
RUN apt install -y \
    khaled-cli \
    # C++
    clang-format \
    gdb \
    # GUI support
    libxrandr2 \
    # Libraries for Isaac Sim
    libgl1 \
    libglu1-mesa \
    libsm6 \
    libssl-dev \
    libxext6 \
    libxt6

USER vscode

# Install Python packages
RUN /home/vscode/venv/bin/pip install --no-input \
    cvxpy \
    matplotlib \
    numpy \
    # TODO: Find out why
    catkin_pkg

########################################

FROM isaaclab AS isaaclab-ros-humble

USER root

# Install ROS 2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

RUN apt update
RUN apt install -y software-properties-common
RUN add-apt-repository -y universe

RUN apt install -y \
    ros-humble-desktop \
    ros-dev-tools \
    python3-colcon-clean

RUN echo "source /opt/ros/humble/setup.bash" >> /home/vscode/.bashrc
RUN echo "source /workspace/colcon_ws/install/local_setup.bash" >> /home/vscode/.bashrc

# Set up rosdep
RUN mkdir -p /etc/ros/rosdep/sources.list.d && echo "yaml file:///workspace/colcon_ws/src/rosdep.yaml" | tee /etc/ros/rosdep/sources.list.d/1-custom-deps.list
RUN rosdep init

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
    ros-humble-controller-manager \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-velocity-controllers \
    ros-humble-joint-state-broadcaster \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-rviz2 \
    ros-humble-xacro

USER vscode

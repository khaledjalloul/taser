FROM mcr.microsoft.com/vscode/devcontainers/base:ubuntu-24.04

# Install ROS2
COPY install_ros2.bash /tmp/install_ros2.bash
RUN bash /tmp/install_ros2.bash
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/vscode/.bashrc
RUN echo "source /workspace/colcon_ws/install/setup.bash" >> /home/vscode/.bashrc

# Install general packages
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

# Install Python packages
RUN virtualenv /home/vscode/venv
RUN echo 'export PYTHONPATH=$PYTHONPATH:/home/vscode/venv/lib/python3.12/site-packages' >> /home/vscode/venv/bin/activate
RUN /home/vscode/venv/bin/pip install --no-input \
    cvxpy \
    ecos \
    sympy \
    matplotlib \
    numpy \
    casadi \
    # TODO: Find out why
    catkin_pkg

# Set up apt repositories
RUN echo "deb [trusted=yes] https://khaledjalloul.github.io/debian-repository unstable main" | tee /etc/apt/sources.list.d/khaledjalloul.list
RUN apt update

# Set up rosdep
RUN mkdir -p /etc/ros/rosdep/sources.list.d && echo "yaml file:///workspace/colcon_ws/src/rosdep.yaml" | tee /etc/ros/rosdep/sources.list.d/1-custom-deps.list
RUN rosdep init

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

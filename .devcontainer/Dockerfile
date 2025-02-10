FROM mcr.microsoft.com/vscode/devcontainers/base:ubuntu-24.04

RUN echo "deb [trusted=yes] https://khaledjalloul.github.io/debian-repository unstable main" \
    | tee /etc/apt/sources.list.d/khaledjalloul.list

COPY install_ros2.bash /tmp/install_ros2.bash
RUN bash /tmp/install_ros2.bash
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/vscode/.bashrc

RUN apt install -y \
    # Python
    python3.12 \
    python3-virtualenv \
    # C++
    cmake \
    clang-format \
    # GUI support
    libxrandr2

RUN apt install -y khaled-cli
RUN kj cli completion

USER vscode

RUN virtualenv /home/vscode/venv
ARG PIP=/home/vscode/venv/bin/pip

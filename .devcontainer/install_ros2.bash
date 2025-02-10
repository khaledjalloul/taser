# locale  # check for UTF-8

# apt update && apt install -y locales
# locale-gen en_US en_US.UTF-8
# update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
# export LANG=en_US.UTF-8

# locale  # verify settings

curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

apt update
apt upgrade -y

apt install -y software-properties-common
add-apt-repository universe -y

apt install -y ros-dev-tools ros-jazzy-desktop python3-colcon-clean

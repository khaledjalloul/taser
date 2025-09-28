# Create the cache folders if not existing and handle correct permissions
sudo mkdir -p ${HOME}/.cache/ov && sudo mkdir -p /workspaces/isaacsim/kit/cache
if [ -n "$USERNAME" ]; then \
    sudo chown -R ${USERNAME} ${HOME}/.cache/ov; \
    sudo chown -R ${USERNAME} /workspaces/isaacsim/kit/cache; \
    fi

# Set up CLI completion
kj setup cli completion

# Load environment variables
echo 'source ${WORKSPACE}/.env.local' >> ~/.bashrc

# Build the project
source /opt/ros/${ROS_DISTRO}/setup.sh \
&& colcon build \
&& echo "source ${WORKSPACE}/install/local_setup.bash" >> ${HOME}/.bashrc

# Simulation aliases
echo "alias sim_ros='ros2 launch taser_ros sim.launch.yaml'" >> ~/.bashrc
echo "alias sim_isaac='omni_python ${WORKSPACE}/src/taser/isaacsim/sim.py'" >> ~/.bashrc

# Isaac Lab aliases
echo "alias train='omni_python ${WORKSPACE}/src/taser/isaaclab/rl/custom/train.py'" >> ~/.bashrc
echo "alias play='omni_python ${WORKSPACE}/src/taser/isaaclab/rl/custom/play.py'" >> ~/.bashrc
echo "alias train_rsl='omni_python ${WORKSPACE}/src/taser/isaaclab/rl/rsl_rl/train.py'" >> ~/.bashrc
echo "alias play_rsl='omni_python ${WORKSPACE}/src/taser/isaaclab/rl/rsl_rl/play.py'" >> ~/.bashrc

# Other aliases
echo "alias update_urdf='xacro ${WORKSPACE}/src/taser_ros/robot_description/urdf/xacro/robot.urdf.xacro -o ${WORKSPACE}/src/taser_ros/robot_description/urdf/taser.urdf'" >> ~/.bashrc
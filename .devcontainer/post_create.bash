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
&& /workspaces/colcon_venv/bin/colcon build \
&& echo "source ${WORKSPACE}/install/local_setup.bash" >> ${HOME}/.bashrc

# Launch the simulation and state machine
echo "alias sim_ros='ros2 launch taser_ros sim.launch.yaml'" >> ~/.bashrc

# Run the Isaac Lab training script
echo "alias train='python ${WORKSPACE}/src/taser_isaaclab/taser_isaaclab/rl/train.py'" >> ~/.bashrc
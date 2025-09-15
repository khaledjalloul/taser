# Create the cache folders if not existing and handle correct permissions
sudo mkdir -p ${HOME}/.cache/ov && sudo mkdir -p /workspaces/isaacsim/kit/cache
if [ -n "$USERNAME" ]; then \
    sudo chown -R ${USERNAME} ${HOME}/.cache/ov; \
    sudo chown -R ${USERNAME} /workspaces/isaacsim/kit/cache; \
    fi

# Set up CLI completion
kj setup cli completion

# Add utils.bash to .bashrc
echo 'source ${WORKSPACE_PATH}/.devcontainer/utils.bash' >> ~/.bashrc

# Load environment variables
echo 'source ${WORKSPACE_PATH}/.env.local' >> ~/.bashrc

# Build the project
# source /opt/ros/${ROS_DISTRO}/setup.sh \
# && cd src/ros_ws \
# && colcon build \
# && echo "source ${WORKSPACE_PATH}/install/local_setup.bash" >> ${HOME}/.bashrc


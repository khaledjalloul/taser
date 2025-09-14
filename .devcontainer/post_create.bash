# Create the cache folders if not existing and handle correct permissions
sudo mkdir -p ${HOME}/.cache/ov && sudo mkdir -p /workspaces/isaacsim/kit/cache
if [ -n "$USERNAME" ]; then \
    sudo chown -R ${USERNAME} ${HOME}/.cache/ov; \
    sudo chown -R ${USERNAME} /workspaces/isaacsim/kit/cache; \
    fi

# Set up CLI completion
kj setup cli completion

# Add utils.bash to .bashrc
echo 'source /workspaces/taser_ws/.devcontainer/utils.bash' >> ~/.bashrc

# Load environment variables
echo 'source /workspaces/taser_ws/.env.local' >> ~/.bashrc

# Build the project
source /opt/ros/${ROS_DISTRO}/setup.sh \
&& colcon build --packages-ignore taser_isaaclab \
&& echo "source /workspaces/taser_ws/install/local_setup.bash" >> ${HOME}/.bashrc

/workspaces/isaacsim/python.sh -m pip install -e src/taser_isaaclab

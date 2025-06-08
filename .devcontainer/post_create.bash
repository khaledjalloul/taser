# Set up CLI completion
kj setup cli-completion

# Change ownership of the workspace to the devcontainer user
sudo chown -R $(whoami) .

# Change ownership of the omni cache used in Docker volume
sudo chown -R $(whoami) /home/$(whoami)/.local/lib/python3.10/site-packages/omni/cache

# Install new ROS dependencies in case of changes
# rosdep update
# rosdep install -iry --from-paths src

# Build the project again in case of changes
colcon build --packages-ignore taser_isaaclab

# Add utils.bash to .bashrc
echo 'source /home/$(whoami)/colcon_ws/.devcontainer/utils.bash' >> ~/.bashrc

# Load environment variables
echo 'source /home/$(whoami)/colcon_ws/.env.local' >> ~/.bashrc
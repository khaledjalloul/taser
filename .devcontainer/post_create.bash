# Set up CLI completion
kj setup cli-completion

# Install ROS dependencies
# rosdep update
# rosdep install -iry --from-paths src

# Build the project
colcon build --packages-ignore wheeled_humanoid_isaaclab
/home/vscode/venv/bin/pip install -e src/isaaclab

# Add utils.bash to .bashrc
echo "source /workspace/colcon_ws/.devcontainer/utils.bash" >> ~/.bashrc

# Load environment variables
echo "source /workspace/colcon_ws/.env.local" >> ~/.bashrc
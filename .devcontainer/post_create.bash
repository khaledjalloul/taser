# Set up CLI completion
kj setup cli completion

# Add utils.bash to .bashrc
echo 'source /workspaces/taser_ws/.devcontainer/utils.bash' >> ~/.bashrc

# Load environment variables
echo 'source /workspaces/taser_ws/.env.local' >> ~/.bashrc

# Build the project
source /opt/ros/jazzy/setup.sh && colcon build --packages-ignore taser_isaaclab
/workspaces/isaacsim/python.sh -m pip install -e src/taser_isaaclab

# Create URDF file
source /opt/ros/jazzy/setup.sh && xacro /workspaces/taser_ws/src/taser_ros/urdf/robot.urdf.xacro -o /tmp/taser/robot.urdf
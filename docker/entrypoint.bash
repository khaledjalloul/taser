# Fix correct permissions for cache folders
if [ -n "$USERNAME" ]; then
	sudo chown -R ${USERNAME} /isaac-sim/kit/cache
	sudo chown -R ${USERNAME} ${HOME}/.cache/ov
	sudo chown -R ${USERNAME} ${HOME}/.cache/pip
	sudo chown -R ${USERNAME} ${HOME}/.cache/nvidia
	sudo chown -R ${USERNAME} ${HOME}/.nv
	sudo chown -R ${USERNAME} ${HOME}/.nvidia-omniverse
	sudo chown -R ${USERNAME} ${HOME}/.local/share
	sudo chown -R ${USERNAME} ${HOME}/Documents
fi

# Set up CLI completion
kj setup cli completion

export WORKSPACE=/workspace/taser

# Build ROS packages if ROS is installed
if [ -f /opt/ros/humble/setup.bash ]; then
	source /opt/ros/humble/setup.sh &&
		colcon build &&
		echo "source ${WORKSPACE}/install/local_setup.bash" >>~/.bashrc
fi

# Simulation aliases
echo "alias sim_ros='ros2 launch taser_ros sim.launch.yaml'" >>~/.bashrc
echo "alias sim_isaac='omni_python ${WORKSPACE}/src/taser/isaacsim/sim.py'" >>~/.bashrc

# Isaac Lab aliases
echo "alias train='omni_python ${WORKSPACE}/src/taser/isaaclab/rl/custom/train.py'" >>~/.bashrc
echo "alias play='omni_python ${WORKSPACE}/src/taser/isaaclab/rl/custom/play.py'" >>~/.bashrc
echo "alias train_rsl='omni_python ${WORKSPACE}/src/taser/isaaclab/rl/rsl_rl/train.py'" >>~/.bashrc
echo "alias play_rsl='omni_python ${WORKSPACE}/src/taser/isaaclab/rl/rsl_rl/play.py'" >>~/.bashrc

# Other aliases
echo "alias update_urdf='xacro ${WORKSPACE}/src/taser_ros/robot_description/urdf/xacro/robot.urdf.xacro -o ${WORKSPACE}/src/taser_ros/robot_description/urdf/taser.urdf'" >>~/.bashrc
echo "alias update_usd='omni_python ${WORKSPACE}/src/taser/isaacsim/utils/urdf_to_usd.py'" >>~/.bashrc

# Execute the docker compose command
exec "$@"

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

export WORKSPACE=/workspace/taser

# Build ROS packages if ROS is installed (run in subshell to avoid ROS env contamination)
(
	if [ -f /opt/ros/jazzy/setup.bash ]; then
		source /opt/ros/jazzy/setup.sh &&
			colcon build &&
			echo "source ${WORKSPACE}/install/local_setup.bash" >>~/.bashrc
	fi
)

# Python aliases
echo "alias omni_python='/isaac-sim/python.sh'" >>~/.bashrc
echo "alias isaacsim='/isaac-sim/isaac-sim.sh'" >>~/.bashrc

# Set up taser CLI with completion
sudo ln -s /workspace/taser/src/taser/cli.py /usr/local/bin/taser
sudo chmod +x /usr/local/bin/taser
_TASER_COMPLETE=bash_source taser | sudo dd status=none of=/etc/bash_completion.d/taser

# Set up personal CLI completion
kj setup cli completion

# Unset python interactive startup script created by vscode
echo "unset PYTHONSTARTUP" >>~/.bashrc

# Execute the docker compose command
exec "$@"

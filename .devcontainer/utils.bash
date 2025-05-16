# Build a specific package
build() {
	local package="$1"

	colcon build --packages-select ${package}
}

# Test a specific package
test() {
	local package="$1"

	colcon build --packages-select ${package}
	colcon test --packages-select ${package} --event-handlers console_cohesion+
}

# Set state machine states manually
states() {
	local states="$1"

	ros2 service call /state_machine/set_states wheeled_humanoid_msgs/srv/SetStates "{states: ${states}}"
}

# Launch the simulation and state machine
alias launch='ros2 launch state_machine state_machine.launch.yaml'

# Spawn a random target in the simulation for the robot to go to and pick up
alias spawn='ros2 service call /spawn_random_target std_srvs/srv/Trigger'

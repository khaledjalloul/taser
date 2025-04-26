state() {
	local state="$1"

	ros2 service call /state_machine/set_state wheeled_humanoid_msgs/srv/SetState "{state: ${state}}"
}

build() {
	local package="$1"

	colcon build --packages-select ${package}
}

test() {
	local package="$1"

	colcon test --packages-select ${package} --event-handlers console_cohesion+
}

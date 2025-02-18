state() {
	local state="$1"

	ros2 service call /state_machine/set_state wheeled_humanoid_msgs/srv/SetState "{state: ${state}}"
}

# Wheeled Humanoid

## Components

- Simulation Environment: Rviz -> (TODO) Isaac Sim
- Arms: 3 DOF
  - (TODO) Navigation: RRT*
  - Controller: PID with inverse kinematics
- Base:
  - Navigation: RRT*
  - Controller: MPC
- State Machine
- (TODO) Sensing:
  - Detecting obstacles: ?
  - SLAM: ?
  - State estimation: ? (maybe not needed in sumulation)

## ROS Packages / Nodes

- wheeled_humanoid: Robot logic (ROS-independent)
- wheeled_humanoid_py: Python bindings of the `wheeled_humanoid` package
- wheeled_humanoid_ros: ROS interface, URDF, Rviz, launch files
- wheeled_humanoid_msgs: Custom ROS messages
- state_machine: States and missions


## To Do

- [x] Learn what /tf and robot state publisher are and how to use them
- [x] Create URDF file with ROS control and set up Rviz
- [x] Test arm to decide if 2 or 3 DOF and which DOF -> 3: y, z, y
- [x] Reuse sketch follower to control one arm, keep everything the same just refactor the code a bit
- [x] Create state machine package and create action to trigger arm motion
- [x] Create base MPC controller -> Very flaky for now
- [x] Create RRT* path planner (no obstacles)
- [x] Integrate path planner and base controller with state machine
- [ ] Improve base controller to be more robust
- [ ] Test RRT* given obstacles in the simulation
- [ ] Create Python binder to be able to use robot logic in Isaac Sim
- [ ] Add path planning to the arms to avoid obstacles
- [ ] Create missions for the robot to go to a location and pick up object
- [ ] Implement sensing such as a lidar
- [ ] SLAM?
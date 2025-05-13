# Wheeled Humanoid

## Components

- Simulation Environment: Rviz -> (TODO) Isaac Sim
- Arms: 3 DOF
  - (TODO) Navigation: RRT*
  - Position Controller: PID with inverse kinematics
- Base:
  - Navigation: RRT*
  - Tracking Controller: MPC
- State Machine
- (TODO) Sensing:
  - Detecting obstacles: ?
  - SLAM: ?
  - State estimation: ? (maybe not needed in sumulation)

## ROS Packages

- core (wheeled_humanoid): Robot logic (ROS-independent)
- core_py (wheeled_humanoid_py): Python bindings of the core package
- core_ros (wheeled_humanoid_ros): ROS interface, URDF, Rviz, launch files
- isaaclab: Isaac Lab environment for RL training
- msgs (wheeled_humanoid_msgs): Custom ROS messages
- state_machine (wheeled_humanoid_state_machine): States and missions


## To Do

- [x] Learn what /tf and robot state publisher are and how to use them
- [x] Create URDF file with ROS control and set up Rviz
- [x] Test arm to decide if 2 or 3 DOF and which DOF -> 3: y, z, y
- [x] Reuse sketch follower to control arms, keep everything the same just refactor the code a bit
- [x] Create state machine package and create action to trigger arm motion
- [x] Create base MPC controller -> Very flaky for now
- [x] Create RRT* path planner (no obstacles)
- [x] Integrate path planner and base controller with state machine
- [x] Create Python bindings to be able to use robot logic in Isaac Sim
- [x] Improve base controller to be more robust
- [x] Start with Isaac Lab environment
- [x] Inflate RRT* obstacles and test in simulation
- [ ] Create missions for the robot to go to a location and pick up object
- [ ] Add path planning to the arms to avoid obstacles or train with RL
- [ ] Improve RRT* to use Dubins paths
- [ ] Enable gravity and train robot to stay in balance

## To Do (Extra)

- [ ] Implement sensing such as a lidar
- [ ] SLAM?
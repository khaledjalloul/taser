# Wheeled humanoid

- Torso with arms and wheels
- In the beginning 3 wheels then maybe try to balance on 2

## Modules

- simulation (gazebo, isaac sim?)
- navigation and obstacle avoidance for the base
  - rrt*
- control of the wheels
  - mpc
  - state: current velocity/acceleration of the wheels + location of the robot
  - input: acceleration of the wheels
  - cost: position of the robot wrt the planned path + maybe terminal velocity
  - constraints: dynamics of the wheels
- navigation of the arms:
  - rrt*
- control of the arms
  - pid or mpc
  - 2 or 3 dof tbd
  - already implemented just modify kinematics
- sensing:
  - detecting obstacles
  - slam
  - state estimation, maybe not needed in sumulation

## Steps

- [x] Learn what is /tf and robot state publisher and how i can use them
- [x] Test arm to decide if 2 or 3 dof and which dof: 3 - y, z, x
- [ ] prepare ros packages
- [ ] reuse sketch follower to control one arm, keep everything the same for now just refactor the code a bit
- [ ] set up simulation, environment, obstacles, and assume that sensing is provided (robot already knows where everything is and where it is in the environment)
- [ ] test RRT* given the obstacles in simulation
- [ ] design base with 3 wheels
- [ ] design base mpc controller
- [ ] test base mpc with rrt path in the environment
- [ ] attach both arms to the base and test them with control
- [ ] add path planning to the arms to be able to pick up objects
- [ ] create missions for the robot to go to a location and pick up object (assume robot knows where objects are and that they are not obstacles)
- [ ] remove hard-coded sensing, add sensors and implement slam maybe?

## ROS Packages

- simulation and environment
- base path planner
- base controller, subscribes to observation and publishes input
- arm path planners
- arm controllers
- "sensing", publishes or idk where obstacles and objects are
- missions state machine
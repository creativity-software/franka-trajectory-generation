** Run controller **
roslaunch trajectory_gen cartesian_impedance_example_controller.launch robot:=panda robot_ip:=192.168.3.8

use the one above! check launch file and config folder in the trajectory_gen pkg

**  rosbag  **
rosbag record rosout /franka_state_controller/F_ext /cartesian_impedance_example_controller/equilibrium_pose



rostopic pub /franka_gripper/grasp/goal franka_gripper/GraspActionGoal "header:
  seq: 1
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
goal_id:
  stamp:
    secs: 0
    nsecs: 0
  id: ''
goal:
  width: 0.0
  epsilon:
    inner: 0.0
    outer: 0.0
  speed: 0.5
  force: 1.0" 



z value table: 0.08398204667844711
cartesian initial position: 0.4177027570738568, -0.01468797204056722, 0.42149735777916353 



TODO:

- add close gripper in node
- add mass estimation



roslaunch franka_example_controllers cartesian_impedance_example_controller.launch \
  robot_ip:=192.168.3.8 load_gripper:=true robot:=panda rviz:=false interactive_marker:=false

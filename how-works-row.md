# Useful resources

[Eingen Library](https://eigen.tuxfamily.org/index.php?title=Main_Page)


## How to run gazebo 

1. `catkin_make` - build your binaries
2. `rosrun trajectory_gen trajectory_gen_node` - run built trajectory in the node 
3. `rostopic echo /cartesian_impedance_example_controller/equilibrium_pose` - check logs from 
# Useful resources
[Eingen Library](https://eigen.tuxfamily.org/index.php?title=Main_Page)


## How to run
1. `catkin_make` - build your binaries
2. `rosrun trajectory_gen trajectory_gen_node` - run built trajectory in the node 
3. `rostopic echo /cartesian_impedance_example_controller/equilibrium_pose` - check logs from 
4. `rostopic info /cartesian_impedance_example_controller/equilibrium_pos` - check the publishers and subscribers of the gezebo
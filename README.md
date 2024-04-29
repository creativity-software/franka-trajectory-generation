running:

roslaunch franka_gazebo panda.launch x:=-0.5 \
    world:=$(rospack find demo_pkg)/world/pick_place.sdf \
    controller:=cartesian_impedance_example_controller \
    rviz:=true
    
    


Creator and manteiner: Alessandro Melone


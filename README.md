# New documentation

New documentation has been created as `.media/Trajectory-Linear-Circular.pdf`. Please refer ther for a more in depth introduction to the code and project. 

## Some useful commands

You can find some useful commands here `./how-to-run.md`. Also, `./commands.md` has some useful utilities for working with real robot.

#  Readme part from Professor


```sh
git clone git@gitlab.lrz.de:00000000014B74D8/assignment-2-group-1.git
git checkout main
```

## Setup
Modify the `.env` file replacing
- `<username>` with your username (i.e., the output of `whoami` in your terminal) in `USERNAME=<username>`
- `<username>` with your username in `ROS_WS=/home/<username>/ros_ws` (`ROS_WS` will be the path of your ROS2 workspace in the container)

> Reason: these variables are used to setup the container user with same credential of the host (by default the container would run as superuser). Among several benefits, this implies that all the files generated by the container have the same ownership of the one generated by the host. For further info, refer to this [guide][dockerROS2guide].   

At the first usage, build the docker image using the following command in the project folder:
```sh
./setup_docker.bash
```

## Usage
Open a terminal in the project folder and run:
```sh
docker compose up
```
In some systems instead of using `docker compose` you have to use `docker-compose` command. Change it in the `setup_docker.bash` as well.

Now a docker container is running on your machine. To access the container, open another terminal and run:
```sh
docker compose exec -it gazebo_franka_ros1-ros1_node-1 bash
```
> this is the equivalent of opening a terminal on your Ubuntu installation but on the Ubuntu installation running in the container. 

where `gazebo_franka_ros1-ros1_node-1` is the name of the container you want to access. To see the names of all the container running on your machine, you can use `docker ps`.

### First Usage
Access a running container and use the following command to initialize your catkin_ws:
```sh 
catkin_make
```

## Start Gazebo simulation with Franka Robot arm
Start simulation with robot arm:
```sh
roslaunch franka_gazebo panda.launch x:=-0.5 \
    controller:=cartesian_impedance_example_controller \
    use_gripper:=false\
    rviz:=true
```
Start simulation with robot arm with pick-and-place setup:
```sh
roslaunch franka_gazebo panda.launch x:=-0.5 \
    world:=$(rospack find demo_pkg)/world/pick_place.sdf \
    controller:=cartesian_impedance_example_controller \
    rviz:=true
```
Start simulation using impedance controller:
```
roslaunch franka_gazebo panda.launch \
    world:=$(rospack find demo_pkg)/world/polish.sdf \
    controller:=cartesian_impedance_example_controller \
    use_gripper:=false \
    rviz:=false \
    interactive_marker:=false

```

## Build a package
In src folder, run:
```sh
catkin_make 
```
## Troubleshooting



## License

MIT © [Alessandro Melone](https://alessandromelone.github.io/) alessandro.melone@tum.de

[//]: # (These are reference links used in the body of this note and get stripped out when the markdown processor does its job. There is no need to format nicely because it shouldn't be seen. Thanks SO - http://stackoverflow.com/questions/4823468/store-comments-in-markdown-syntax)

   [git-repo-tinyfsm]: <https://github.com/joemccann/dillinger>

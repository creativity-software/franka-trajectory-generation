#!/bin/bash
# Basic entrypoint for ROS / Colcon Docker containers

# Source ROS 1
source /opt/ros/${ROS_DISTRO}/setup.bash
echo "Sourced ROS 1 ${ROS_DISTRO}"

# Source the base workspace, if built
echo "trying to source ${ROS_WS}/devel/setup.bash"
if [ -f ${ROS_WS}/devel/setup.bash ]
then
  source ${ROS_WS}/devel/setup.bash
#  export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/models
  echo "Sourced ros_ws base workspace"
fi

export PATH="$PATH:/home/tueirsi-mia-n42/.local/bin"

# Execute the command passed into this entrypoint
exec "$@"

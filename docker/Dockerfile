# ROS distribution to use
ARG ROS_DISTRO=noetic

FROM osrf/ros:${ROS_DISTRO}-desktop-full-focal
ENV ROS_DISTRO=${ROS_DISTRO}
SHELL ["/bin/bash", "-c"]

ARG USERNAME=${USERNAME}
ARG ROS_WS=${ROS_WS}
# ARG ROS_WS=/home/${USERNAME}

RUN apt-get update && apt-get install -y --no-install-recommends lsb-release wget gnupg\
&& sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' \
&& wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add - \
&& sudo apt-get update \
&& apt-get install -y --no-install-recommends ignition-citadel

# Install libfranka and Gazebo
RUN apt-get update && apt-get install -y --no-install-recommends \
ros-noetic-libfranka ros-noetic-franka-ros \
&& rm -rf /var/lib/apt/lists/*

# Create workspace with external dependencies
RUN mkdir -p ${ROS_WS}/src
WORKDIR ${ROS_WS}/src

# Build the base Colcon workspace, installing dependencies first.
WORKDIR ${ROS_WS}
RUN apt-get update -y \
 && apt-get install -y --no-install-recommends \
  python3-pip 
#  && MAKEFLAGS=-2 colcon build --parallel-workers=2 --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release

RUN apt-get update && apt-get install -y --no-install-recommends nano


# Dev container arguments
ARG UID=${UID:-1000}
ARG GID=${UID:-1000}

# Create new user and home directory
RUN groupadd --gid $GID $USERNAME \
 && useradd --uid ${GID} --gid ${UID} --create-home ${USERNAME} \
 && echo ${USERNAME} ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME} \
 && mkdir -p /home/${USERNAME} \
 && chown -R $USERNAME:$USERNAME /home/${USERNAME} 
# Set the ownership of the overlay workspace to the new user

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
 && catkin_make

COPY ./docker/entrypoint.sh /home/${USERNAME}/
# ENV my_user=$USERNAME
# ENTRYPOINT source /home/$my_user/entrypoint.sh
#sleep infinity does not work in the compose

# Set the user and source entrypoint in the user's .bashrc file
USER ${USERNAME}
ENV ROS_WS=${ROS_WS}
ENV XDG_RUNTIME_DIR=/tmp
RUN echo "source /home/${USERNAME}/entrypoint.sh" >> /home/${USERNAME}/.bashrc

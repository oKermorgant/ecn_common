#!/bin/bash
# The BSD License
# Copyright (c) 2014 OROCA and ROS Korea Users Group
# Adapted for specific needs during labs - Olivier Kermorgant

#set -x

ros_distro="indigo" # default version

version=`lsb_release -sc`

# yep, we only support LTS's
if [ "$version" = "xenial" ]
then
  ros_distro="kinetic"
fi


echo "[System update]"
sudo apt-get update -qy
sudo apt-get upgrade -qy

echo "[Adding ROS repositories]"
if [ ! -e /etc/apt/sources.list.d/ros-latest.list ]; then
  sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu $version main\" > /etc/apt/sources.list.d/ros-latest.list"
fi

echo "[Getting ROS key]"
roskey=`apt-key list | grep "ROS builder"`
if [ -z "$roskey" ]; then
  wget --quiet https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -O - | sudo apt-key add -
fi

echo "[Update]"
sudo apt-get update -qy

echo "[ROS installation]"
sudo apt-get install -y ros-$ros_distro-desktop ros-$ros_distro-opencv3 ros-$ros_distro-vision-visp qtcreator python-rosdep gitk python-sympy python-rosinstall python-catkin-tools ros-$ros_distro-gazebo-ros ros-$ros_distro-gazebo-plugins ros-$ros_distro-moveit-setup-assistant ros-$ros_distro-moveit ros-$ros_distro-amcl ros-$ros_distro-gmapping ros-$ros_distro-image-view ipython python-matplotlib

source /opt/ros/$ros_distro/setup.bash
echo "[rosdep init]"
sudo sh -c "rosdep init"

#!/bin/bash
# The BSD License
# Will install V-rep in /opt and configure the ros plugin
# Olivier Kermorgant

vrep_version="3_4_0"    # without ROS plugin and check vitals

ros_distro="indigo" # default version

version=`lsb_release -sc`

if [ "$version" = "xenial" ]
then
  ros_distro="kinetic"
fi

base_dir=$('pwd')

echo "[Installing catkin tools]"
sudo apt-get install -y python-catkin-tools

echo "[Downloading V-REP]"
if [ ! -f V-REP_PRO_EDU_V${vrep_version}_Linux.tar.gz ]
then
    wget http://coppeliarobotics.com/files/V-REP_PRO_EDU_V${vrep_version}_Linux.tar.gz
fi

if [ ! -f V-REP_PRO_EDU_V${vrep_version}_Linux.tar.gz ]
then
    exit
fi

tar -xzf V-REP_PRO_EDU_V${vrep_version}_Linux.tar.gz

if [ -d /opt/vrep ]
then
echo "Deleting previous directory"
sudo rm -rf /opt/vrep
fi

sudo cp -R V-REP_PRO_EDU_V${vrep_version}_Linux /opt/vrep
sudo chmod -R a+rwx /opt/vrep
export VREP_ROOT=/opt/vrep

echo "[Configuring ROS plugin and interface]"
cd /opt/vrep/programming/ros_packages/
mkdir -p catkin/src
chmod -R a+rX catkin
mv v_repExtRosInterface catkin/src

echo "[Compiling ROS plugins]"
cd catkin
catkin config --extend /opt/ros/${ros_distro} -i /opt/ros/${ros_distro} --install
catkin clean -b --yes
sudo -E bash -c 'VREP_ROOT=/opt/vrep && catkin build --pre-clean'

# copy ros interface to vrep folder
if [ -f /opt/vrep/libv_repExtRosInterface.so ]
then 
   sudo rm /opt/vrep/libv_repExtRosInterface.so
fi
sudo ln -s /opt/vrep/programming/ros_packages/catkin/devel/.private/vrep_ros_interface/lib/libv_repExtRosInterface.so /opt/vrep

sudo chmod -R a+rX /opt/ros/
sudo chmod -R a+rX /opt/vrep

echo "[Creating V-REP launch file and shortcut]"
launch_file=/opt/ros/${ros_distro}/share/vrep_ros_interface/vrep.launch
if [ -f $launch_file ]
then 
  sudo rm $launch_file
fi
sudo touch $launch_file
sudo chmod a+r $launch_file
sudo cat>$launch_file <<EOL
<?xml version="1.0"?>
<launch>
    <!-- Launch V-REP and roscore -->
    <arg name="scene" default=""/>
    <node name="vrep" pkg="vrep_ros_interface" type="vrep.py" respawn="false" output="screen" args="\$(arg scene)"/>   
</launch>
EOL

# shortcut for V-REP launch script
node_file=/opt/ros/${ros_distro}/share/vrep_ros_interface/vrep.py
if [ -f $node_file ]
then 
  sudo rm $node_file
fi
sudo touch $node_file
sudo chmod a+rX $node_file
sudo cat>$node_file <<EOL
#!/usr/bin/env python
from subprocess import call
from sys import argv
from os import chdir
from os.path import abspath
arg = ''
if len(argv) > 1:
    arg = abspath(argv[1])
chdir('/opt/vrep')
call(['bash','vrep.sh',arg])
EOL
sudo chmod a+x $node_file


cd $base_dir

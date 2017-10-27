#!/bin/bash

export ros_distro="indigo" # default version

export version=`lsb_release -sc`

# yep, we only support LTS's
if [ "$version" = "xenial" ]
then
  export ros_distro="kinetic"
fi

# install skel
skel()
{
    wget http://www.irccyn.ec-nantes.fr/~kermorga/files/skel.tar.gz
    sudo tar -xzf skel.tar.gz -C /etc/
}

# install ROS
install_ros()
{
    wget http://www.irccyn.ec-nantes.fr/~kermorga/files/ros_system_install.sh
    bash ros_system_install.sh
}

# install V-REP + ROS plugins
install_vrep()
{
    wget http://www.irccyn.ec-nantes.fr/~kermorga/files/vrep_system_install.sh
    bash vrep_system_install.sh
}


before_dl()
{
    # before ROS installations
    mkdir -p catkin/src
    cd catkin/src
}

# download Baxter files into temporary catkin ws
dl_baxter()
{
    source /opt/ros/$ros_distro/setup.bash
    wget http://www.irccyn.ec-nantes.fr/~kermorga/files/baxter.tar.gz
    tar -xzf baxter.tar.gz -C .
}


# download any github into temporary catkin ws
dl_gh()
{
    git clone $1    
}


# catkin install to /opt/ros
after_dl()
{    
    catkin config --extend /opt/ros/${ros_distro} -i /opt/ros/${ros_distro} --install
    sudo -E bash -c 'catkin build --pre-clean'
    sudo chmod -R a+rX /opt/ros    
}


#!/bin/bash

source def_reinstall.sh

# purge unused packages
sudo apt purge -y thunderbird pidgin mousepad
sudo apt install ipython geany gedit

base_dir=$('pwd')

# # we do everything in this directory and then we remove it
mkdir -p reinstall
cd reinstall


# first install V-REP
install_vrep $base_dir

before_dl

dl_baxter
dl_gh https://github.com/oKermorgant/ecn_common.git
dl_gh https://github.com/oKermorgant/slider_publisher.git

# after ROS installations
after_dl


cd $base_dir
sudo rm -rf reinstall

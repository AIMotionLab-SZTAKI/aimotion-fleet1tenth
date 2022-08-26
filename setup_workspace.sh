#!/usr/bin/env bash

# Setup script of the master PC

# source ROS and include into .bashrc
source /opt/ros/noetic/setup.bash
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

# build aimotion-f1tenth-system
cd aimotion-f1tenth-system
rm -r devel
rm -r build 
catkin_make
path=$(cd "$MY_PATH" && pwd)

source $path/devel/setup.bash
echo "source $path/devel/setup.bash" >> ~/.bashrc

cd ..
cd fleet1tenth

# build fleet1tenth
rm -r devel # remove existing folders
rm -r build
catkin_make
path=$(cd "$MY_PATH" && pwd)
echo "source $path/devel/setup.bash" >> ~/.bashrc
source $path/devel/setup.bash

echo "aimotion-fleet1tenth is built successfully"

#!/usr/bin/env bash

# Setup script of the master PC

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
path=$(cd "$MY_PATH" && pwd) 
echo "source $path/devel/setup.bash" >> ~/.bashrc
echo "fleet1tenth workspace built successfully!"

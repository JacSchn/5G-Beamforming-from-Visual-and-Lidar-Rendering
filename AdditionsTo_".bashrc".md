File covers information regarding what was added into the .bashrc file on the Nanos and why.

What is .bashrc?
The .bashrc file is located in the home directory on the Nanos and Ubuntu systems in general. This file is run each time a new terminal is opened. This means that any “source” command that needs to be run for a common use item can be placed here and it’ll run each time a terminal is opened and eliminate the need for the user to do it manually. The file uses shell scripting, or more specifically, bash scripting in this case. So, any commands or shell scripts can be placed here to run to make it easier for the user.

Both Nanos
	This section covers what is the same on both Nano’s .bashrc file.

Activating py3.6.9 Virtual Environment
Command:
source /home/usk/env/bin/activate

This activates the python virtual environment that the donkeycar system runs in.

ROS and Catkin Set Up
source /opt/ros/melodic/setup.bash
source /home/musk/catkin_ws_py3/devel/setup.bash
export EDITOR=nano


The first command activates ROS commands for this terminal session.
The second command sets up the catkin_ws to allow use of the packages in the source folder.
The last command allows the default editor for rosed to be nano.

Donkeycar Dependency
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libgomp.so.1


This allows the use of a shared object file for donkeycar v4.2.1. The self driving model will not load without it.

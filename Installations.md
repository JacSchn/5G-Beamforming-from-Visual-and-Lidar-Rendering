	The details in this document are in regards to Hawk Rover and will not be covering the software implementation of the Xiaor Geek vehicle.

Jetpack
Version
v4.5.1

Info
	Jetpack is provided by Nvidia at no cost. It provides the baseline for developing on one of their platforms. The Jetpack version used is the one for the Jetson Nano 4GB, not the 2GB version. Jetpack comes with an installation of Ubuntu 18.04. As of 08/11/2021, the Jetpack image does not support Ubuntu 20.04.

	A copy of the Jetpack version used is located in the same directory as this file.

Issues
	The main issue in regards to the Jetpack version is a bug for connecting a ps4 controller via bluetooth. The user has to either delete the controller connection and then reconnect or connect the device manually in a terminal using bluetoothctl.

Installation
	Setting up Jetpack is a straightforward process. First, download the Jetpack version you will be using. Hawk Rover uses v4.5.1 and a copy of the file is located in the same directory as this file.

	Next, the file will need to be etched onto a microSD card. A popular etching tool is BalenaEtcher and it was used for Hawk Rover as well. Download the correct version of BalenaEtcher for your device.

	https://www.balena.io/etcher/

	Now, follow the installation instructions from the official Nvidia developer forms below

https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit#write

	That is it for setting up the microSD card with the Jetpack version. Now the microSD can be inserted into the Nano and turned on. This will prompt a standard Ubuntu desktop set up if not done in headless mode.

	Now that everything is set up, proceed to the next section for setting up the donkeycar software.

Donkeycar
Version
v4.2.1

Info
	Donkeycar is an open source software for self-driving vehicles. A copy of the donkeycar version is available in the donkey car folder located in this directory.

Issues
	The hex codes for the ps4 controller were incorrect and were manually corrected for Hawk Rover. They will need to be fixed manually for any new downloads of the folder.

	Link to correct hex codes

https://githubmemory.com/repo/autorope/donkeycar/issues?cursor=Y3Vyc29yOnYyOpK5MjAyMS0wMi0yN1QyMzoyMjoxMCswODowMM4wwOwc&pagination=next&page=4

Installation
	Follow the steps below for installing the software of donkeycar. When it asks for cloning the master branch, either use their version from the site or use the folder stored in this file’s directory. The master branch may be up to a new version and could cause some inconsistencies for the code that is used for data collection.

	https://docs.donkeycar.com/guide/install_software/


ROS
Version
Melodic Morenia for Ubuntu 18.04

Info
	Robot Operating System (ROS) is used in our case for collecting data from the cameras and LiDAR. More information can be found in ROS Notes.

Issues
	Main issue is that Melodic only supports Ubuntu 18.04 and python2 natively. Donkeycar uses python 3.6.9 to run. This causes some issues, but it can be worked around. Below are some other options given from one of the maintainers of donkeycar on Discord.

ROS Neotic seems to work on Ubuntu 18.04 as well. https://github.com/dusty-nv/jetson-containers/blob/master/Dockerfile.ros.noetic 
You can also challenge
docker run --rm -it arm64v8/ubuntu:20.04
# or
docker run --rm -it arm64v8/ros:noetic-ros-core-focal

Jetson Containers may work well for installing ROS Noetic which natively supports py3 on the Nano. 
https://github.com/dusty-nv/jetson-containers

Installation
Follow the below instructions, but keep in mind the points below.
We used ros-melodic-desktop-full.
Do up to 1.5 in the install process and stop there. We do not need to go to step 1.6.
Install Process
	
Install rosdep:

	$ sudo apt install python-rosdep

	
ROS Melodic natively only supports python2. The self driving script uses python 3.6.8. Although ROS Noetic natively supports python3, that is only for Ubuntu 20.04. The Jetson Nano only supports Ubuntu 18.04, so ROS Melodic is required.
	
The post installation process removes some ROS packages for py2 and replaces them with ones compatible with py3. The only noticeable downside is that custom messages seem to not work, but this can be worked around.

	Update to Py3

	Follow the rest of the ROS tutorials for setting up a catkin_ws here.

	The first catkin_make command should be as follows, also shown in the tutorial.
	$ catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3


Download empy

	$ sudo apt-get install python3-empy


Install GitHub Repositories
Info
In order to avoid pulling the whole repository multiple times, only a single branch will be cloned into each section that it is needed for. There are five different sections.
data: Folder name is “data”
development
ros-data-collection-dev
router_data_collection


git clone https://github.com/JacSchn/5G-Beamforming-from-Visual-and-Lidar-Rendering --branch development --single-branch development

git clone https://github.com/JacSchn/5G-Beamforming-from-Visual-and-Lidar-Rendering --branch ros-data-collection-dev --single-branch ~/catkin_ws/src/

git clone https://github.com/JacSchn/5G-Beamforming-from-Visual-and-Lidar-Rendering --branch ros-data-collection-dev --single-branch ~/catkin_ws/src/

git clone https://github.com/JacSchn/5G-Beamforming-from-Visual-and-Lidar-Rendering --branch router_data_collection --single-branch router_scripts



"git clone <url> --branch <branch> --single-branch <folder>"

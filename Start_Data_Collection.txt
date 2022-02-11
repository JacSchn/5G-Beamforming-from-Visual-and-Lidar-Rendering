Document covers how to start the data collection process for the Hawk Rover.

IMPORTANT!! Ensure the system clocks are all synched up before collecting genuine data!


Initial Notes:
While using ROS, a roscore needs to be running before any publishers/ subscribers or messages/ services can connect or operate.

Before running the roscore, the catkin workspace needs to be sourced. This step is already done for the user every time a new terminal is created since the command is in the .bashrc file located in the home directory.

If the rplidar_ros package is launched first, then roscore does not need to be initialized. The rplidar_ros package launches a roscore and there only needs to be one roscore running.

All data is stored in files in their respective directory under the data directory which is located in the home directory.
Ex. ~/data/lidar/scan_data_*.txt

There will be multiple terminals open for each package with their respective publishers and subscribers.

There is also a script that automates most of starting the data collection and a separate script for stopping it. They are described below.

Simple Start/ End Collection
	A few scripts were developed to simplify the data collection process.

Order of Launch
	The commands below indicate the order in which to launch each file.

$ cd ~/development
$ ./initialize_collection.sh
$ ./start_data_collection.sh
$ ./stop_data_collection.sh


initialize_collection.sh
This file must be run first. It runs necessary files before data collection can begin.

The PID (Process ID) of each process on the Nano is saved into a file located in each sensor’s respective folder in the ~/logs directory. This is used to be able to kill the process later.

It first boots up chrony on the nano and on the two routers. The user must check the output to make sure that the dates are correct.

It then boots up the pinging between the two routers. No data is being collected at this time.

It then launches the A3 lidar, then the front USB cam on port 0, and the rear USB cam on port 2. The lidar will start spinning to indicate that it is working. Each of these sensors have a log file that they send their output to located in ~/logs/.

start_data_collection.sh
	This is the second file to run. It launches all of the data collection scripts.
	
Just like the initialize_collection.sh script, this script saves the PID of each process into a different file located in each sensor’s respective folder in the ~/logs directory.

The order in which data collection starts is as follows, routers, lidar, front_usb, then rear_usb.

Once all of the scripts for data collection have been launched, a message appears to the terminal indicating the start time of data collection. This info is saved into a file in the ~/logs/ directory.

stop_data_collection.sh
	This file stops the data collection process for all devices.

	The method for stopping all devices is to use the PID number located in each sensor’s respective folder in the ~/logs/ directory.

	Sometimes the USB camera does not want to be killed, so extra steps are taken to ensure that the port is freed for future data collection use.

	A stop time is displayed when all processes have been killed. This is also saved to a file located in the ~/logs/ directory.

CSI Data Collection
	To start collecting data from the camera that operates the self-driving car, there are a few steps that need to be taken first.

1. Initialize roscore if not done already.

$ roscore


2. In a new terminal, start the vehicle’s drive command. Make sure to link the model that will be used for self driving. This will also launch the csi camera publisher.

$ rosrun csi_drive_data manage.py --model <model path>


Insert the model path to link to the model that will be used. 

3. Once the previous command says the website is up, connect to it through Google chrome using the vehicle’s IP address. The user must be connected to the same network as the vehicle.

	<vehicle’s_IP>:8887/drive

4. Set the vehicle on local angle or local pilot mode. Local angle will have the vehicle control its turning while the user controls the speed. Local pilot will allow the AI to control the turning and speed of the vehicle. It is best to set a max constant speed for the vehicle if local angle is used. All of these are options on the site.

	A default value can be set in the myconfig.py file to have the vehicle launch in local angle or local pilot mode.

5. Start collecting camera image data. This command can be run before sep 4 or can be run in a SSH shell through putty or another method:

$ rosrun csi_drive_data cam_data_sub.py


Image arrays along with their timestamps are saved in a file located in the ~/data/camera/ directory.

6. To end data collection, just hit ctrl + C to end each process.

USB Data Collection
1. Initialize roscore if not done already.

$ roscore


2. In a new terminal, start the publisher script

$ rosrun usb_cam sony_cam.py


3. In a new terminal, start the subscriber script

$ rosrun usb_cam cam_sub.py


4. To end data collection, just hit ctrl + C in each terminal.

Old LiDAR Data Collection
1. Boot up the LiDAR:

$ roslaunch rplidar_ros rplidar_a3.launch


2. Start collecting the scan data:

$ rosrun rplidar_ros rplidarNodeWriteFile


The scan data as well as their timestamp saved in a file located in the ~/data/lidar/ directory.

3. To end data collection, just hit ctrl + C in each terminal.

Router Data Collection
1. Ensure the time has been synchronized with the Nano. Information about how to do that is here.

2. a. Start the collect_data.sh script on the Server router from Nano

$ ssh root@192.168.1.7 '~/collect_data.sh


2. b. Optionally, ssh into the Server router and launch the script from there.

$ ssh root@192.168.1.7
$ ./collect_data.sh


3. To end data collection, run the stop_collect_data.sh file located on the Server router.

$ ssh root@192.168.1.7 '~/stop_collect_data.sh'


OR

$ ssh root@192.168.1.7
$ ./stop_collect_data.sh



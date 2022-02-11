	The service used to sync the system clocks on the Nano and both the routers is chrony. Below is some information about chrony and how it is set up for each device.

Info
	Chrony is a time synchronization service for syncing time between multiple devices. It uses NTP as its method for synching, but it is a more updated version of the standard NTP service and is the recommended service to use. For our purposes, we will be synching up the system clocks of the routers with the Nano. This is done to have as accurate of timestamps as possible for the data that is collected. The time does not have to be perfect with true time from atomic clocks. However, the system time on the routers must match the nano with ideally less than 5ms variance.
	
Chrony gradually syncs up the time of the various system clocks. It will initially update the system clock quickly, every 2-16 seconds, until the devices are synchronized to within roughly 600 μs. It will then update the clock on longer intervals. It was noted with the routers that their system clock drifts relatively quickly from the Nano if not updated frequently. Thus, both routers were set to have a max period between syncs of 32 seconds. This results in synched clocks that are accurate to the millisecond.

	The structure of the synchronization network is as follows. Note that a ‘server’ is a NTP host that sends its clock out to synchronize other devices. A ‘client’ receives the system clock information from a server and will then update its own clock. Lastly, consider router A to be the router with an IP address of 192.168.1.7 and router B as having an IP address of 192.168.1.10. So, the Nano receives real time from a NTP server from ubuntu. Then the Nano acts as a server to router A. Next, router A will act as a server to router B. Essentially, ubuntu updates the Nano with true time, then the Nano updates router A, and then router A updates router B.

Here is a graphical representation of our implementation:

Ubuntu -> Nano -> Router A -> Router B

The important thing to note is that the Nano may be required to synchronize with the ubuntu NTP server upon first boot before it can be a server to router A.

The chrony version that the routers are using is an older version and thus their configuration formats are different than that of the Nano.

Link to generic NTP information:
https://ubuntu.com/server/docs/network-ntp

Link to chrony documentation:
http://manpages.ubuntu.com/manpages/focal/man5/chrony.conf.5.html?_ga=2.165484735.1674038847.1628522077-1276439187.1626759313

	Link to the chronyc command documentation:
http://manpages.ubuntu.com/manpages/focal/man1/chronyc.1.html

Link to chronyd command documentation:
http://manpages.ubuntu.com/manpages/focal/man8/chronyd.8.html

Easy Startup
	To make it easier starting up data collection, a convenience script was made to boot data collection and time synchronization. This script is located in the development branch and is named initialize_collection.sh. The first section of this file takes care of synchronizing the time of the Nano and both routers. Make sure the Nano was able to sync its time online first before launching the script. It is also important that all devices are connected properly before launching the script.

	To run the script first be in the development directory and run
	$ ./initialize_connection.sh

	This script will launch a script on the server router which will then launch a script on the client router. Both of these scripts are named sync_time.sh. They use chrony online to activate the servers and then display the sources using chrony sources so that the user can verify the connection has been made.

	Once the time synchronization is done, the initializing of data collection will begin right away.

Commands
	A larger list of commands and their explanations are in the above links. Below are a few commands that are useful in determining the offset of the system clock as well as whether or not the system is up and running.

chronyc tracking
	Displays some useful information regarding time synchronization.

chronyc activity
	Displays information about the number of active services. Useful for finding out if the server is offline.

chronyc sources
	Displays information about the sources that the router is synchronized with.
	Use ‘-v’ flag to display information about the format of the table.
	‘^?’ means that there is no connection to the NTP server.

chronyc online/offline
	Lets chronyd know if the services are online or offline.This will need to be used to turn on chronyd at times on the routers as they tend to be offline initially.
	Command needs to be run if chronyc activity shows the server as offline for the Nano or router A.

service chrony status
	Command shows if the chrony service is running with some added details as well. The routers do not have this command, only the Nano.

 Technical Setup
	Specific details about the time synchronization structure is below. It is important to allow the chrony system to synchronize on all devices for a minute or two before starting data collection. This will improve the timestamp accuracy to an acceptable level.

Nano
	The Nano has its chrony.conf file located in the /etc/chrony/ directory. This configuration file has two notable sections. One is the server list from which the Nano collects time synchronization data from. The other is opening the Nano to act as a NTP server.

	The Nano collects synchronization data from several pool sources provided by ubuntu. A pool is a collection of NTP servers that are chosen randomly from a list. This prevents any one server from being overloaded with multiple requests.

	Towards the end of the /etc/chrony/chrony.conf file, there is a section that uses the ‘allow all’ keyword. This allows the Nano to act as a server to any client connected to the same network.

Routers
	Both routers have two chrony configuration files. One is located in /etc/chrony/chrony.conf. The other is located in /etc/config/chrony. The main file that is used to configure the chrony service is located in the config directory.

	The routers have an older version of chrony, but it is the latest version that can be installed on it through their package manager. Viewing the chrony file in the config/ directory shows what NTP server that the router is connected to. Router A will have a hostname of ‘192.168.1.100’, which is the IP address of the Nano over the ethernet. Router B will have a hostname of ‘192.168.100.7’, which is the IP address of router A over the 60GHz wireless connection.

The maxpoll and minpoll indicate the max and min times the server should be polled for its time in seconds. The numbers indicate powers of two. So, a maxpoll of five indicates 2^5 seconds, or 32 seconds.

Router A has an additional configuration to allow all connections on its network to connect to sync with its NTP server.

How to Connect
	 This process works best by turning on each system one at a time once their NTP connection is made. So, turn on and sync the Nano, then turn on and sync router A, and lastly turn on and sync router B.

	This also assumes that chrony has been installed on all systems.

Sync the Nano
	Synching the Nano should be straightforward. Chrony should disable the timedatectl service that comes with Ubuntu, but it doesn’t have to in order to function.
	
	The Nano will need to be connected to an internet connection to sync up with online NTP servers. This synchronization process is automatic and should happen within a minute of logging in. A way to check if the synchronization is active is running
	$ service chrony status

	This command will indicate if chrony is active as will indicate if chrony has synched with a server. If it does not show that it has selected a source after saying it has started chrony, then check your internet connection. If the LAN cable to the router is connected to Nano, disconnect it for a moment until the Nano updates the system clock. The cable connection may be causing the issue.
	Once the Nano’s system clock has updated, then proceed to synching router A.

Sync Router A
	Ensure the Nano’s system clock has been synchronized first before continuing.

	As a refresher, router A is the router with IP address of 192.168.1.7. It is also the server router for the client router for both the NTP service as well as for the 60GHz connection. Router A has an IP address of 192.168.100.7 over the 60GHz wireless network.

	First, ssh into router A from the Nano. This assumes router A is connected by LAN.
	$ ssh root@192.168.1.7

	Next, check if the router is connected to the Nano’s NTP server.
	$ chronyc sources

	If the output shows a ‘^?’ under MS, then Nano’s NTP server is currently unreachable. Proceed to the next steps to connect.

	First, check if the source is considered online or offline.
	$ chronyc activity

	If it says, “1 sources offline”, then the router believes the Nano’s server is offline. Since the Nano’s service is actually online, let chrony know by using the following command
	$ chronyc online

	This will let chrony know that the source is online. Check that this is the case by using
	$ chronyc activity

	It should now say that the source is online. Wait a few seconds before checking that a connection was made with the Nano’s server.
	$ chronyc sources

	The command should show a ‘^*’ under MS. A detailed description of this table can be viewed by using
	$ chronyc sources -v

	Lastly, check that the date is current with the Nano. View the date by entering
	$ date

	Router A should now be synched with the Nano. A detailed description of the connection can be viewed by using
	$ chronyc tracking

	This is useful to see how close the two clocks are to each other. Details about the format of this output are in the chronyc command documentation link located towards the bottom of the Info section.

Sync Router B
	Ensure router A’s system clock has been synchronized first before continuing.

	As a refresher, router B is the router with IP address of 192.168.1.10 on its label. Its IP address over the 60GHz network is 192.168.100.10. Router B is only a client in the chain of NTP connections.

	First, ssh into router A from the Nano. This assumes router A is connected by LAN.
	$ ssh root@192.168.1.7

	Now, ssh into router B from the ssh terminal from router A.
	$ ssh root@192.168.100.10

	Next, check if the router is connected to the router A’s server.
	$ chronyc sources

	If the output shows a ‘^?’ under MS, then Nano’s NTP server is currently unreachable. Proceed to the next steps to connect.

	First, check if the source is considered online or offline.
	$ chronyc activity

	If it says, “1 sources offline”, then the router believes router A’s server is offline. Since the Nano’s service is actually online, let chrony know by using the following command
	$ chronyc online

	This will let chrony know that the source is online. Check that this is the case by using
	$ chronyc activity

	It should now say that the source is online. Wait a few seconds before checking that a connection was made with router A’s server.
	$ chronyc sources

	The command should show a ‘^*’ under MS. A detailed description of this table can be viewed by using
	$ chronyc sources -v

	Lastly, check that the date is current with router A. View the date by entering
	$ date

	Router B should now be in sync with router A. A detailed description of the connection can be viewed by using
	$ chronyc tracking

	This is useful to see how close the two clocks are to each other. Details about the format of this output are in the chronyc command documentation link located towards the bottom of the Info section.

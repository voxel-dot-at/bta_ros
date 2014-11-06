bta_ros
===================
### ROS integration for Bluetechnix devices operated by the Bluetechnix 'Time of Flight' BTAeth or BTAusb libraries. ###

# Summary #

This drives allows to configure your system and ROS to use all Bluetechnix Time
of Flight devices supported by the BtaApi. It includes an example allowing you 
to visualize depth data using the rviz viewer included in ROS. It shows you how
to use the ToF devices together with ROS and how you can modify different 
parameters from them. Additionally we have include a nodelet to capture rgb
video from those new devices that includes a 2D sensor as the Argos 3D P320 or 
Sentis TOF P510

## First step: Get Ros ##

The bta_ros driver works with ROS versions hydro and indigo. You can use catkin 
workspaces or the previous rosbuild to configure, compile and get ready ROS.

The following ROS tutorial links describe how to get ros_hydro and catkin 
workspace.

In Ubuntu:
Follow the ROS installation tutorial: 
>http://wiki.ros.org/hydro/Installation/Ubuntu.

Use catkin workspaces:
>http://wiki.ros.org/catkin 
>
>http://wiki.ros.org/catkin_or_rosbuild
>
>http://wiki.ros.org/catkin/Tutorials/create_a_workspace

To configure a catkin workspace in your ROS installation, follow this; 
>ROS tutorial: http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment

## Known Problems ##

* This is a release candidate

# 1. Getting the Bluetechnix 'Time of Flight' API #

The BtaApi is provided and maintained by Bluetechnix. You can get if from:

> https://support.bluetechnix.at/wiki/Bluetechnix_%27Time_of_Flight%27_API
 
Please be sure you download the right library for you device. Move the
library and the header to the bta_ros directory. You could install the library 
and its headers in the system path i.e. "/usr/include" and "/usr/lib". Also you
can modify the Findbta.cmake file and add your own paths.

# 2. Compiling the package #

#### 2.1 Install dependencies ####

Make sure you have the following ROS dependencies already installed:
<pre><code>apt-get install ros-indigo-pcl-ros ros-indigo-pcl-conversions ros-indigo-perception-pcl 
</code></pre>

#### 2.2 Install the package ####

Clone from repository: https://github.com/voxel-dot-at/bta_ros.git
to the src/ folder in your catkin workspace.
Now compile it with:
<pre><code>cd catkin_ws
source devel/setup.bash ## initialize search path to include local workspace
cd src/
git clone https://github.com/voxel-dot-at/bta_ros.git
cd ..
catkin_make
</code></pre>

# 3. Usage #

We have included some .launch files to help you to get the camera working in a 
very simple way. You have the possibility of capture tof data, rgb images or 
both at the same time. We coded this ROS driver to use the ROS parameter 
server and the dynamic_reconfiguration tools which let you change the camera 
configuration before running and in run time. We will explain you in the 
following lines how you can write your own configuration for connecting to your 
Bluetechnix ToF device. You can also run the package as a node or as standalone 
nodelet and set the camera configuration by line commands.

### * Watch our demo video:  ###

> TODO

### 3.1 Use roslaunch ###
To easily start using the bta_ros driver you can use the roslaunch files we 
have included. It will launch the ROS core, start the bta_ros driver and the 
device, load the parameter configuration, start the run-time reconfiguration 
gui and the ROS viewer rviz already configured to show the depth or/and rgb 
information captured by the ToF camera.

In order to execute it you have just to type the following:

<pre><code>roslaunch bta_ros nodelet.launch #for running it as nodelet
#or
roslaunch bta_ros node.launch #for running it as node
</code></pre>

#### 3.1.1 Write you own configuration to the parameter server ####

As we commented before, you can load the camera configuration to the parameter 
server. The launch files include, inside the tag "node", the tag "rosparam" in 
which you can indicate a file containing the server parameters for the tof 
sensor. We have include the configuration file "/launch/bta_ros_1.yaml" that 
defines default configuration values for the camera. You may modify this file 
or add yours to load your own configuration. 

Almost all configuration options are those corresponding to the configuration 
object given for start the connection with the ToF camera. You also can set the
integration time and the frame rate when starting the connection with the 
device.

This is a example of the needed parameter for connecting to a Bluetechnix tof
device with a network interface: 

<pre><code>#Parameter server configuration for bta_ros.
udpDataIpAddr: {
n1: 224,
n2: 0,
n3: 0,
n4: 1,
}
udpDataIpAddrLen: 4
udpDataPort: 10002

tcpDeviceIpAddr: {
n1: 192,
n2: 168,
n3: 0,
n4: 10,
}
tcpDeviceIpAddrLen: 4
tcpControlPort: 10001
</code></pre>

All configuration options are those corresponding to the configuration object 
used for starting the connection with the ToF camera. To get more information 
about the BtaApi configuration parameter, please refer to the BtaApi 
documentation. 

> https://support.bluetechnix.at/wiki/BltTofApi_Quick_Start_Guide

For the 2d stream we just need the http ip address where the camera is serving 
the .sdp file to start capturing video. Inside you sensor2D node you must 
include the following parameter:

<pre><code> < param name="2dIP" value="192.168.0.10"/ >
</pre></code>


#### 3.1.2 Modifying camera parameter runtime ####

Using the dynamic_reconfigure package we can change the camera register to 
adapt the tof camera functions to our needs. We included different options for 
changing the integration time and the frame rate as well as the possibility of
read and write values to the camera registers.

For using this option in an user interface, run the ROS rqt_reconfigure node.
We have included it to be run in the .launch files.



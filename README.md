bta_ros
===================
### ROS integration for Bluetechnix devices operated by the Bluetechnix 'Time of Flight' BTAeth or BTAusb libraries. ###

# Summary #

This drives allows to configure your system and ROS to use all Bluetechnix Time of Flight devices supported by the BtaApi.
It includes an example allowing you to visualize depth data using the rviz viewer included in ROS.
It shows you how to use the ToF devices together with ROS and how you can modify different parameters from them.

## First step: Get Ros ##

The bta_ros driver works with ROS versions groovy, hydro and indigo. You can use catkin workspaces or the previous rosbuild to configure, compile and get ready ROS.

The following ROS tutorial links describe how to get ros_hydro and catkin workspace.

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
 
Please be sure you download the right library for you device. Please move the library and the header to the bta_ros directory. You could install the library and its headers in the system path i.e. "/usr/include" and "/usr/lib". Also you can modify the Findbta.cmake file and add your own paths.

# 2. Compiling the package #

#### 2.1 Install dependencies ####

Make sure you have the following ROS dependencies already installed:
<pre><code> apt-get install ros-indigo-pcl-ros ros-indigo-pcl-conversions ros-indigo-perception-pcl 
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

We have included some .launch files to help you to get the camera working in a very simple way. We coded this ROS driver to use the ROS parameter server. We will explain you in the following lines how you can write your own configuration for connecting to your Bluetechnix ToF device. You can also run the package node or nodelet standalone and set the camera configuration by line commands.

### * Watch our demo video:  ###

> TODO

### 3.1 Use roslaunch ###
To easily start using the bta_ros driver you can use the roslaunch files we have included. It will launch the ROS core, start the bta_ros driver and the device, load the parameter configuration, start the run-time reconfiguration gui and the ROS viewer rviz already configured to show the depth information captured by the ToF camera.

In order to execute it you have just to type the following:

<pre><code>roslaunch bta_ros nodelet.launch #for running it as nodelet
#or
roslaunch bta_ros node.launch #for running it as node
</code></pre>

#### 3.1.1 Write you own configuration to the parameter server ####

As we commented before, you can load the camera configuration to the parameter server. The file launch files include, inside the tag "node", the tag "rosparam" in which you can indicate a file containing the server parameters. We have include the configuration file "/launch/bta_ros_1.yaml" that defines default configuration values for the camera. You may modify this file or add yours to load your own configuration.

Almost all configuration options are those corresponding to the configuration object given for start the connection with the ToF camera. You also can set the integration time and the frame rate when starting the connection with the device.

To get more information about the BtaApi configuration parameter, please refer to the BtaApi documentation.


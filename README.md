# Instruction to Amr Radar Obstacle Avoidance 


This repository provides an example set up that can be used to automate your ROS workflow in the VS Code IDE.


## Contents:
* [1. Installation](#1-installation)
* [2. Configuration](#2-configuration)
* [3. Running](#3-running)


## 1 Installation
### 1.1 Making workspace for catkin
```
mkdir catkin_ws
cd catkin_ws
```
Next step is download our package:
``` 
git clone https://https://github.com/KubaArrow/costum_safety_zone.git
```
---

### 1.2  Bulding package

Next step is bulding all package

```
cd catkin_ws
catkin_make 
```

>__Recommended in Raspberry PI 3__
> ```
> catkin build
> ```
>
>
>__WARNING!!!__
>Surely you will need other packages to ti_mmwave. Don't worry, all package you will found to ros melodic or ros noetic :)
### 1.4 Environment Package

You must source this script in every bash terminal you use ROS in.

```
echo "HETBOT/amr_radar_obstacle_avoidance/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
```
## 2 Configuration *amr_radar_obstacle_avoidance*

### 2.1 File structure:
<pre>
|-- include
|   |-- RadarScan.h 
|   |-- Zone.h
|   |-- pugiconfig.hpp
|   |-- pugixml.hpp
|
|-- launch
|   |-- robot.launch
|   |-- server.launch
|   |-- testing.launch
|
|-- msg
|   |-- RadarScan.msg 
|
|-- rviz
|   |-- represent.rviz 
|
|-- src
|   |-- converterTf.cpp
|   |-- Zone.cpp
|   |-- zone_handler.cpp
|   |-- pugixml.cpp
</pre>

Folders *inlcude*, *msg*, *src* contain our source code and necessary file from Texas Instrument (*RadarScan.msg*).

In *rviz* folder have configuration files to RVIZ.

The most important is folder *launch*.
In this folder we have configuration files to package.
*robot.launch* is launch file for robot.
*server.launch* is launch file for server.
*testing.launch* is launch file for server with sensors.

### 2.2 Configuration launch files

__Node *server.launch* for server:__
```xml
<launch>
  <!--Main tf name-->
  <arg name="main_tf" value="base_link" doc="Name main tf for ti_mmwave_sesnors"/>
  <param name="bubble_zone_main_tf" value="$(arg main_tf)"  />
  <!--Path to xml with zones-->
  <param name="bubble_zone_path_xml" value="/home/monia/HETBOT/amr_radar_obstacle_avoidance/build/bubble_zone/xml/Radars.xml"  />
  <!--Making TF for one sensor-->
  <node pkg="tf2_ros" type="static_transform_publisher" name="radar_baselink_0" args="-0.1 0 0.15 0 0 0 base_link ti_mmwave_0"/>
  <!--Start node for one sensors-->
  <node pkg="bubble_zone"  type="converterTf" name="converterTf_0" args="0" output="screen"  ></node>
  <!--Start node with checking points-->
  <node pkg="bubble_zone"  type="zoneHandler" name="zoneHandler" ns="bubble_zone" output="screen"  ></node>
    <!--Start rviz-->
If the entire installation and configuration has been successfully completed, RVIZ should start. :)
  ```
  This file starts three nodes:

  Node *zon[mmwave_ti_ros](https://dev.ti.com/tirex/explore/node?node=AFb4FPjEEzryPswZ9ILeFQ__VLyFKFf__LATEST). 
eHandler* make zones from xml file, captures data from ti_mmwave node after conversion in converterTf node and send information if sensors detect object in zones.

  Node *converterTf* converting pointcloud from ti_mmwave publisher to point for zoneHandler.

  Node *rviz* starts RVIZ program with suitable configutration.

You must downl
  >If you use sensors in different amount or configuration, can change launch file. 
  >His construction is easy.

  __Node *robot.launch* for robot:__
```xml
<launch>
  <!--Configuration Device (Default value from TI)-->
  <arg name="[mmwave_ti_ros](https://dev.ti.com/tirex/explore/node?node=AFb4FPjEEzryPswZ9ILeFQ__VLyFKFf__LATEST). 
me="max_allowed_elevation_angle_deg" default="90" doc="Maximum allowed elevation angle in degrees for detected object data [0 > value >= 90]}"/>
  <arg name="max_allowed_azimuth_angle_deg" default="90" doc="Maximum allowed azimuth angle in degrees for detected object data [0 > value >= 90]}"/>
  <!---->zone change own size in all direction" output="screen">
    <param name="command_port" value="/dev/ttyUSB0"  />
    <pa>Surely you will need other packages to ti_mmwave. Don't worry, all package you will found to ros melodic or ros noetic :)

Next step is paste our launch for this package [launch_for_turtlebot](#). ram name="command_rate" value="115200"   />
    <param name="data_port" value="/dev/ttyUSB1"  />
    <param name="data_rate" value="921600"   />
    <param name="max_allowed_elevation_angle_deg" value="$(arg max_allowed_elevation_angle_deg)"   />
    <param name="max_allowed_azimuth_angle_deg" value="$(arg max_allowed_azimuth_angle_deg)"   />
    <param name="frame_id" value="ti_mmwave_0"/>
    <param name="mmWaveCLI_name" value="/mmWaveCLI_0" />
    <remap from="/ti_mmwave/radar_scan" to="/ti_mmwave/radar_scan_0"/>
    <remap from="/ti_mmwave/radar_scan_pcl" to="/ti_mmwave/radar_scan_pcl_0"/>
  </node>
  <node pkg="ti_mmwave_rospkg" type="mmWaveQuickConfig" name="ti_mmwave_config" ns="radar_0" args="$(find ti_mmwave_rospkg)/cfg/$(arg device)_$(arg config).cfg" output="screen" >
    <param name="mmWaveCLI_name" value="/mmWaveCLI_0" />
  </node>
</launch>
```
This launch is from TI. First node prepare ROS for sensor and second node send config to sensor and catching points.

You can preper own configuration for sensor in:
[Program to making cfg file](https://dev.ti.com/gallery/view/mmwave/mmWave_Demo_Visualizer/ver/4.2.0/). 

### 2.3 Make zones

In this version, we make zones in xml files.

```xml
<?xml version="1.0" encoding="utf-8" standalone="no"?>
<!--Main marker with all zones-->
<Zones FormatVersion="1"> 
    <!--Marker of zone-->
    <Zone work="true" zoneType="adaptive_rectangle"  amplificationFactorLinear="5" amplificationFactorAngular="1" colorAlpha="1" colorRed="1" colorGreen="0" colorBlue="0" msg="zone_1">
        <!--Markers with point of zones-->
        <Point x = "2" y = "-0.5"></Point>
        <Point x = "2" y = "0.5"></Point>
        <Point x = "-0.5" y = "0.5"></Point>
        <Point x = "-0.5" y = "-0.5"></Point>
    </Zone>
    <Zone work="true" zoneType="extension_zone" amplificationFactorLinear="5" amplificationFactorAngular="1" colorAlpha="1" colorRed="0" colzone change own size in all direction "1"></Point>
        <Point x = "-1" y = "-1"></Point>
        <Point x = "1" y = "-1"></Point>
    </Zone>
    <Zone work="true" zoneType="adaptive_triangle" amplificationFactorLinear="5" amplificationFactorAngular="1" colorAlpha="1" colorRed="0" colorGreen="0" colorBlue="1" msg="zone_3">
        <Point x = "2" y = "1"></Point>
        <Point x = "2" y = "-1"></Point>
        <Point x = "-1" y = "0"></Point>
    </Zone>
    <Zone work="true" zoneType="half_extension_zone" amplificationFactorLinear="5" amplificationFactorAngular="1" colorAlpha  ="1" colorRed="1" colorGreen="1" colorBlue="0" msg="zone_4">
        <Point x = "-1" y = "3"></Point>
        <Point x = "-3" y = "1"></Point>
        <Point x = "-3" y = "-1"></Point>
        <Point x = "-1" y = "-3"></Point>
        <Point x = "1" y = "-3"></Point>
        <Point x = "3" y = "-1"></Point>
        <Point x = "3" y = "1"></Point>
        <Point x = "1" y = "3"></Point>
    </Zone>
    <Zone work="true" zoneType="static" amplificationFactorLinear="5" amplificationFactorAngular="1" colorAlpha  ="1" colorRed="1" colorGreen="0" colorBlue="1" msg="zone_5">
        <Point x = "-1" y = "2"></Point>
        <Point x = "-2" y = "1"></Point>
        <Point x = "-2" y = "-1"></Point>
        <Point x = "-1" y = "-2"></Point>
    </Zone>
</Zones>
```
__*work*__ - on/off zone (true/false)
__*zoneType*__- type of zone
  types:
    \- *static* (zone don't change own size)
    \- *extension_zone* (zone change own size in all direction)
    \- *half_extension_zone* (zone change own size in driving direction)
    \- *adaptive_rectangle* (special zone for turning)
__*amplificationFactorLinear*__ - amplification factor for linear value
__*amplificationFactorAngular*__ - amplification factor for angular value

*colorAlpha, colorRed, colorGreen, colorBlue* - colors markers in rviz (values 1 or 0)

*msg* - value sending after detection point on *status* topic (if value is empty, program don't send msg)

__BOLD__ atributt can change in present time in *rosparam*

## 3 Running

### 3.1 Creating a package on the robot

You must download and build package on robot and server.

Now, can launch for your configuration.

In robot:

```
roslaunch ti_mmwave_rospkg robot.launch
```

In server:

```
roslaunch ti_mmwave_rospkg server.launch
```

You can modify this launch or make new.

### 3.2 Receiving data

#### First way

On topic */bubble_zone/status* you receive msg from Zones (XML declaration)

#### Second way

On topic */bubble_zone/zonesStatus* you receive table with detected violations of zones



## 4 Features

### 4.1 Dynamic changes

You can change a few params in real time.
For example:

#### Change package with zones

Commends changing XML file with zones:
```
rosparam set /bubble_zone_path_xml "/home/$USER/HETBOT/amr_radar_obstacle_avoidance/src/bubble_zone/xml/Radars.xml"

```
and
```
rosparam set /bubble_zone_path_xml "/home/$USER/HETBOT/amr_radar_obstacle_avoidance/src/bubble_zone/xml/Radars2.xml
```


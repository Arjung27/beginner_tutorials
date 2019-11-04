# Beginner_tutorials - Publisher Subscribers Model
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
---

## Overview

This repository contains the development of basic ROS publisher subcriber node. The process is learned and followed from [here](http://wiki.ros.org/ROS/Tutorials). The development has two nodes:

	1. Talker- src/publisher.cpp
	2. Listener - src/subscriber.cpp

## Dependencies

Following dependencies needs to be installed before running this project:
	
	1. ROS Kinetic (installation can be followed from [here](http://wiki.ros.org/kinetic/Installation/Ubuntu))
	2. catkin (installed with ROS)
	3. Ubuntu 16.04

## How to Build project

To build the project follow the steps below:
```
mkdir -p ~/catkin_ws/src/
cd ~/catkin_ws/
catkin_make
source ~/catkin_ws/devel/setup.bash (For every new terminal this command needs to be run before running any 
	ROS command. You could add it to your bashrc so that every time a terminal opens it would run by itself)
cd src/
git clone https://github.com/Arjung27/beginner_tutorials.git
cd ..
catkin_make
```

## Running the project

To run the given model, follow the following steps (NOTE: Every ros command below needs to be run on a separate terminal):

```
1. roscore
2. rosrun beginner_tutorials publisher
3. rosrun beginner_tutorials subscriber
```
NOTE: if ~/catkin_ws/devel/setup.bash is not added to bash then run it before running each command mentioned above and run each command in separate terminal.

## Running the demo using launch file

```
1. Build the project as decribed under "How to build your project"
2. ~/catkin_ws/devel/setup.bash
3. roslaunch beginner_tutorials beginner_tutorials.launch frequency:=<frequency value between 0 to inf>
For e.g. roslaunch beginner_tutorials beginner_tutorials.launch frequency:=0
4. You can omit the frequency:=<> part, if you want to run the demo using default value
```
This will start the roscore and the two nodes in single terminal

## Running the service

We have added the service to modify the base outout string and the service could be accessed using the following commands. After you run the two nodes (either using launch file or running nodes separately), Open a new terminal and run the follwing demo commnad:
```
rosservice call /modifyDefaultMessage "Current Frequency"
```
NOTE: Instead of "Current Frequency" you could type any custom string and those changes would be reflected in the output terminal

## How to check the log messages

After you have run the nodes (either using launch file or running nodes separately). You can check the log messages using the GUI provided by ROS 
by simply typing the following command in a new terminal:
```
rqt_console
```
## Licence 

```
MIT License

Copyright (c) 2019 Arjun Gupta

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE
```
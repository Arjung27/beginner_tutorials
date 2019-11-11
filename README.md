# Beginner_tutorials - Publisher Subscribers Model
<a href='https://github.com/Arjung27/beginner_tutorials/blob/master/LICENSE'><img src='https://img.shields.io/badge/License-MIT-brightgreen.svg'/></a>
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
2. rosrun beginner_tutorials publisher <frequency_value(optional)>
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

## TF Frames
To inspect the static/child tf frames broadcasted by talker node with respect to the parent/world, first launch the talker and listener node. You can do that by:
```
roslaunch beginner_tutorials beginner_tutorials.launch
```
Now run the following command to view the tf frames:
```
rosrun tf tf_echo /world /talk
```

The above command will give the following output:
```
At time 1573497290.201
- Translation: [0.000, 0.000, 0.000]
- Rotation: in Quaternion [0.707, 0.001, -0.707, 0.001]
            in RPY (radian) [3.140, 1.570, 0.000]
            in RPY (degree) [179.909, 89.954, 0.000]
At time 1573497290.901
- Translation: [0.000, 0.000, 0.000]
- Rotation: in Quaternion [0.707, 0.001, -0.707, 0.001]
            in RPY (radian) [3.140, 1.570, 0.000]
            in RPY (degree) [179.909, 89.954, 0.000]
At time 1573497291.901
- Translation: [0.000, 0.000, 0.000]
- Rotation: in Quaternion [0.707, 0.001, -0.707, 0.001]
            in RPY (radian) [3.140, 1.570, 0.000]
            in RPY (degree) [179.909, 89.954, 0.000]
```
For graphical visualization of the frame use ```rqt_tf_tree``` command:
```
rosrun rqt_tf_tree rqt_tf_tree
```
This command also generates the pdf storing the result. You can find the pdf in the directory where it is run.

## Running ROSTEST

Used gtest/rostest to create a Level 2 integration test, that tests the Publisher node. To run the test run the following command from your workspace root directory:
```
catkin_make run_tests_beginner_tutorials
```
This will run the tests and the output will be as follows:
```
[Testcase: testtestPublisher] ... ok

[ROSTEST]-----------------------------------------------------------------------

[beginner_tutorials.rosunit-testPublisher/serviceExists][passed]
[beginner_tutorials.rosunit-testPublisher/serviceMessage][passed]

SUMMARY
 * RESULT: SUCCESS
 * TESTS: 2
 * ERRORS: 0
 * FAILURES: 0
```

## Record Bag files

To enable recording of all the topics, we need to launch the nodes and enable recording. We do this simltaneousl using the following command:
```
roslaunch beginner_tutorials beginner_tutorials.launch frequency:=1 record:=true
```
```frequency:=1``` initializes the frequency to the user set value, by default the value is set to 10. Now to check the recorded files run:
```
rosbag info results/listener.bag
```
The output of the command would be like:
```
path:        results/listener.bag
version:     2.0
duration:    14.7s
start:       Nov 11 2019 00:03:11.68 (1573448591.68)
end:         Nov 11 2019 00:03:26.36 (1573448606.36)
size:        224.7 KB
messages:    1047
compression: none [1/1 chunks]
types:       rosgraph_msgs/Log  [acffd30cd6b6de30f120938c17c593fb]
             std_msgs/String    [992ce8a1687cec8c8bd883ec73ca41d1]
             tf2_msgs/TFMessage [94810edda583a504dfda3829e70d7eec]
topics:      /chatter      145 msgs    : std_msgs/String   
             /rosout       380 msgs    : rosgraph_msgs/Log  (3 connections)
             /rosout_agg   377 msgs    : rosgraph_msgs/Log 
             /tf           145 msgs    : tf2_msgs/TFMessage
```
To play the rosbag we need to run just the subscriber node and not the publisher node. Run the following subsequent commands in separate terminals:
```
roscore
rosrun beginner_tutorials subscriber
rosbag play results/listener.bag
``` 
## License 

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
/**
 *  MIT License
 *
 *  Copyright (c) 2019 Arjun Gupta
 *
 *  Permission is hereby granted, free of charge, to any person obtaining a
 *  copy of this software and associated documentation files (the "Software"),
 *  to deal in the Software without restriction, including without
 *  limitation the rights to use, copy, modify, merge, publish, distribute,
 *  sublicense, and/or sell copies of the Software, and to permit persons to
 *  whom the Software is furnished to do so, subject to the following
 *  conditions:
 *
 *  The above copyright notice and this permission notice shall be included
 *  in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 *  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 *  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 *  THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 *  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 *  FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 *  DEALINGS IN THE SOFTWARE.
 *
 *  @file    publisher.cpp
 *  @author  Arjun Gupta
 *  @copyright MIT License
 *
 *  @brief   Implementation of ROS publisher and subscriber nodes
 *
 *  @section DESCRIPTION
 *
 *  This program is a part of the beginner tutorials in ROS
 *  It defines the publisher (talker)
 *
 */

#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/modifyDefaultMessage.h"

/**
 * Declare the global string variable and initializing it
 */

extern std::string defaultMessage = "Updated Frequency ";

bool modifyDefaultMessage( \
                  beginner_tutorials::modifyDefaultMessage::Request &req, \
                  beginner_tutorials::modifyDefaultMessage::Response &resp) {
  defaultMessage = req.inputMessage;
  resp.outputMessage = defaultMessage;
  ROS_INFO_STREAM("New default message is: " << defaultMessage);
  return true;
}
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
  // Default frequency value
  int frequency = 20;
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
// %Tag(INIT)%
  ros::init(argc, argv, "talker");
// %EndTag(INIT)%
if (argc == 2) {
  frequency = atoi(argv[1]);
} else {
  ROS_WARN_STREAM("Since no input is given using default starting frequency");
}

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
// %Tag(NODEHANDLE)%
  ros::NodeHandle n;
// %EndTag(NODEHANDLE)%

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
// %Tag(PUBLISHER)%
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
// %EndTag(PUBLISHER)%
// Flag to check if frequency is in acceptable range or not
int flag = 0;
if (frequency <= 0) {
  ROS_ERROR_STREAM("Frequency is set to less than equal to 0");
  frequency = 1;
  ROS_DEBUG_STREAM("Since frequency <= 0, setting frequency to 1");
}
// %Tag(LOOP_RATE)%
  ros::Rate loop_rate(frequency);
// %EndTag(LOOP_RATE)%
  auto server = n.advertiseService("modifyDefaultMessage", \
                                              modifyDefaultMessage);
  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
// %Tag(ROS_OK)%
  if (!ros::ok()) {
    ROS_FATAL_STREAM("ROS node not running");
  }
  while (ros::ok()) {
// %EndTag(ROS_OK)%
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String message;
    std::stringstream ss;
    ss << defaultMessage << frequency;
    message.data = ss.str();

// %Tag(ROSCONSOLE)%
    ROS_INFO("%s", message.data.c_str());
// %EndTag(ROSCONSOLE)%
  if ((frequency < 10) || (frequency > 30) && flag == 0) {
      ROS_WARN_STREAM("Frequency is out of the utility range (i.e. 10-30)");
      frequency++;
      flag = 0;
      if (frequency >= 49) {
        flag = 1;
      }
    } else if ((frequency >= 10) && (frequency <= 30) && flag == 0) {
        ROS_INFO_STREAM("Frequency is in utility range (i.e. 10-30)");
        frequency++;
        flag = 0;
    } else if (frequency >= 50 || flag == 1) {
        ROS_DEBUG_STREAM("Frequency is too high reducing it");
        frequency--;
        flag = 1;
    }
    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
// %Tag(PUBLISH)%
    chatter_pub.publish(message);
// %EndTag(PUBLISH)%

// %Tag(SPINONCE)%
    ros::spinOnce();
// %EndTag(SPINONCE)%

// %Tag(RATE_SLEEP)%
    loop_rate.sleep();
// %EndTag(RATE_SLEEP)%
  }


  return 0;
}
// %EndTag(FULLTEXT)%

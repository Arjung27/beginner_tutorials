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
#include "tf/transform_broadcaster.h"
#include "std_msgs/String.h"
#include "beginner_tutorials/modifyDefaultMessage.h"

/**
 * Declare the global string variable and initializing it
 */
extern std::string defaultMessage = "Updated Frequency";

/**
 * @brief Function to change the output base string
 * 
 * @param req Request message
 * @param resp Response message
 * 
 * @return bool if callback function ran successfully or not
 */
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
  ros::init(argc, argv, "publisher");
  // Creating a TransformBroadcaster object
  static tf::TransformBroadcaster br;
  // Creating a Transform object
  tf::Transform transform;
  // If and else conditional check to see of the user has set any input or not
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

  ros::NodeHandle n;

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
  auto chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  /**
  * Flag to check if frequency is in acceptable range or not
  */
  int flag = 0;
  /** 
   * Conditional check if the frequency is less than zero or not if less than 
   * zero then set the frequency to 1 else continue with the entered/default 
   * frequency
  */
  if (frequency <= 0) {
    ROS_ERROR_STREAM("Frequency is set to less than equal to 0");
    frequency = 1;
    ROS_DEBUG_STREAM("Since frequency <= 0, setting frequency to 1");
  }

  ros::Rate loop_rate(frequency);
  /*
  * Call to callback function designed to modify the base output string
  */
  auto server = n.advertiseService("modifyDefaultMessage", \
                                              modifyDefaultMessage);
  /* 
  * Conditional to check if the node is running or not
  */
  if (!ros::ok()) {
    ROS_FATAL_STREAM("ROS node not running");
  }
  /**
   * While loop to print the output as per the current frequency. 
   * THe output of the stream changes as per the range of frequency. 
   * If frequency is too high i.e. greater than 50 then we reduce the 
   * frequency to bring it to the desired range.
   */ 
  while (ros::ok()) {
    // set the translation and rotation values
    transform.setOrigin(tf::Vector3(0.0, 0.0, 0.0));
    // Create quaternion object.
    tf::Quaternion q;
    // set roll, pitch, yaw values
    q.setRPY(3.14, 3.14/2, 0);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform,
                         ros::Time::now(), "world", "talk"));
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String message;
    std::stringstream ss;
    ss << defaultMessage << " " << frequency;
    message.data = ss.str();

    ROS_INFO("%s", message.data.c_str());
    /*
    * Conditional to check the range of frequency
    */
    if ((frequency < 10) || (frequency > 30) && flag == 0) {
        ROS_WARN_STREAM("Frequency is out of the utility range (i.e. 10-30)");
        frequency++;
        flag = 0;
        // Conditional to set flag equal to 1
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
          // Conditional to set flag equal to 0
          if (frequency == 20) {
            flag = 0;
          }
      }
    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(message);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}

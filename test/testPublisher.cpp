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
 */

/**
 *@file       testPublisher.cpp
 *@author     Arjun Gupta
 *@copyright  MIT License
 *@brief      testPublisher.cpp file for rostest.
 */

#include <ros/ros.h>
#include <gtest/gtest.h>
#include <ros/service_client.h>
#include <tf/transform_listener.h>
#include "beginner_tutorials/modifyDefaultMessage.h"

/**
 * @brief Test case to check whether the modifyDefaultMessage service
 *        exists or not.
 * @param none
 * @return none
 */

TEST(TestSuite, serviceExists) {
    // Create a NodeHandle object
    ros::NodeHandle nh;
    // Inititalize a ServiceClient object
    ros::ServiceClient client = nh.serviceClient<beginner_tutorials::\
                    modifyDefaultMessage>("modifyDefaultMessage");
    EXPECT_TRUE(client.waitForExistence(ros::Duration(10)));
}
/**
 * @brief Test case to check whether the modifyDefaultMessage service
 *        is modifying the message as expected or not.
 * @param none
 * @return none
 */

TEST(TestSuite, serviceMessage) {
    // Create a NodeHandle object
    ros::NodeHandle nh;
    // Inititalize a ServiceClient object
    ros::ServiceClient client = nh.serviceClient<beginner_tutorials::\
                    modifyDefaultMessage>("modifyDefaultMessage");
    beginner_tutorials::modifyDefaultMessage srv;
    // Assigning a input message
    srv.request.inputMessage = "Current frequency is";
    client.call(srv);
    // Test to check if the response and the input are the same
    EXPECT_EQ("Current frequency is", srv.response.outputMessage);
}

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "testPublisher");
  return RUN_ALL_TESTS();
}
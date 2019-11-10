#include <ros/ros.h>
#include <gtest/gtest.h>
#include <ros/service_client.h>

int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "testPublisher");
  ros::NodeHandle nh;
  return RUN_ALL_TESTS();
}
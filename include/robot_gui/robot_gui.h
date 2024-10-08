#pragma once

#include "robotinfo_msgs/RobotInfo10Fields.h"
#include "ros/node_handle.h"
#define CVUI_IMPLEMENTATION
#include "cvui.h"
#include "std_msgs/Float64.h"
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

class RobotGUI {
public:
  RobotGUI(ros::NodeHandle *nh);
  void run();
  void robot_info_display(cv::Mat &frame);
  void teleop_buttons();
  void current_velocity();
  void robot_position_odom();
  void distance_travelled_service();

private:
  ros::NodeHandle *nh_;
  ros::Subscriber sub_;
  robotinfo_msgs::RobotInfo10Fields message;
  std::string topic_name;
  void msgCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg);
  const std::string WINDOW_NAME = "ROS ROBOT GUI";
};
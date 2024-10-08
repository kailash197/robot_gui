#pragma once
#define CVUI_IMPLEMENTATION
#include "cvui.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include "ros/node_handle.h"
#include "std_srvs/Trigger.h"
#include <opencv2/opencv.hpp>
#include <ros/ros.h>

class RobotGUI {
public:
  RobotGUI(ros::NodeHandle *nh);
  void run();
  void robot_info_display(cv::Mat &frame);
  void teleop_buttons(cv::Mat &frame);
  void current_velocity(cv::Mat &frame);
  void robot_position_odom(cv::Mat &frame);
  void distance_travelled_service(cv::Mat &frame);

private:
  ros::NodeHandle *nh_;
  ros::Subscriber sub_;
  robotinfo_msgs::RobotInfo10Fields robotinfo_message;
  ros::Publisher pub_;
  ros::Subscriber velocity_sub_;
  geometry_msgs::Twist twist_message;
  ros::Subscriber odom_sub_;
  nav_msgs::Odometry odom_message;
  ros::ServiceClient get_distance_service_client, reset_distance_service_client;
  std_srvs::Trigger get_distance_service_req, reset_distance_service_req;
  std::string info_topic_name, vel_topic_name, odom_topic_name,
      get_distance_service_name, reset_distance_service_name, distance_;
  void msgCallback(const robotinfo_msgs::RobotInfo10Fields::ConstPtr &info_msg);
  void velMsgCallback(const geometry_msgs::Twist::ConstPtr &twist_msg);
  void odomMsgCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);
  const std::string WINDOW_NAME = "ROS ROBOT GUI";
};
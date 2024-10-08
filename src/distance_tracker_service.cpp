#include "../include/robot_gui/distance_tracker_service.h"
#include "ros/node_handle.h"
#include <iomanip>
#include <sstream>

DistanceTrackerService::DistanceTrackerService(
    ros::NodeHandle *node_handle_ptr) {
  nh_ = node_handle_ptr;
  distance_ = 0.0;
  odom_sub_ =
      nh_->subscribe("odom", 1, &DistanceTrackerService::odomCallback, this);
  get_distance_service_ = nh_->advertiseService(
      "get_distance", &DistanceTrackerService::getDistanceServiceCallback,
      this);
  reset_distance_service_ = nh_->advertiseService(
      "reset_distance", &DistanceTrackerService::resetDistanceServiceCallback,
      this);
  ROS_INFO("Distance tracker node initialized.");
}

std::string DistanceTrackerService::formatFloatToString(float f) {
  std::ostringstream out;
  out << std::fixed << std::setprecision(2) << f;
  return out.str();
}

void DistanceTrackerService::odomCallback(
    const nav_msgs::Odometry::ConstPtr &msg) {
  // calculate distance traveled using Euclidean distance formula
  double dx = msg->pose.pose.position.x - prev_pose_.position.x;
  double dy = msg->pose.pose.position.y - prev_pose_.position.y;
  double dz = msg->pose.pose.position.z - prev_pose_.position.z;
  double distance = sqrt(dx * dx + dy * dy + dz * dz);

  // add distance traveled to total distance
  distance_ += distance;

  // update previous pose
  prev_pose_ = msg->pose.pose;
}

bool DistanceTrackerService::getDistanceServiceCallback(
    std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
  res.success = true;
  // Respond with distance traveled in meters
  res.message = formatFloatToString(distance_);
  ROS_INFO("Server: Get Distance Service Return Success.");
  return true;
}

bool DistanceTrackerService::resetDistanceServiceCallback(
    std_srvs::Trigger::Request &reset_req,
    std_srvs::Trigger::Response &reset_res) {
  reset_res.success = true;
  distance_ = 0.00;
  // Respond with distance traveled in meters
  reset_res.message = formatFloatToString(distance_);
  ROS_INFO("Server: Reset Distance Service Return Success.");
  return true;
}

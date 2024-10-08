#pragma once
#include "nav_msgs/Odometry.h"
#include "ros/node_handle.h"
#include "ros/ros.h"
#include "std_srvs/Trigger.h"

class DistanceTrackerService {
public:
  DistanceTrackerService(ros::NodeHandle *node_handle_ptr);

private:
  ros::NodeHandle *nh_;
  ros::Subscriber odom_sub_;
  ros::ServiceServer get_distance_service_;
  ros::ServiceServer reset_distance_service_;
  double distance_;

  std::string formatFloatToString(float flt);
  void odomCallback(const nav_msgs::Odometry::ConstPtr &odom_msg);
  bool getDistanceServiceCallback(std_srvs::Trigger::Request &req,
                                  std_srvs::Trigger::Response &res);
  bool resetDistanceServiceCallback(std_srvs::Trigger::Request &reset_req,
                                    std_srvs::Trigger::Response &reset_res);
  geometry_msgs::Pose prev_pose_;
};

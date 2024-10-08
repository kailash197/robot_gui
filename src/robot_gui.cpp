#include "../include/robot_gui/robot_gui.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include "ros/node_handle.h"
#include "std_srvs/Trigger.h"

RobotGUI::RobotGUI(ros::NodeHandle *node_handle) {
  this->nh_ = node_handle;
  info_topic_name = "robot_info";
  vel_topic_name = "cmd_vel";
  odom_topic_name = "odom";
  get_distance_service_name = "get_distance";
  reset_distance_service_name = "reset_distance";
  sub_ = nh_->subscribe<robotinfo_msgs::RobotInfo10Fields>(
      info_topic_name, 20, &RobotGUI::msgCallback, this);
  velocity_sub_ = nh_->subscribe<geometry_msgs::Twist>(
      vel_topic_name, 20, &RobotGUI::velMsgCallback, this);
  odom_sub_ = nh_->subscribe<nav_msgs::Odometry>(
      odom_topic_name, 10, &RobotGUI::odomMsgCallback, this);
  pub_ = nh_->advertise<geometry_msgs::Twist>(vel_topic_name, 100);
  get_distance_service_client =
      nh_->serviceClient<std_srvs::Trigger>(get_distance_service_name);
  distance_ = "0.0";
  reset_distance_service_client =
      nh_->serviceClient<std_srvs::Trigger>(reset_distance_service_name);
  ROS_INFO("RobotGUI Node created.");
}

void RobotGUI::velMsgCallback(const geometry_msgs::Twist::ConstPtr &twist_msg) {
  twist_message = *twist_msg;
  ROS_DEBUG("Twist Message Received.");
}

void RobotGUI::msgCallback(
    const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg) {
  robotinfo_message = *msg;
  ROS_DEBUG("Robot Info Message Received.");
}

void RobotGUI::odomMsgCallback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
  odom_message = *odom_msg;
  ROS_DEBUG("Odometry Message Received.");
}

void RobotGUI::robot_info_display(cv::Mat &frame) {

  // parameters
  int win_posx = 40, win_posy = 20, win_width = frame.cols - 80,
      win_height = 200;
  int posx = 45, posy = 45, white = 0xffffff;
  float font_s = 0.4;

  std::vector<std::string> data_fields = {
      robotinfo_message.data_field_01, robotinfo_message.data_field_02,
      robotinfo_message.data_field_03, robotinfo_message.data_field_04,
      robotinfo_message.data_field_05, robotinfo_message.data_field_06,
      robotinfo_message.data_field_07, robotinfo_message.data_field_08,
      robotinfo_message.data_field_09, robotinfo_message.data_field_10};

  cvui::window(frame, win_posx, win_posy, win_width, win_height,
               "General Info Area: " + info_topic_name);

  for (int i = 0; i < 10; i++) {
    cvui::printf(frame, posx, posy + 20 * i, font_s, white,
                 data_fields[i].c_str());
  }
}

void RobotGUI::teleop_buttons(cv::Mat &frame) {
  // parameters
  int width = 90, height = 30;
  float font_s = 0.5;
  int for_posx = 160, for_posy = 230;
  int stp_posx = 160, stp_posy = 270;
  int bac_posx = 160, bac_posy = 310;
  int lft_posx = 60, lft_posy = 270;
  int rgt_posx = 260, rgt_posy = 270;

  if (cvui::button(frame, for_posx, for_posy, width, height, "Forward")) {
    if (twist_message.linear.x < 1.0) {
      twist_message.linear.x += 0.1;
    } else {
      twist_message.linear.x = 1.0;
    }
  }

  if (cvui::button(frame, stp_posx, stp_posy, width, height, "Stop")) {
    twist_message.linear.x = 0.0;
    twist_message.angular.z = 0.0;
  }

  if (cvui::button(frame, bac_posx, bac_posy, width, height, "Backward")) {
    if (twist_message.linear.x > -1.0) {
      twist_message.linear.x -= 0.1;
    } else {
      twist_message.linear.x = -1.0;
    }
  }

  if (cvui::button(frame, lft_posx, lft_posy, width, height, "Left")) {
    if (twist_message.angular.z < 1.0) {
      twist_message.angular.z += 0.1;
    } else {
      twist_message.angular.z = 1.0;
    }
  }
  if (cvui::button(frame, rgt_posx, rgt_posy, width, height, "Right")) {
    if (twist_message.angular.z > -1.0) {
      twist_message.angular.z -= 0.1;
    } else {
      twist_message.angular.z = -1.0;
    }
  }
  pub_.publish(twist_message);
}

void RobotGUI::current_velocity(cv::Mat &frame) {
  // params:
  int win_posx_linear = 40, win_posy_linear = 350;
  int win_posx_angular = 210, win_posy_angular = 350;
  int width = 150, height = 40;

  // Create window at (220, 20) with size 250x80 (width x height) and title
  cvui::window(frame, win_posx_linear, win_posy_linear, width, height,
               "Linear Velocity");
  cvui::printf(frame, win_posx_linear + 75, win_posy_linear + 25, 0.4, 0xff0000,
               "%.2f m/s", twist_message.linear.x);

  cvui::window(frame, win_posx_angular, win_posy_angular, width, height,
               "Angular Velocity");
  cvui::printf(frame, win_posx_angular + 67, win_posy_angular + 25, 0.4,
               0xff0000, "%.2f rad/s", twist_message.angular.z);
}

void RobotGUI::robot_position_odom(cv::Mat &frame) {
  // params:
  int posx = 40, posy = 405;
  float font_size_text = 0.4, font_size_pos = 0.85;
  int win_posx_X = posx;
  int win_posy_X = posy + 10;
  int width = 100, height = 100;

  cvui::text(frame, posx, posy, "Estimated robot position based on odometer",
             font_size_text);

  cvui::window(frame, win_posx_X, win_posy_X, width, height, "X");
  cvui::printf(frame, win_posx_X + 10, win_posy_X + 45, font_size_pos, 0xffffff,
               "%.2f", odom_message.pose.pose.position.x);
  cvui::window(frame, win_posx_X + 110, win_posy_X, width, height, "Y");
  cvui::printf(frame, win_posx_X + 120, win_posy_X + 45, font_size_pos,
               0xffffff, "%.2f", odom_message.pose.pose.position.y);
  cvui::window(frame, win_posx_X + 220, win_posy_X, width, height, "Z");
  cvui::printf(frame, win_posx_X + 230, win_posy_X + 45, font_size_pos,
               0xffffff, "%.2f", odom_message.pose.pose.position.z);
}

void RobotGUI::distance_travelled_service(cv::Mat &frame) {
  // params
  int posx = 40, posy = 525;
  float font_size_text = 0.4;
  cvui::text(frame, posx, posy, "Distance Travelled by Robot", font_size_text);

  int button_posx = posx, button_posy = posy + 15;
  int width = 100, height = 80;
  if (cvui::button(frame, button_posx, button_posy, width, height,
                   "Get Distance")) {
    if (get_distance_service_client.call(get_distance_service_req)) {
      distance_ = get_distance_service_req.response.message;
      ROS_INFO("Client: Distance Service Call Success.");
    } else {
      ROS_WARN("Client: Distance Service Call Failed.");
    }
  }

  int win_posx = posx + width + 10, win_posy = button_posy;
  int win_width = 210, win_height = 80;
  float font_size_pos = 0.7;

  cvui::window(frame, win_posx, win_posy, win_width, win_height,
               "Distance in meters");
  cvui::printf(frame, win_posx + 75, win_posy + 45, font_size_pos, 0xffffff,
               "%s", distance_.c_str());

  if (cvui::button(frame, button_posx, button_posy + 90, 320, 60,
                   "Reset Distance")) {
    if (reset_distance_service_client.call(reset_distance_service_req)) {
      distance_ = reset_distance_service_req.response.message;
      ROS_INFO("Client: Reset Distance Service Call Success.");
    } else {
      ROS_WARN("Client: Reset Distance Service Call Failed.");
    }
  }
}

void RobotGUI::run() {
  // canvas for drawing the GUI height x width
  cv::Mat frame = cv::Mat(700, 400, CV_8UC3);

  // Init a OpenCV window and tell cvui to use it.
  cv::namedWindow(WINDOW_NAME); //, cv::WINDOW_NORMAL);
  cvui::init(WINDOW_NAME);

  while (ros::ok()) {
    // Fill the frame with a shade of gray
    frame = cv::Scalar(49, 52, 49);

    // general info display
    robot_info_display(frame);
    teleop_buttons(frame);
    current_velocity(frame);
    robot_position_odom(frame);
    distance_travelled_service(frame);

    cvui::update();
    cv::imshow(WINDOW_NAME, frame);

    // Check if ESC key was pressed
    if (cv::waitKey(20) == 27) {
      break;
    }
    // Spin as a single-threaded node
    ros::spinOnce();
  }
}

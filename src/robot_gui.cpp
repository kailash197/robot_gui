#include "../include/robot_gui/robot_gui.h"
#include "geometry_msgs/Twist.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include "ros/node_handle.h"

RobotGUI::RobotGUI(ros::NodeHandle *node_handle) {
  this->nh_ = node_handle;
  info_topic_name = "robot_info";
  vel_topic_name = "cmd_vel";
  sub_ = nh_->subscribe<robotinfo_msgs::RobotInfo10Fields>(
      info_topic_name, 20, &RobotGUI::msgCallback, this);
  velocity_sub_ = nh_->subscribe<geometry_msgs::Twist>(
      vel_topic_name, 20, &RobotGUI::velMsgCallback, this);
  pub_ = nh_->advertise<geometry_msgs::Twist>(vel_topic_name, 100);
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
               "Topic: " + info_topic_name);

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

void RobotGUI::run() {
  // canvas for drawing the GUI height x width
  cv::Mat frame = cv::Mat(700, 400, CV_8UC3);

  // Init a OpenCV window and tell cvui to use it.
  cv::namedWindow(WINDOW_NAME, cv::WINDOW_NORMAL);
  cvui::init(WINDOW_NAME);

  while (ros::ok()) {
    // Fill the frame with a shade of gray
    frame = cv::Scalar(49, 52, 49);

    // general info display
    robot_info_display(frame);
    teleop_buttons(frame);

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

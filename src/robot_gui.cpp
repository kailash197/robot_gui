#include "../include/robot_gui/robot_gui.h"
#include "robotinfo_msgs/RobotInfo10Fields.h"
#include "ros/node_handle.h"

RobotGUI::RobotGUI(ros::NodeHandle *node_handle) {
  this->nh_ = node_handle;
  topic_name = "robot_info";
  sub_ = nh_->subscribe<robotinfo_msgs::RobotInfo10Fields>(
      topic_name, 20, &RobotGUI::msgCallback, this);
  ROS_INFO("RobotGUI Node created.");
}

void RobotGUI::msgCallback(
    const robotinfo_msgs::RobotInfo10Fields::ConstPtr &msg) {
  message = *msg;
  ROS_INFO("Robot Info Message Received.");
}

void RobotGUI::robot_info_display(cv::Mat &frame) {

  // parameters
  int win_posx = 40, win_posy = 20, win_width = frame.cols - 80,
      win_height = 200;
  int posx = 45, posy = 45, white = 0xffffff;
  float font_s = 0.4;

  std::vector<std::string> data_fields = {
      message.data_field_01, message.data_field_02, message.data_field_03,
      message.data_field_04, message.data_field_05, message.data_field_06,
      message.data_field_07, message.data_field_08, message.data_field_09,
      message.data_field_10};

  cvui::window(frame, win_posx, win_posy, win_width, win_height,
               "Topic: " + topic_name);

  for (int i = 0; i < 10; i++) {
    cvui::printf(frame, posx, posy + 20 * i, font_s, white,
                 data_fields[i].c_str());
  }
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

    // Update cvui internal stuff
    cvui::update();

    // Show everything on the screen
    cv::imshow(WINDOW_NAME, frame);

    // Check if ESC key was pressed
    if (cv::waitKey(20) == 27) {
      break;
    }
    // Spin as a single-threaded node
    ros::spinOnce();
  }
}

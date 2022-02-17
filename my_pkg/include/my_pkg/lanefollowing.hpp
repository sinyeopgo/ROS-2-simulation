#include <iostream>
#include "opencv2/opencv.hpp"
#include "rclcpp/rclcpp.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include <geometry_msgs/msg/twist.hpp>

using namespace cv;
using namespace std;
using Image = sensor_msgs::msg::Image;

class LaneFollowing : public rclcpp::Node{
private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::TimerBase::SharedPtr update_timer_;
  rclcpp::Subscription<Image>::SharedPtr img_sub;

  Mat frame , gray, dst;
  Point prevpt1 = Point(110, 60);
  Point prevpt2 = Point(620, 60);
  Point cpt[2];
  Point fpt;
  int minlb[2];
  double ptdistance[2];
  double threshdistance[2];
  vector<double> mindistance1;
  vector<double> mindistance2;
  int error;

  void subs_callback(const Image::SharedPtr msg);
  void update_callback();
public:
  LaneFollowing();
};
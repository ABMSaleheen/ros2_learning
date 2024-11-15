// #include <chrono>
// #include <iostream>
// #include <string>
// #include <vector>


#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <NumCpp.hpp>
using namespace std;

class AutonmousDrivingNode : public rclcpp::Node {

public:
  AutonmousDrivingNode() : Node("autonomous_driving_node") {
    // int codec = cv::VideoWriter::fourcc('M','J','P','G');
    cv::Size frame_size(640, 480);
    double fps = 40.0;
    double mid_pt_error = 0.0;
    // nc::NdArray<double> edge;
    // nc::NdArray<double> prev_edge;

    vid_subscriber = this->create_subscription<sensor_msgs::msg::Image>(
        "/camera_scan_fwd/image_raw", 20,
        std::bind(&AutonmousDrivingNode::image_process_callback, this,
                  std::placeholders::_1));
  }

private:
  void image_process_callback(sensor_msgs::msg::Image::SharedPtr msg_vid) {
    cv::Mat frame = cv_bridge::toCvCopy(msg_vid, "bgr8")->image;

    // // Resize frame to 640x480
    cv::Mat resized_frame;
    cv::Mat vid_output;
    cv::resize(frame, resized_frame, cv::Size(640, 480));
    RCLCPP_INFO(this->get_logger(), "frame size-- %dx%d", resized_frame.cols,
                resized_frame.rows);

    vid_output = resized_frame;

    cv::imshow("output", vid_output);
    if (cv::waitKey(1) == 27) {
      cout << "esc key is pressed by user" << endl;
    }
  }

  // cv::VideoWriter video_writer_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr vid_subscriber;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AutonmousDrivingNode>());
  rclcpp::shutdown();
  return 0;
}
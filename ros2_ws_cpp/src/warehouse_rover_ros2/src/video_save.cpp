#include <chrono>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/image.hpp>
// using namespace cv;

class VideoSaveNode : public rclcpp::Node
{
public:
    VideoSaveNode() : Node("video_saver_node")
    {       
        // int codec = cv::VideoWriter::fourcc('M','J','P','G');
        cv::Size frame_size(640, 480);
        double fps = 40.0;

        // video_writer_.open("output.avi",  cv::VideoWriter::fourcc('M','J','P','G'), fps, frame_size, true);
        // cv::VideoWriter video_writer_("output.mp4", cv::VideoWriter::fourcc('X', '2', '6', '4'), fps,frame_size, true);
        // video_writer_.open("/home/saleheen_linux/others/cv_vid/warehouse.avi", cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps,frame_size, true);
        int codec = cv::VideoWriter::fourcc('X', 'V', 'I', 'D');
        video_writer_.open("/home/saleheen_linux/others/cv_vid/warehouse/output.avi", codec, fps, frame_size, true);


        if (!video_writer_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Could not open the output video file for write.");
        }

        vid_subscriber = this->create_subscription<sensor_msgs::msg::Image>("/camera_scan_fwd/image_raw",40,
            std::bind(&VideoSaveNode::sub_video_writer_callback, this, std::placeholders::_1));
    }

    ~VideoSaveNode() {
            if (video_writer_.isOpened()) {
                video_writer_.release();
                RCLCPP_INFO(this->get_logger(), "Video writer released.");
            }
    }

private:
    
    void sub_video_writer_callback(const sensor_msgs::msg::Image::SharedPtr msg_vid){
        try {
            cv::Mat frame = cv_bridge::toCvCopy(msg_vid, "bgr8")->image;

            // // Resize frame to 640x480
            cv::Mat resized_frame;
            cv::resize(frame, resized_frame, cv::Size(640, 480));

            // Save individual frames as images for testing
            static int frame_count = 0;
            cv::imwrite("/home/saleheen_linux/others/cv_vid/warehouse/resized_frame_" + std::to_string(frame_count++) + ".jpg", resized_frame);

            if (video_writer_.isOpened()) {
                video_writer_.write(resized_frame);
                RCLCPP_INFO(this->get_logger(), "Resized frame written to video.");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Video writer is not open.");
            }

            cv::imshow("output",resized_frame);
            cv::waitKey(1);
        } 
        catch (cv_bridge::Exception& e) {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        }
    }
    
    cv::VideoWriter video_writer_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr vid_subscriber;

};


int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoSaveNode>());
    rclcpp::shutdown();
    return 0;
}


#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class PubSubNode : public rclcpp::Node
{
public:
    PubSubNode() : Node("pub_sub_node")
    {
        // Create a publisher on the "cmd_vel" topic
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

        // Set up a timer to publish messages periodically
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&PubSubNode::publish_velocity, this));


        subscriber_lidar = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 40, std::bind(&PubSubNode::lidarCallback, this, std::placeholders::_1));
    }

private:
    void publish_velocity()
    {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = 0.2;  // Set a constant linear velocity
        message.angular.z = 0.0; // No rotation

        // RCLCPP_INFO(this->get_logger(), "Publishing linear velocity: %.2f", message.linear.x);
        publisher_->publish(message);

    }

    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr lidar_msg)
    {
        // auto lidar_msg = sensor_msgs::msg::LaserScan();
        auto lidar_range = lidar_msg->ranges.size();

        // for (size_t i = 0; i < std::min(lidar_msg->ranges.size(), static_cast<size_t>(10)); ++i){
        //     RCLCPP_INFO(this->get_logger(), "Received [%zu] range- %f", i, lidar_msg->ranges[i]);
        // }
        size_t section_size = lidar_range / 3;

        std::vector<float> right(lidar_msg->ranges.begin(), lidar_msg->ranges.begin() + section_size);
        std::vector<float> center(lidar_msg->ranges.begin() + section_size, lidar_msg->ranges.begin() + 2 * section_size);
        std::vector<float> left(lidar_msg->ranges.begin() + 2 * section_size, lidar_msg->ranges.end());

        float left_dist = std::min(*std::min_element(left.begin(),left.end()),100.0f);
        RCLCPP_INFO(this->get_logger(), "Range on the left == %f", left_dist);

        // for(size_t i =0; i< left.size(); ++i){
        //     RCLCPP_INFO(this->get_logger(), "Range %zu == %f", i,lidar_msg->ranges[i]);
        // }

    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_lidar;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PubSubNode>());
    rclcpp::shutdown();
    return 0;
}

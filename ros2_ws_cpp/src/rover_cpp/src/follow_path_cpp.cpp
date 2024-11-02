#include <chrono>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>


class FollowPathNode : public rclcpp::Node{

public: 
    FollowPathNode() : Node("follow_path_node")
    {
        publisher_vel = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 40);

        timer_vel_pub = this->create_wall_timer(
                    std::chrono::microseconds(100),
                    std::bind(&FollowPathNode::send_cmd_vel, this));
    
        subscriber_lidar = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 40, std::bind(&FollowPathNode::sub_lidar_calllback, this, std::placeholders::_1));

        subscriber_odom = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", rclcpp::SensorDataQoS(), std::bind(&FollowPathNode::sub_odom_calllback, this, std::placeholders::_1));


    }

private:


    double current_vel_x = 0.0;
    float lin_velocity = 0.75;

    // PD
    float right_wall_err; 
    float prev_right_wall_err;
    rclcpp::Time previous_time_;
    
    geometry_msgs::msg::Twist msg_vel;

    float fwd_spd = 0.0f;
    float right_proximity = 0.0f;
    float mid_proximity =  0.0f;
    float left_proximity = 0.0f;

    float right_proximity_1 = 0.0f;
    float right_proximity_2 = 0.0f;
    float right_proximity_3 = 0.0f;

    void sub_lidar_calllback(const sensor_msgs::msg::LaserScan::SharedPtr msg_lidar){
        // auto lidar_msg = sensor_msgs::msg::LaserScan();
        auto lidar_range = msg_lidar->ranges.size();

        RCLCPP_INFO(this->get_logger(), "Lidar_range: %ld", lidar_range);


        // Dividing into 3 section of each 60 degrees
        size_t section_size = lidar_range/3;
        // RCLCPP_INFO(this->get_logger(), "----Left Sect: %d || Mid Sect: %d || RightSect: %d ----", 
        //                                         section_size, 2* section_size, 3*section_size);


        std::vector<float> right_lidar_section(msg_lidar->ranges.begin(), msg_lidar->ranges.begin()+section_size); // 0:120
        std::vector<float> mid_lidar_section(msg_lidar->ranges.begin()+section_size, msg_lidar->ranges.begin()+2*section_size); // 120:240
        std::vector<float> left_lidar_section(msg_lidar->ranges.begin()+2*section_size, msg_lidar->ranges.end()); //240:360 

        right_proximity = std::min(*std::min_element(right_lidar_section.begin(),right_lidar_section.end()), 100.0f);
        mid_proximity = std::min(*std::min_element(mid_lidar_section.begin(), mid_lidar_section.end()), 100.0f);
        left_proximity = std::min(*std::min_element(left_lidar_section.begin(), left_lidar_section.end()), 100.0f);


        // Dividing right and left sections into 3 sections of 20 degrees each 
        size_t smaller_section_size = section_size/3;

        std::vector<float> right_section_1(right_lidar_section.begin(),right_lidar_section.begin()+smaller_section_size);
        std::vector<float> right_section_2(right_lidar_section.begin()+ smaller_section_size,right_lidar_section.begin()+2*smaller_section_size);
        std::vector<float> right_section_3(right_lidar_section.begin()+2*smaller_section_size, right_lidar_section.end());

        right_proximity_1 = std::min(*std::min_element(right_section_1.begin(),right_section_1.end()), 100.0f);
        right_proximity_2 = std::min(*std::min_element(right_section_2.begin(),right_section_2.end()), 100.0f);
        right_proximity_3 = std::min(*std::min_element(right_section_3.begin(),right_section_3.end()), 100.0f);

    }

    void sub_odom_calllback(const nav_msgs::msg::Odometry::SharedPtr msg_odom){
        // RCLCPP_WARN(this->get_logger(), "Odometry callback triggered");
        current_vel_x = -msg_odom->twist.twist.linear.x;

    }

    void proportional_controller(){

        if(right_proximity_1< 5.0f){
            RCLCPP_WARN(this->get_logger(), "Following Right Wall");
        }
        float setpoint = 1.3f;
        float kp = 1.0f;
        float kd = 1.0;

        // // dt 
        rclcpp::Time current_time = this->now();
        float dt = (previous_time_.nanoseconds() > 0) ? (current_time - previous_time_).seconds() : 0.1;
        if (dt <= 0.0) {
            dt = 0.1;  // Fallback to a default value if dt is invalid
        }


        // Process error
        right_wall_err = setpoint - right_proximity_1;
        float derivative_err = (right_wall_err - prev_right_wall_err)/ dt;
        prev_right_wall_err = right_wall_err;

        // if(right_wall_err < -1.3){
        //     right_wall_err = -1.3;
        // }
        float control_output = kp*right_wall_err + kd*derivative_err;
        control_output= std::clamp(control_output, -1.2f, 1.2f);

        msg_vel.linear.x = lin_velocity;
        msg_vel.angular.z = control_output;  // steer right if too far from right wall

    }


    void send_cmd_vel(){
        float obs_dist_limit = 1.40;
        
        // msg_vel.linear.x = lin_velocity;
        // RCLCPP_WARN(this->get_logger(), "-----Current Linear Velocity----- : %0.2f", current_vel_x);
        // RCLCPP_INFO(this->get_logger(), "Left : %.2f || Mid: %.2f || Right: %.2f", left_proximity, mid_proximity, right_proximity);

        if(right_proximity_1< 2.0){
            proportional_controller();
        }

        // proportional_controller();

        if(mid_proximity < obs_dist_limit )
        {
            // msg_vel.angular.z = 1.0;
            

            if (left_proximity > right_proximity){msg_vel.angular.z = 1.150;}     
            if (right_proximity > left_proximity){msg_vel.angular.z = -1.150;}
            msg_vel.linear.x = 0.5;

            RCLCPP_INFO(this->get_logger(), "Obstacle Ahead");

        }


        else{
            if (left_proximity> obs_dist_limit && mid_proximity > obs_dist_limit && right_proximity > obs_dist_limit){
                msg_vel.angular.z = 0.0;
                msg_vel.linear.x = 1.0;
                RCLCPP_INFO(this->get_logger(), "Going Straight");
            }

            if (left_proximity> obs_dist_limit && mid_proximity > obs_dist_limit && right_proximity < obs_dist_limit){

                msg_vel.angular.z = 1.1;
                RCLCPP_INFO(this->get_logger(), "Turning Left");

            }

            if (left_proximity< obs_dist_limit && mid_proximity > obs_dist_limit && right_proximity > obs_dist_limit){

                msg_vel.angular.z = -1.1;
                RCLCPP_INFO(this->get_logger(), "Turning Right");

            }
        }

        // if ((left_proximity< obs_dist_limit && right_proximity < obs_dist_limit) || mid_proximity < 1.7f )
        // if (left_proximity< obs_dist_limit*1.4 && mid_proximity < obs_dist_limit) {msg_vel.angular.z = -1.10;}
        // if (right_proximity< obs_dist_limit*1.4 && mid_proximity < obs_dist_limit) {msg_vel.angular.z = 1.10;}




        // ////////////////////////
        
        // else if(current_vel_x < 0.1){

        //     msg_vel.linear.x = -0.2;
        //     if (left_proximity > right_proximity){msg_vel.angular.z = 1.150;}     
        //     if (right_proximity > left_proximity){msg_vel.angular.z = -1.150;}
        // }
        // ///////////////////////

        fwd_spd = msg_vel.linear.x;
        // RCLCPP_INFO(this->get_logger(), "-------Fwd Speed : %.2f", fwd_spd);
        
        publisher_vel-> publish(msg_vel);
    }


    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_vel;
    rclcpp::TimerBase::SharedPtr timer_vel_pub;

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_lidar;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odom;
};



int main(int argc, char *argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FollowPathNode>());
    rclcpp::shutdown();
    return 0;
}
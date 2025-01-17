#include <chrono>
#include <functional>
#include <memory>
#include <math.h> 
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

class Docking : public rclcpp::Node
{
public:
    Docking() : Node("Docking")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::Point>(
            "/Intersection_Point", 10, std::bind(&Docking::listener_cb, this, std::placeholders::_1));

        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        rcv_timeout_secs_ = this->declare_parameter("rcv_timeout_secs", 1.0);
        angular_chase_multiplier_ = this->declare_parameter("angular_chase_multiplier", 0.7);
        forward_chase_speed_ = this->declare_parameter("forward_chase_speed", 0.2);
        search_angular_speed_ = this->declare_parameter("search_angular_speed", 0.0); // Change, so the motor starts turning around in search of Docking Station
        max_size_thresh_ = this->declare_parameter("max_size_thresh", 0.4); // stop at 40 cenimeters from the intersection point
        filter_value_ = this->declare_parameter("filter_value", 0.9);

        timer_ = this->create_wall_timer(100ms, std::bind(&Docking::timer_cb, this));
        target_dist_ = 0.0;
        target_ang_ = 0.0;
        lastrcvtime_ = this->now().seconds() - 10000;
    }

private:
    void timer_cb()
    {
        auto msg = geometry_msgs::msg::Twist();
        double current_time = this->now().seconds();

        if ((current_time - lastrcvtime_) < rcv_timeout_secs_) {
            RCLCPP_INFO(this->get_logger(), "Target: %f", target_dist_);
             if (target_dist_ > max_size_thresh_) {
                 msg.linear.x = target_dist_;
             }
            
            msg.angular.z = target_ang_; // -angular_chase_multiplier_ * target_dist_;
        } else {
            RCLCPP_INFO(this->get_logger(), "Target lost");
            msg.angular.z = search_angular_speed_;
        }
        std::cout << "linear.x: " << msg.linear.x <<  " , angular.z: " <<  msg.angular.z << " , dist " << target_dist_ << std::endl;
        publisher_->publish(msg);
    }

    void listener_cb(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        double f = filter_value_;
        target_dist_ = sqrt(pow(msg->x, 2) + pow(msg->y, 2));
        target_ang_ = atan2(msg->y, msg->x);
        lastrcvtime_ = this->now().seconds();
        
    }

    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

    double rcv_timeout_secs_;
    double angular_chase_multiplier_;
    double forward_chase_speed_;
    double search_angular_speed_;
    double max_size_thresh_;
    double filter_value_;

    double target_dist_;
    double target_ang_;
    double lastrcvtime_;

};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Docking>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
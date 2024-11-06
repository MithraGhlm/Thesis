#include <chrono>
#include <functional>
#include <memory>
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
        forward_chase_speed_ = this->declare_parameter("forward_chase_speed", 0.1);
        search_angular_speed_ = this->declare_parameter("search_angular_speed", 0.5);
        max_size_thresh_ = this->declare_parameter("max_size_thresh", 0.1);
        filter_value_ = this->declare_parameter("filter_value", 0.9);

        timer_ = this->create_wall_timer(100ms, std::bind(&Docking::timer_cb, this));
        target_val_ = 0.0;
        target_dist_ = 0.0;
        lastrcvtime_ = this->now().seconds() - 10000;
    }

private:
    void timer_cb()
    {
        auto msg = geometry_msgs::msg::Twist();
        double current_time = this->now().seconds();

        if ((current_time - lastrcvtime_) < rcv_timeout_secs_) {
            RCLCPP_INFO(this->get_logger(), "Target: %f", target_val_);
            if (target_dist_ < max_size_thresh_) {
                msg.linear.x = forward_chase_speed_;
            }
            msg.angular.z = -angular_chase_multiplier_ * target_val_;
        } else {
            RCLCPP_INFO(this->get_logger(), "Target lost");
            msg.angular.z = search_angular_speed_;
        }
        publisher_->publish(msg);
    }

    void listener_cb(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        double f = filter_value_;
        target_val_ = target_val_ * f + msg->x * (1 - f);
        target_dist_ = target_dist_ * f + msg->z * (1 - f);
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

    double target_val_;
    double target_dist_;
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
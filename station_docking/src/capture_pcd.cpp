#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <vector>
#include <cmath>

/*
This code captures one instance of the message from Lidar and tries to detect the 
line cross section on this imahe.
Note1: the code is not complete.
Note2: the code is not tested for functionality.
*/
class LidarFrameCapture : public rclcpp::Node {
public:
    LidarFrameCapture() : Node("lidar_capture"), frame_captured_(true) {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LidarFrameCapture::scan_cb, this, std::placeholders::_1));
    }

private:
    void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (frame_captured_) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);

            for (size_t i = 0; i < msg->ranges.size(); ++i) {
                float range = msg->ranges[i];
                if (std::isfinite(range)) {
                    float angle = msg->angle_min + i * msg->angle_increment;
                    float x = range * std::cos(angle);
                    float y = range * std::sin(angle);
                    cloud->points.emplace_back(x, y, 0.0f);
                }
            }

            cloud->width = cloud->points.size();
            cloud->height = 1;
            cloud->is_dense = false;

            pcl::io::savePCDFileASCII("/ros2_ws/lidar_frames/frame.pcd", *cloud);
            RCLCPP_INFO(this->get_logger(), "Frame captured.");
            frame_captured_ = true;
            rclcpp::shutdown();
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    bool frame_captured_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarFrameCapture>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

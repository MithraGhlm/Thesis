#include <chrono>
#include <functional>
#include <memory>
#include <iostream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/laser_scan.h> 
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/filesystem.hpp>
#include "pcl_ros/transforms.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <laser_geometry/laser_geometry.hpp>

//using namespace std::chrono_literals;
// rclcpp::SensorDataQoS()

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

class LidarPclProcessor : public rclcpp::Node
{
public:
  LidarPclProcessor()
  : Node("lidar_pcl_processor")
  {
    // Create a subscriber for LaserScan data
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(), std::bind(&LidarPclProcessor::scan_cb, this, std::placeholders::_1));

    // Publisher for processed LaserScan data
    //scan_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("processed_scan", 10);

    // Publisher for intermediate PointCloud2 data (for visualization or debug)
    pointcloud_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("processed_PC2", 10);
  }

private:
  // Callback function to process the LaserScan data
  void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
  {
    // Converting LaserScan to PointCloud2
    sensor_msgs::msg::PointCloud2 cloud_msg;
    projector_.projectLaser(*scan_msg, cloud_msg);

    // PCL container for filtered point cloud data
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2 cloud_filtered;

    // Converting ROS PointCloud2 to PCL data type
    pcl_conversions::toPCL(cloud_msg, *cloud);

    // Performing PCL downsampling & NAN value removal, hence producing "real" values
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.05f, 0.05f, 0.05f); // leaf size of 5cm
    sor.filter(cloud_filtered);

    // Performing PCL line detection

    // Convert the filtered cloud back to ROS PointCloud2 message
    sensor_msgs::msg::PointCloud2 output_cloud;
    pcl_conversions::moveFromPCL(cloud_filtered, output_cloud);
    output_cloud.header = cloud_msg.header;  // Keep the original header

    // publishing PointCloud2 result
    pointcloud_publisher_->publish(output_cloud);

    // Converting filtered PointCloud2 back to LaserScan (is needed?)
    // implement this conversion if the result in LaserScan format is needed

    // Publishing the final processed LaserScan
    // scan_publisher_->publish(processed_scan_msg);
  }

  // LaserScan to PointCloud2 projector
  laser_geometry::LaserProjection projector_;

  // Subscriber and publishers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_publisher_;
  //rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_publisher_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarPclProcessor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}



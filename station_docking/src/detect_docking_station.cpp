#include <chrono>
#include <functional>
#include <memory>
#include <iostream>
#include <string>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/laser_scan.hpp> 
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/filesystem.hpp>
#include <pcl_ros/transforms.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <laser_geometry/laser_geometry.hpp>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/angles.h>
#include <visualization_msgs/msg/marker_array.hpp>
//#include "lineLineIntersect.hpp"



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

    // Publisher for intermediate PointCloud2 data (visualization or debug)
    pointcloud2_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("processed_PC2", 10);

    marker_pub_=this->create_publisher<visualization_msgs::msg::Marker>("Marker", 10);
  }

private:
  // Callback function to process the LaserScan data
  void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
  {
    // Converting LaserScan to PointCloud2
    sensor_msgs::msg::PointCloud2 cloud_msg;
    projector_.projectLaser(*scan_msg, cloud_msg);

    // Converting ROS PointCloud2 to PCL data type
    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2 cloud_filtered;
    pcl_conversions::toPCL(cloud_msg, *cloud);


    // Performing PCL downsampling & NAN value removal, hence producing "real" values
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.01f, 0.01f, 0.01f); // leaf size of 1cm
    sor.filter(cloud_filtered);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pc(new pcl::PointCloud<pcl::PointXYZ>());

    // Converting from PCLPointCloud2 to PCLPointCloud (for better PCL processing)
    pcl::fromPCLPointCloud2(cloud_filtered, *cloud_pc);
     
    //std::cout << "Number of points in the filtered point cloud: " << cloud_pc->size() << std::endl;

    //==========================================>

    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.004); // Adjust based on point cloud
    //seg.setMaxIterations(1000);


    // Detecting lines using RANSAC
    pcl::ModelCoefficients::Ptr line_coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr line_inliers(new pcl::PointIndices);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    std::vector<pcl::ModelCoefficients::Ptr> coeffs;
    std::vector<pcl::PointIndices::Ptr> inliers;

    // std::cout << "Entering while loop ((((((( " << std::endl;
    // while(true){

    //     seg.setInputCloud(cloud_pc);
    //     seg.segment(*line_inliers, *line_coefficients);

    //     // Check if any line was detected at all
    //     if (line_inliers->indices.empty()) {
    //         break;
    //     }
    //     std::cout << "line_coefficients: " << *line_coefficients << std::endl;
    //     coeffs.push_back(line_coefficients);
        
    //     // Extract the line points from the pointcloud data
    //     extract.setInputCloud(cloud_pc);
    //     extract.setIndices(line_inliers);
    //     extract.setNegative(true);
    //     extract.filter(*cloud_pc);
    // }
    // std::cout << "number of lines found is: " << coeffs.size() << std::endl;
    // std::cout << "Exiting while loop )))))))) " << std::endl;

    // /***************************************************
    // Detect the 1st line using RANSAC
    pcl::ModelCoefficients::Ptr line1_coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr line1_inliers(new pcl::PointIndices);
    seg.setInputCloud(cloud_pc);
    seg.segment(*line1_inliers, *line1_coefficients);
    coeffs.push_back(line1_coefficients);
    inliers.push_back(line1_inliers);
    // std::cout << "line1_coefficients: " << *line1_coefficients << std::endl;
    //publishLine(line1_coefficients, line1_inliers, 1.0, 0.0, 0.0);
    

    // Extract line 1 points from data
    extract.setInputCloud(cloud_pc);
    extract.setIndices(line1_inliers);
    extract.setNegative(true); // if false, removes the outliers of line1
    extract.setKeepOrganized (true);
    extract.filter(*cloud_pc);

    // Detect the 2nd line using RANSAC
    pcl::ModelCoefficients::Ptr line2_coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr line2_inliers(new pcl::PointIndices);
    seg.setInputCloud(cloud_pc);
    seg.segment(*line2_inliers, *line2_coefficients);
    coeffs.push_back(line2_coefficients);
    inliers.push_back(line2_inliers);
    //publishLine(line2_coefficients, line2_inliers, 0.0, 1.0, 0.0);

    // // publish both lines
    // publishCrossMarker(line1_coefficients, line2_coefficients, line1_inliers, line2_inliers);
    // ****************************************************/

    // Calculate intersection point (approximation for now)
    findCrossShape(coeffs, inliers);

    //==========================================>

    // // Printing the x y z of each PointCloud point
    // for (size_t i=0; i<cloud_pc->size(); i++){
    //     const auto& point = cloud_pc->points[i];
    //     std::cout << "Point " << i << ": ["
    //           << "x = " << point.x << ", "
    //           << "y = " << point.y << ", "
    //           << "z = " << point.z << "]"
    //           << std::endl;
    // }

    // // check if the dataset is organized or not
    // std::string isOrganized = (cloud_pc.isOrganized()) ? "Dataset is organized." : "Dataset is unorganized.";
    // std::cout << isOrganized << std::endl;
    // std::cout << "The dataset height is: " << cloud_pc.height << std::endl;
    

    // Performing PCL line detection

    
    // Converting back from PCLPointCloud to PCLPointCloud2 (for better compatibility with ROS)
    pcl::toPCLPointCloud2(*cloud_pc, cloud_filtered);


    // Convert the filtered cloud back to ROS PointCloud2 message
    sensor_msgs::msg::PointCloud2 output_cloud;
    pcl_conversions::moveFromPCL(cloud_filtered, output_cloud);
    output_cloud.header = cloud_msg.header;  // Keep the original header

    // publishing PointCloud2 result
    pointcloud2_publisher_->publish(output_cloud);

    // Converting filtered PointCloud2 back to LaserScan (is needed?)
    // implement this conversion if the result in LaserScan format is needed

    // Publishing the final processed LaserScan
    // scan_publisher_->publish(processed_scan_msg);


  } // scan_cb


  void findCrossShape(const std::vector<pcl::ModelCoefficients::Ptr> &coeffs, const std::vector<pcl::PointIndices::Ptr> &inliers)
  {
    // Defining a target angle for "cross" detection (in radians)
    const float target_angle = pcl::deg2rad(60.0f); 
    const float angle_tolerance = pcl::deg2rad(5.0f); // Acceptable tolerance

    for (size_t i = 0; i < coeffs.size(); ++i)
    {
      for (size_t j = i + 1; j < coeffs.size(); ++j)
      {
        // Extract direction vectors for lines i and j
        Eigen::Vector2f dir_i(coeffs[i]->values[3], coeffs[i]->values[4]);
        Eigen::Vector2f dir_j(coeffs[j]->values[3], coeffs[j]->values[4]);

        // Calculate the angle between the two direction vectors
        float angle = std::acos(dir_i.dot(dir_j) / (dir_i.norm() * dir_j.norm()));

        // Check if the angle is close to the target angle for a cross shape
        if (std::abs(angle - target_angle) < angle_tolerance)
        {
          //RCLCPP_INFO(this->get_logger(), "Cross detected between lines %ld and %ld", i, j);
          publishCrossMarker(coeffs[i], coeffs[j], inliers[i], inliers[j]);
          return;
        }
      }
    }
  }

  void publishCrossMarker(const pcl::ModelCoefficients::Ptr &line1, const pcl::ModelCoefficients::Ptr &line2, const pcl::PointIndices::Ptr &inliers1, const pcl::PointIndices::Ptr &inliers2)
  {
    // Create markers for visualization in Rviz
    visualization_msgs::msg::Marker marker1, marker2;

    marker1.header.frame_id = marker2.header.frame_id = "laser_frame";
    marker1.header.stamp = marker2.header.stamp = this->get_clock()->now();
    marker1.ns = "line1";
    marker2.ns = "line2";
    marker1.id = 0;
    marker2.id = 1;
    marker1.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker2.type = visualization_msgs::msg::Marker::LINE_STRIP;
    // marker1.action = visualization_msgs::msg::Marker::ADD;
    // marker2.action = visualization_msgs::msg::Marker::ADD;
    // Set line width
    marker1.scale.x = marker2.scale.x = 0.03; 
    marker1.scale.y = marker2.scale.y = 0.03; 
    marker1.scale.z = marker2.scale.z = 0.0;

    // Set color
    marker1.color.r = 1.0;
    marker1.color.g = 0.0;
    marker1.color.b = 0.0;
    marker1.color.a = 1.0;
    marker2.color.r = 0.0;
    marker2.color.g = 1.0;
    marker2.color.b = 0.0;
    marker2.color.a = 1.0;

    // Define points for line 1 and line 2
    geometry_msgs::msg::Point p1_start, p1_end, p2_start, p2_end;
    p1_start.x = line1->values[0]; p1_start.y = line1->values[1]; p1_start.z = 0.0;
    p1_end.x = p1_start.x + line1->values[3]; 
    p1_end.y = p1_start.y + line1->values[4]; 

    p2_start.x = line2->values[0]; p2_start.y = line2->values[1]; p2_start.z = 0.0;
    p2_end.x = p2_start.x + 0.5*line2->values[3]; 
    p2_end.y = p2_start.y + 0.5*line2->values[4]; 

    marker1.points.push_back(p1_start); marker1.points.push_back(p1_end);
    marker2.points.push_back(p2_start); marker2.points.push_back(p2_end);

    // // Publish markers
    if(inliers1->indices.size() > 30){
      //marker_pub_->publish(marker1);
      marker1.action = visualization_msgs::msg::Marker::ADD; //ADD=0, DELETE=2
      marker_pub_->publish(marker1);
      //publishLineStartPoint(p1_start, visualization_msgs::msg::Marker::ADD);

    } else { // delete the marker if the line is out of scope
      marker1.action = visualization_msgs::msg::Marker::DELETE;
      marker_pub_->publish(marker1);
      //publishLineStartPoint(p1_start, visualization_msgs::msg::Marker::DELETE);
    }
    if(inliers2->indices.size() > 30){
      marker2.action = visualization_msgs::msg::Marker::ADD;
      marker_pub_->publish(marker2);
      //publishLineStartPoint(p2_start, visualization_msgs::msg::Marker::ADD);
    } else {
      marker2.action = visualization_msgs::msg::Marker::DELETE;
      marker_pub_->publish(marker2);
      //publishLineStartPoint(p2_start, visualization_msgs::msg::Marker::DELETE);
    }
  }

  void publishLineStartPoint(const geometry_msgs::msg::Point& point, const int32_t& action) //action: ADD=0, DELETE=2
  {
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "laser_frame";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "point";
    marker.id = 2;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = action;

    marker.scale.x = 0.05; 
    marker.scale.y = 0.05; 
    marker.scale.z = 0.05;

    // Set color
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    // Set position for CUBE
    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.pose.position.z = 0.0;

    marker_pub_->publish(marker);
  }

  bool is_close_enough(){
    return true;
  }

  // publish only one line
  void publishLine(const pcl::ModelCoefficients::Ptr &line, const pcl::PointIndices::Ptr &inliers, const float r, const float g, const float b){
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "laser_frame";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "line";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.03; marker.scale.y = 0.03; marker.scale.z = 0.0;

    //color
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;

    // Define points for line 1
    geometry_msgs::msg::Point p_start, p_end;
    p_start.x = line->values[0]; p_start.y = line->values[1]; //p_start.z = 0.0;
    p_end.x = p_start.x + 0.5* line->values[3]; 
    p_end.y = p_start.y + 0.5* line->values[4];

    marker.points.push_back(p_start); marker.points.push_back(p_end);

    // setting threshold for the minimum number of inliers in a line
    if(inliers->indices.size() > 40){
        marker_pub_->publish(marker);
    }
    

    std::cout << "number of inliers: " << inliers->indices.size() << std::endl;
  }


  // LaserScan to PointCloud2 projector
  laser_geometry::LaserProjection projector_;

  // Subscriber and publishers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud2_publisher_;

  //publisher for visualization markers
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;
};


 // The six coefficients of the line:
 // [point_on_line.x point_on_line.y point_on_line.z line_direction.x line_direction.y line_direction.z]
 inline std::ostream& operator<<(std::ostream& s, const  ::pcl::ModelCoefficients & v)
  {
    s << "header: " << std::endl;
    s << v.header;
    s << "values[]" << std::endl;
    for (std::size_t i = 0; i < v.values.size (); ++i)
    {
      s << "  values[" << i << "]: ";
      s << "  " << v.values[i] << std::endl;
    }
    return (s);
  }



int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarPclProcessor>();
  rclcpp::spin(node);

  // Create a lambda function to spin the node in a separate thread
  auto spin_thread = std::thread([&node]() {
    rclcpp::spin(node);
  });

  // Create a Rate object with 10 Hz
  rclcpp::Rate rate(100);

  try {
        while (rclcpp::ok()) {
            std::cout << "Help me body, you are my only hope" << std::endl;
            rate.sleep();
        }
    } catch (const std::exception &e) {
        // Handle exceptions if needed
        std::cerr << "Exception: " << e.what() << std::endl;
    }


  // Shutdown the ROS 2 client library
  rclcpp::shutdown();

  // Join the spin thread to clean up
  if (spin_thread.joinable()) {
      spin_thread.join();
  }

  return 0;
}



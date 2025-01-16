#include <chrono>
#include <functional>
#include <memory>
#include <iostream>
#include <string>
#include <math.h>
#include <stdio.h>
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
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <cstdlib>
#include <ctime>
#include <fstream>

#define DBG_INLIER        0
#define DBG_ANGLE         0
#define DBG_TARGET        0
#define DBG_FRAMESCAN     0
#define INLIER_NUM 10
#define ANGLE_DVA  10.0f
#define START_P_DIST 0.33f
#define SAMPLE_NUM 10
#define DISTANCE_THRESHOLD 0.01
#define MAX_ITERATION 100
#define DS_DIS            2.0f

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

    marker_pub_=this->create_publisher<visualization_msgs::msg::Marker>("Line_Marker", 10);
    marker_pub_1_=this->create_publisher<visualization_msgs::msg::Marker>("Line1_Marker", 10);
    marker_pub_2_=this->create_publisher<visualization_msgs::msg::Marker>("Line2_Marker", 10);
    marker_pub_3_=this->create_publisher<visualization_msgs::msg::Marker>("Intersection_Marker", 10);
    intersectPoint_pub_=this->create_publisher<geometry_msgs::msg::Point>("Intersection_Point", 10);
    angles_dbg.open("/ros2_ws/lidar_frames/angles.txt");
    target_dbg.open("/ros2_ws/lidar_frames/targets.txt");
    inlier_dbg.open("/ros2_ws/lidar_frames/inliers.txt");
  }
  
  ~LidarPclProcessor() {
    if (angles_dbg.is_open()) {
            angles_dbg.close();
    }
    if (target_dbg.is_open()) {
            target_dbg.close();
    }
    if (inlier_dbg.is_open()) {
            inlier_dbg.close();
    }
  }


private:
  int scan_num = 1;
  std::ofstream angles_dbg;
  std::ofstream target_dbg;
  std::ofstream inlier_dbg;

  // Callback function to process LaserScan data
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
    sor.setLeafSize(0.0005f, 0.0005f, 0.0005f); // leaf size of 0.5cm
    sor.filter(cloud_filtered);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pc(new pcl::PointCloud<pcl::PointXYZ>());

    // Converting from PCLPointCloud2 to PCLPointCloud (for better PCL processing)
    pcl::fromPCLPointCloud2(cloud_filtered, *cloud_pc);
     
    //std::cout << "Number of points in the filtered point cloud: " << cloud_pc->size() << std::endl;

    //==========================================>

    pcl::SACSegmentation<pcl::PointXYZ> seg(true);
    seg.setOptimizeCoefficients(false);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC); //SAC_MLESAC , SAC_RANSAC
    seg.setDistanceThreshold(DISTANCE_THRESHOLD); // maximum distance a point can be from the model to be considered an inlier, in meters(?)
    seg.setMaxIterations(MAX_ITERATION);
    seg.setNumberOfThreads(4);
    //seg.setRadiusLimits(0.001f, 0.5f);

    // Detecting lines using RANSAC
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    std::vector<pcl::ModelCoefficients::Ptr> coeffs;
    std::vector<pcl::PointIndices::Ptr> inliers;

    int frame_num = 1;
    // Loop to detect all possible lines
    while (cloud_pc->points.size() > 30) { // Stop if there are few points left in the cloud

        pcl::ModelCoefficients::Ptr line_coefficients(new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr line_inliers(new pcl::PointIndices);
        seg.setInputCloud(cloud_pc);
        seg.segment(*line_inliers, *line_coefficients);

        // Check if sufficient inliers are found to define a line
        // if (line_inliers->indices.size() < 10) {
        //     //std::cout << "No more lines detected or insufficient inliers." << std::endl;
        //     extract.setInputCloud(cloud_pc);
        //     extract.setIndices(line_inliers);
        //     extract.setNegative(true);  // Remove line inliers from cloud
        //     continue;
        // }

        // limiting the expand of inliers
        //std::cout << " number of inliers before: " << line_inliers->indices.size() << std::endl;
        limit_line_length(cloud_pc, line_coefficients, line_inliers);
        //std::cout << " number of inliers after: " << line_inliers->indices.size() << std::endl;



        // Store the coefficients and inliers if line detected
        if (line_inliers->indices.size() > INLIER_NUM){
          coeffs.push_back(line_coefficients);
          inliers.push_back(line_inliers);
#if DBG_INLIER
          inlier_dbg << scan_num << "," << frame_num << "\n";
#endif
        }
#if DBG_FRAMESCAN
        std::string filename1 = "/ros2_ws/lidar_frames/" + std::to_string(scan_num) + "A_f" + std::to_string(frame_num) + ".pcd";
        pcl::io::savePCDFileASCII(filename1, *cloud_pc);
#endif
        // std::cout << "Line detected with coefficients: ";
        // for (const auto &coef : line_coefficients->values) {
        //     std::cout << coef << " ";
        // }
        // std::cout << std::endl;

        // Remove the detected line points from the cloud
        extract.setInputCloud(cloud_pc);
        extract.setIndices(line_inliers); 
        extract.setNegative(true);  // Remove line inliers from cloud
        extract.filter(*cloud_pc);
        //std::cout << "cloud_pc size: " << cloud_pc->points.size() << std::endl;
        frame_num++;
    }

    // std::cout << "number of lines: " << coeffs.size() << std::endl; 
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

    
    // Converting back from PCLPointCloud to PCLPointCloud2 (for better compatibility with ROS)
    pcl::toPCLPointCloud2(*cloud_pc, cloud_filtered);


    // Convert the filtered cloud back to ROS PointCloud2 message
    sensor_msgs::msg::PointCloud2 output_cloud;
    pcl_conversions::moveFromPCL(cloud_filtered, output_cloud);
    output_cloud.header = cloud_msg.header;  // Keep the original header

    // publishing PointCloud2 result
    pointcloud2_publisher_->publish(output_cloud);
    scan_num++;
  } // scan_cb


  void findCrossShape(const std::vector<pcl::ModelCoefficients::Ptr> &coeffs, const std::vector<pcl::PointIndices::Ptr> &inliers)
  {
    // Defining a target angle for "cross" detection (in radians)
    float target_angle = pcl::deg2rad(60.0f); 
    const float angle_tolerance = pcl::deg2rad(ANGLE_DVA); // Acceptable tolerance

    int found_angl = 0;
    int found_dist = 0;
    std::vector<float> angles;
    angles.push_back(static_cast<float>(scan_num));

    for (size_t i = 0; i < coeffs.size(); ++i)
    {
      publishLine(coeffs[i], inliers[i], 1, 0, 0);
      for (size_t j = i + 1; j < coeffs.size(); ++j)
      {
        // Extract direction vectors for lines i and j
        Eigen::Vector2f dir_i(coeffs[i]->values[3], coeffs[i]->values[4]);
        Eigen::Vector2f dir_j(coeffs[j]->values[3], coeffs[j]->values[4]);
        dir_i.normalize();
        dir_j.normalize();
        float dot_product = dir_i.dot(dir_j);
        if (dot_product > 0) {
            target_angle = pcl::deg2rad(60.0f); 
        } else if (dot_product < 0) {
            target_angle = pcl::deg2rad(120.0f); 
        } else {
            std::cout << "The lines are perpendicular." << std::endl;
        }
        // Calculate the angle between the two direction vectors
        float angle = std::acos(dir_i.dot(dir_j) / (dir_i.norm() * dir_j.norm()));
        angles.push_back(pcl::rad2deg(angle));
          
        // Check if the angle is close to the target angle for a cross shape
        if (std::abs(angle - target_angle) < angle_tolerance)
        {
          found_angl++;
          bool tmp = publishCrossMarker(coeffs[i], coeffs[j]); //, inliers[i], inliers[j]);
          if(tmp) found_dist++;
          std::cout << "angle between line " << i << " and " << j << " is: " << pcl::rad2deg(angle) << std::endl;
          //return;
        }
      }
    }
    std::cout << "#scan " << scan_num << ", fnd_angl " << found_angl << ", fnd_dist " << found_dist << ", #lines " << coeffs.size() << std::endl;

    // capture the frame that no cross section was fined in it
#if DBG_ANGLE
    if(!found_dist) {
      if(!found_angl) {
        angles_dbg << "ANGL_NOT_FOUND,";
      }
      // dump all angles to file
      for (size_t i = 0; i < angles.size(); ++i) {
        angles_dbg << angles[i] << (i + 1 < angles.size() ? "," : "");
      }
      angles_dbg << "\n";
    }
#endif
  }

  bool publishCrossMarker(const pcl::ModelCoefficients::Ptr &line1, const pcl::ModelCoefficients::Ptr &line2) //, const pcl::PointIndices::Ptr &inliers1, const pcl::PointIndices::Ptr &inliers2)
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

    // Define start and end points for line 1 and line 2
    geometry_msgs::msg::Point p1_start, p1_end, p2_start, p2_end;
    p1_start.x = line1->values[0]; p1_start.y = line1->values[1]; p1_start.z = 0.0;
    p1_end.x = p1_start.x + 0.5*line1->values[3]; 
    p1_end.y = p1_start.y + 0.5*line1->values[4]; 

    p2_start.x = line2->values[0]; p2_start.y = line2->values[1]; p2_start.z = 0.0;
    p2_end.x = p2_start.x + 0.5*line2->values[3]; 
    p2_end.y = p2_start.y + 0.5*line2->values[4]; 

    marker1.points.push_back(p1_start); marker1.points.push_back(p1_end);
    marker2.points.push_back(p2_start); marker2.points.push_back(p2_end);

    // intermidiate variable to hold marker action
    int32_t marker_action;

    // check if the the starting points of lines are not too far away
    bool close_enough = is_close_enough(p1_start, p2_start, START_P_DIST);
    
    if(close_enough){
      //if(inliers1->indices.size() > INLIER_NUM && inliers2->indices.size() > INLIER_NUM){ // Publish markers
        marker1.action = visualization_msgs::msg::Marker::ADD;
        marker2.action = visualization_msgs::msg::Marker::ADD;
        marker_action = visualization_msgs::msg::Marker::ADD;
        publishInterSectionPoint(line1,line2, marker_action, true);
      } else { // delete the marker if the line is out of scope
        marker1.action = visualization_msgs::msg::Marker::DELETE;
        marker2.action = visualization_msgs::msg::Marker::DELETE;
        marker_action = visualization_msgs::msg::Marker::DELETE;
        publishInterSectionPoint(line1,line2, marker_action, false);
      }
    
      marker_pub_1_->publish(marker1);
      marker_pub_2_->publish(marker2);
      publishLineStartPoint(p1_start, marker_action);
      
    //}
    return close_enough;
  }

  // calculate determinant
  inline double Det(double a, double b, double c, double d)
  {
    return a*d - b*c;
  }

  //Calculate intersection of two lines.
  void publishInterSectionPoint(const pcl::ModelCoefficients::Ptr &line1, const pcl::ModelCoefficients::Ptr &line2, const int32_t& action, bool pub2motor)
  {
    // 2D Line-line intersection (using determinants)
    double ixOut, iyOut; // the output intersection point
    geometry_msgs::msg::Point p1_start, p1_end, p2_start, p2_end;
    geometry_msgs::msg::Point intersectPoint;
    p1_start.x = line1->values[0]; p1_start.y = line1->values[1]; p1_start.z = 0.0;
    p1_end.x = p1_start.x + line1->values[3]; 
    p1_end.y = p1_start.y + line1->values[4]; 

    p2_start.x = line2->values[0]; p2_start.y = line2->values[1]; p2_start.z = 0.0;
    p2_end.x = p2_start.x + 0.5*line2->values[3]; 
    p2_end.y = p2_start.y + 0.5*line2->values[4];

    double detL1 = Det(p1_start.x, p1_start.y, p1_end.x, p1_end.y);
    double detL2 = Det(p2_start.x, p2_start.y, p2_end.x, p2_end.y);
    double x1mx2 = p1_start.x - p1_end.x;
    double x3mx4 = p2_start.x - p2_end.x;
    double y1my2 = p1_start.y - p1_end.y;
    double y3my4 = p2_start.y - p2_end.y;

    double xnom = Det(detL1, x1mx2, detL2, x3mx4);
    double ynom = Det(detL1, y1my2, detL2, y3my4);
    double denom = Det(x1mx2, y1my2, x3mx4, y3my4);

    if(denom == 0.0)//Lines don't seem to cross
    {
      ixOut = NAN;
      iyOut = NAN;
    }

    ixOut = xnom / denom;
    iyOut = ynom / denom;
    if(!isfinite(ixOut) || !isfinite(iyOut)) //Probably a numerical issue
      RCLCPP_INFO(this->get_logger(), "There has been a numerical issue in calculating the intersection point.");

    // End of line detection

    
    // Marker for intersection point
    visualization_msgs::msg::Marker marker;

    marker.header.frame_id = "laser_frame";
    marker.header.stamp = this->get_clock()->now();
    marker.ns = "intersection";
    marker.id = 3;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = action;

    marker.scale.x = 0.1; 
    marker.scale.y = 0.1; 
    marker.scale.z = 0.1;

    // Set color
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    // Set position for SPHERE
    marker.pose.position.x = ixOut;
    marker.pose.position.y = iyOut;
    marker.pose.position.z = 0.0;

    marker_pub_3_->publish(marker);


    // Topic publishing the point for robot to follow
    if(pub2motor) {
      // only accept intersection points which are in a 2x2 m² circumference
      //TODO: change it to radius
      bool x_in_range = (ixOut < DS_DIS);

       if (x_in_range){
        std::cout << "(ixOut, iyOut): (" << ixOut << ", " << iyOut << ")" << std::endl;
        
        intersectPoint.x = ixOut;
        intersectPoint.y = iyOut;
        intersectPoint.z = 0.0;
        intersectPoint_pub_->publish(intersectPoint);
#if DBG_TARGET
      target_dbg << scan_num << "," << ixOut << "," << iyOut << "\n";
#endif
      }
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

    marker.scale.x = 0.08; 
    marker.scale.y = 0.08; 
    marker.scale.z = 0.08;

    // Set color
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    // Set position for CUBE
    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.pose.position.z = 0.0;

    marker_pub_->publish(marker);
  }

  // returns true if 2 points are closer than a threshold
  bool is_close_enough(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2, const float thresh){
    const float delt_x = p1.x - p2.x;
    const float delt_y = p1.y - p2.y;
    const float dist = sqrt(pow(delt_x, 2) + pow(delt_y, 2));
    if(dist > thresh){
      return false;
    }
    // std::cout << "Distance between points is: " << dist << std::endl;
#if DBG_TARGET
    if(thresh >= START_P_DIST) target_dbg << dist << ",";
#endif
    return true;
  }


  // function to limit the number of points counted as inliers of a line
  void limit_line_length(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud_pc, const pcl::ModelCoefficients::Ptr& line, pcl::PointIndices::Ptr& inliers)
  {
    geometry_msgs::msg::Point p_start;
    p_start.x = line->values[0]; 
    p_start.y = line->values[1];
    //p_start.z = 0.0;

    std::vector<int> inliers_filtered;

    for ( auto &indx : inliers->indices){
      const auto &point_xyz = cloud_pc->points[indx];

    // Convert pcl::PointXYZ to geometry_msgs::msg::Point
      geometry_msgs::msg::Point point;
      point.x = point_xyz.x;
      point.y = point_xyz.y;
      point.z = 0.0f;

      if ( is_close_enough(p_start, point, 0.2f) ){
        inliers_filtered.push_back(indx); // Keep this index if it's close enough
      }
    }

    // Replace the inliers vector with the filtered indices
    inliers->indices = inliers_filtered;

  }

  // publish only one line
  void publishLine(const pcl::ModelCoefficients::Ptr &line, const pcl::PointIndices::Ptr &inliers, const float r, const float g, const float b)
  {
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

    // Define points for line
    geometry_msgs::msg::Point p_start, p_end;
    p_start.x = line->values[0]; p_start.y = line->values[1]; //p_start.z = 0.0;
    p_end.x = p_start.x + 0.5* line->values[3]; 
    p_end.y = p_start.y + 0.5* line->values[4];

    marker.points.push_back(p_start); marker.points.push_back(p_end);

    // setting threshold for the minimum number of inliers in a line
     marker_pub_->publish(marker);
    
    //std::cout << "number of inliers: " << inliers->indices.size() << std::endl;
  }


  // LaserScan to PointCloud2 projector
  laser_geometry::LaserProjection projector_;

  // Subscriber and publishers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud2_publisher_;

  //publisher for visualization markers
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;  // start point of lines
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_1_; // line 1
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_2_; // line 2
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_3_; // two lines intersection point
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr intersectPoint_pub_; // intersection point to send to motion control
  
}; // end of class LidarPclProcessor


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

  // Shutdown the ROS 2 client library
  rclcpp::shutdown();

  return 0;
}



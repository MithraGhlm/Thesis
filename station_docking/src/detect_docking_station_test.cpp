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
#include <numeric>

#define DBG_INLIER        0
#define DBG_ANGLE         0
#define DBG_TARGET        0
#define DBG_FRAMESCAN     0
#define SAMPLE_NUM 10

class LidarPclProcessor : public rclcpp::Node
{
public:
  LidarPclProcessor()
  : Node("lidar_pcl_processor"),
  INLIER_NUM(10), ANGLE_DVA(10.0f), START_P_DIST(0.33f), DISTANCE_THRESHOLD(0.01), MAX_ITERATION(100)
  {
    // Create a subscriber for LaserScan data
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", rclcpp::SensorDataQoS(), std::bind(&LidarPclProcessor::scan_cb, this, std::placeholders::_1));

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
    test_file.open("/ros2_ws/lidar_frames/test.txt");
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
    if (test_file.is_open()) {
            test_file.close();
    }
  }

  //==================================
  // Setter
  void setInlierNum(int value){
    INLIER_NUM = value;
  }
  void setAngleDeviation(float value){
    ANGLE_DVA = value;
  }
  void setStartingPointDistance(float value){
    START_P_DIST = value;
  }
  void setDistanceThreshold(float value){
    DISTANCE_THRESHOLD = value;
  }
  void setMaxIteration(int value){
    MAX_ITERATION = value;
  }
  
  //================================
  // test process management
  void resetScanCounter() { scan_num = 0; }
  void enableTesting(bool enable) { testing_enabled_ = enable; }
  bool isTestComplete() const { return testing_enabled_ && scan_num >= target_scans_; }


private:

  int INLIER_NUM;
  float ANGLE_DVA;
  float START_P_DIST;
  float DISTANCE_THRESHOLD;
  int MAX_ITERATION;

  int test_num = 0;
  int scan_num = 0;
  int corr_detect_scan_num = 0;
  int target_scans_ = 100; // Num of scans per configuration
  bool testing_enabled_ = false;
  std::ofstream angles_dbg;
  std::ofstream target_dbg;
  std::ofstream inlier_dbg;
  std::ofstream test_file;

  // variables for SMA
  std::vector<std::pair<float, float>> intersect_points;

  // Callback function to process LaserScan data
  void scan_cb(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
  {
    if (!testing_enabled_) return;
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
     
    //==========================================>

    pcl::SACSegmentation<pcl::PointXYZ> seg(true);
    seg.setOptimizeCoefficients(false);
    seg.setModelType(pcl::SACMODEL_LINE);
    seg.setMethodType(pcl::SAC_RANSAC); //SAC_MLESAC , SAC_RANSAC
    seg.setDistanceThreshold(DISTANCE_THRESHOLD); // maximum distance a point can be from the model to be considered an inlier
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

        // limiting the expand of inliers
        limit_line_length(cloud_pc, line_coefficients, line_inliers);

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

        // Remove the detected line points from the cloud
        extract.setInputCloud(cloud_pc);
        extract.setIndices(line_inliers); 
        extract.setNegative(true);  // Remove line inliers from cloud
        extract.filter(*cloud_pc);
        frame_num++;
    }

    // Calculate intersection point (approximation for now)
    findCrossShape(coeffs, inliers);

    //==========================================

    // Converting back from PCLPointCloud to PCLPointCloud2 (for better compatibility with ROS)
    pcl::toPCLPointCloud2(*cloud_pc, cloud_filtered);


    // Convert the filtered cloud back to ROS PointCloud2 message
    sensor_msgs::msg::PointCloud2 output_cloud;
    pcl_conversions::moveFromPCL(cloud_filtered, output_cloud);
    output_cloud.header = cloud_msg.header;  // Keep the original header

    // publishing PointCloud2 result
    pointcloud2_publisher_->publish(output_cloud);
    scan_num++;
     if (scan_num >= target_scans_) {
      // writing to file
      test_file << MAX_ITERATION << "," << INLIER_NUM << "," << ANGLE_DVA << "," << DISTANCE_THRESHOLD << "," << START_P_DIST << "," << scan_num << "," << corr_detect_scan_num << "\n";
      test_file.flush();
      RCLCPP_INFO(this->get_logger(), "Configuration test complete!");
    }
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
          //std::cout << "angle between line " << i << " and " << j << " is: " << pcl::rad2deg(angle) << std::endl;
          //return;
        }
      }
    }
    // std::cout << "#scan " << scan_num << ", fnd_angl " << found_angl << ", fnd_dist " << found_dist << ", #lines " << coeffs.size() << std::endl;

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
      
    return close_enough;
  }

  // calculate determinant
  inline double Det(double a, double b, double c, double d)
  {
    return a*d - b*c;
  }

  //Calculate intersection of two lines
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

    marker.scale.x = 0.08; 
    marker.scale.y = 0.08; 
    marker.scale.z = 0.08;

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

    //=================================
    if(pub2motor) {

      // only accept intersection points which are in a 1.5x1.5 m² circumference
      bool x_in_range = (ixOut < 1.5 && ixOut > -1.5);
      bool y_in_range = (iyOut < 1.5 && iyOut > -1.5);
 
      if (x_in_range && y_in_range){
        std::cout << "(ixOut, iyOut): (" << ixOut << ", " << iyOut << ")" << std::endl;

#if !DBG_SMA
        intersectPoint.x = ixOut;
        intersectPoint.y = iyOut;
#else
        float Xs = 0.0; float Ys = 0.0; // holding the sums 
        float x_SMA = 0.0; float y_SMA = 0.0; // holding the means

        // simple moving average (SMA)
        intersect_points.push_back({ixOut, iyOut});

        if(intersect_points.size() > SAMPLE_NUM -1){
          // for (const auto& point : intersect_points) {
          //     std::cout << "(" << point.first << ", " << point.second << ")" << std::endl;}

          // sum all Xs and all Ys
          for (const auto& row:intersect_points){
            Xs += row.first;
            Ys += row.second;
          }
          // std::cout << "sum of Xs and Ys: " << Xs << ", " << Ys << std::endl;

          // Remove the first element of the vector (FIFO)
          intersect_points.erase(intersect_points.begin()); 
            
          x_SMA = Xs / SAMPLE_NUM;
          y_SMA = Ys / SAMPLE_NUM;

          intersectPoint.x = x_SMA;
          intersectPoint.y = y_SMA;
    //=================================

#endif

          intersectPoint.z = 0.0;  // <<====== Should it be zero or sth else?

          // Topic publishing the point for robot to follow
          intersectPoint_pub_->publish(intersectPoint);
          corr_detect_scan_num ++;

#if DBG_TARGET
          target_dbg << scan_num << "," << ixOut << "," << iyOut << "\n";
#endif
#if DBG_SMA
        }
#endif
      }
    }
  }

  // returns true if 2 points are closer than a threshold
  bool is_close_enough(const geometry_msgs::msg::Point &p1, const geometry_msgs::msg::Point &p2, const float thresh){
    const float delt_x = p1.x - p2.x;
    const float delt_y = p1.y - p2.y;
    const float dist = sqrt(pow(delt_x, 2) + pow(delt_y, 2));
    if(dist > thresh){
      return false;
    }

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
    // if(inliers->indices.size() > 40){
     marker_pub_->publish(marker);
    // }
    
    //std::cout << "number of inliers: " << inliers->indices.size() << std::endl;
  }


  // LaserScan to PointCloud2 projector
  laser_geometry::LaserProjection projector_;

  // Subscribers and publishers
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud2_publisher_;

  //publishers for visualization markers
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;  // start point of lines
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_1_; // line 1
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_2_; // line 2
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_3_; // two lines intersection point
  rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr intersectPoint_pub_; // intersection point to send to motion control


}; // end of class LidarPclProcessor


int main(int argc, char **argv)
{
  //=====================================
  // Testing values
  std::vector<int> max_iteration = {50, 100, 200, 500}; // iteration of RANSAC
  std::vector<int> num_of_inliers = {10, 15, 20, 25}; // min allowed number of inliers in a line
  std::vector<float> model_angle_deviation = {3.0, 5.0, 10.0}; // model sides angle deviation tolerance (deg)
  std::vector<float> distance_thresh = {0.0025, 0.005, 0.01};   // how close points are to the model (meters)
  std::vector<float> line_strtPoint_dists = {0.20, 0.33, 0.42, 0.50}; // distance between the 2 lines starting points (meters)
  //=====================================

  rclcpp::init(argc, argv);
  auto node = std::make_shared<LidarPclProcessor>();

  //=====================================
  // Testing loops
  
  for (int iterationNum : max_iteration){
    std::cout << "max_iteration number: " << iterationNum << std::endl;
    node->setMaxIteration(iterationNum);

    for (int inlierNum : num_of_inliers){
      std::cout << "number_of_inliers: " << inlierNum << std::endl;
      node->setInlierNum(inlierNum);

      for (float angleDev : model_angle_deviation){
        std::cout << "model_angle_deviation: " << angleDev << std::endl;
        node->setAngleDeviation(angleDev);

        for (float distance : distance_thresh){
          std::cout << "distance_threshold: " << distance << std::endl;
          node->setDistanceThreshold(distance);

          for (float points_distance : line_strtPoint_dists){
            std::cout << "line_startingPoint_distants: " << points_distance << std::endl;
            node->setStartingPointDistance(points_distance);

            // Preparing the node to start the next test
            node->resetScanCounter();
            node->enableTesting(true);

            // Wait until 1000 scans get processed
            while (!node->isTestComplete()) {
              rclcpp::spin_some(node);
              std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            // Disabling test for this configuration
            node->enableTesting(false);
          }
        }
      }
    }
  }
  //=====================================

  //rclcpp::spin(node);

  // Shutdown ROS 2 client library
  rclcpp::shutdown();

  return 0;
}



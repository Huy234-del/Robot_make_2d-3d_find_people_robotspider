#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/int32.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "cv_bridge/cv_bridge.h"

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/dnn.hpp>

class PerceptionNode : public rclcpp::Node
{
public:
  PerceptionNode();
  ~PerceptionNode();

private:
  void initialize_yolo_model();
  void synchronized_callback(
    const sensor_msgs::msg::Image::SharedPtr camera_msg,
    const sensor_msgs::msg::LaserScan::SharedPtr lidar_msg,
    const std_msgs::msg::Int32::SharedPtr tof_msg,
    const sensor_msgs::msg::Image::SharedPtr thermal_msg);
  
  // ROS2 subscribers and publishers
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr tof_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr thermal_sub_;
  
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr detection_pub_;
  
  // Message filters for synchronized callback
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> camera_sync_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>> lidar_sync_sub_;
  std::shared_ptr<message_filters::Subscriber<std_msgs::msg::Int32>> tof_sync_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::Image>> thermal_sync_sub_;
  
  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, 
    sensor_msgs::msg::LaserScan, 
    std_msgs::msg::Int32, 
    sensor_msgs::msg::Image> SyncPolicy;
  
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  
  // Neural network models
  cv::dnn::Net yolo_net_;
  
  // Configuration
  std::vector<std::string> class_names_;
  double confidence_threshold_;
  double thermal_threshold_;
  
  // For temporal filtering
  std::map<int, std::vector<std::pair<rclcpp::Time, double>>> detection_history_;
};
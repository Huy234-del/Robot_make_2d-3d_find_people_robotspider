#pragma once

#include "rclcpp/rclcpp.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"

// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

class FusionNode : public rclcpp::Node
{
public:
  FusionNode();
  ~FusionNode();

private:
  void synchronized_callback(
    const vision_msgs::msg::Detection2DArray::SharedPtr detection_msg,
    const sensor_msgs::msg::LaserScan::SharedPtr lidar_msg);
  
  void update_occupancy_grid(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
  void publish_human_markers(const std::vector<geometry_msgs::msg::Point>& human_positions);
  
  // ROS2 subscribers and publishers
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detection_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
  
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr human_marker_pub_;
  
  // Message filters for synchronized callback
  std::shared_ptr<message_filters::Subscriber<vision_msgs::msg::Detection2DArray>> detection_sync_sub_;
  std::shared_ptr<message_filters::Subscriber<sensor_msgs::msg::LaserScan>> lidar_sync_sub_;
  
  typedef message_filters::sync_policies::ApproximateTime<
    vision_msgs::msg::Detection2DArray, 
    sensor_msgs::msg::LaserScan> SyncPolicy;
  
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  
  // Occupancy grid
  nav_msgs::msg::OccupancyGrid occupancy_grid_;
  double grid_resolution_;
  int grid_width_;
  int grid_height_;
};
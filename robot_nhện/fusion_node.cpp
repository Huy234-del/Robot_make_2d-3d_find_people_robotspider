#include "rescue_robot/fusion_node.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

FusionNode::FusionNode() : Node("fusion_node")
{
  // Initialize parameters
  this->declare_parameter("grid_resolution", 0.1);
  this->declare_parameter("grid_width", 200);
  this->declare_parameter("grid_height", 200);
  
  grid_resolution_ = this->get_parameter("grid_resolution").as_double();
  grid_width_ = this->get_parameter("grid_width").as_int();
  grid_height_ = this->get_parameter("grid_height").as_int();
  
  // Initialize occupancy grid
  occupancy_grid_.header.frame_id = "map";
  occupancy_grid_.info.resolution = grid_resolution_;
  occupancy_grid_.info.width = grid_width_;
  occupancy_grid_.info.height = grid_height_;
  occupancy_grid_.info.origin.position.x = -grid_width_ * grid_resolution_ / 2.0;
  occupancy_grid_.info.origin.position.y = -grid_height_ * grid_resolution_ / 2.0;
  occupancy_grid_.info.origin.position.z = 0.0;
  occupancy_grid_.info.origin.orientation.w = 1.0;
  occupancy_grid_.data.resize(grid_width_ * grid_height_, -1);
  
  // Create subscribers
  detection_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
    "/detections", 10,
    [this](const vision_msgs::msg::Detection2DArray::SharedPtr msg) {
      // Empty callback, using synchronized callback instead
    });
  
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar/scan", 10,
    [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
      // Empty callback, using synchronized callback instead
    });
  
  // Create publishers
  pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/fused_pointcloud", 10);
  
  occupancy_grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/occupancy_grid", 10);
  
  human_marker_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
    "/human_markers", 10);
  
  // Setup synchronized subscribers
  detection_sync_sub_ = std::make_shared<message_filters::Subscriber<vision_msgs::msg::Detection2DArray>>(this, "/detections");
  lidar_sync_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(this, "/lidar/scan");
  
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(10), *detection_sync_sub_, *lidar_sync_sub_);
  sync_->registerCallback(std::bind(&FusionNode::synchronized_callback, this,
    std::placeholders::_1, std::placeholders::_2));
  
  RCLCPP_INFO(this->get_logger(), "Fusion node initialized");
}

void FusionNode::synchronized_callback(
  const vision_msgs::msg::Detection2DArray::SharedPtr detection_msg,
  const sensor_msgs::msg::LaserScan::SharedPtr lidar_msg)
{
  try {
    // Convert LaserScan to PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->header.frame_id = lidar_msg->header.frame_id;
    cloud->height = 1;
    cloud->width = 0;
    
    for (size_t i = 0; i < lidar_msg->ranges.size(); ++i) {
      if (std::isfinite(lidar_msg->ranges[i]) && lidar_msg->ranges[i] > lidar_msg->range_min && lidar_msg->ranges[i] < lidar_msg->range_max) {
        float angle = lidar_msg->angle_min + i * lidar_msg->angle_increment;
        pcl::PointXYZ point;
        point.x = lidar_msg->ranges[i] * cos(angle);
        point.y = lidar_msg->ranges[i] * sin(angle);
        point.z = 0.0;
        cloud->push_back(point);
        cloud->width++;
      }
    }
    
    // Update occupancy grid
    update_occupancy_grid(cloud);
    
    // Process detections and fuse with pointcloud
    std::vector<geometry_msgs::msg::Point> human_positions;
    
    for (const auto& detection : detection_msg->detections) {
      // Simple fusion: project detection center to pointcloud
      int u = detection.bbox.center.position.x;
      int v = detection.bbox.center.position.y;
      
      // Find corresponding point in pointcloud (simplified)
      if (!cloud->empty()) {
        // Simple projection (this should be replaced with proper projection matrix)
        size_t point_index = (u * cloud->size()) / 640;  // Assuming 640px width
        
        if (point_index < cloud->size()) {
          pcl::PointXYZ point = cloud->points[point_index];
          
          geometry_msgs::msg::Point human_point;
          human_point.x = point.x;
          human_point.y = point.y;
          human_point.z = point.z;
          
          human_positions.push_back(human_point);
        }
      }
    }
    
    // Publish human markers
    publish_human_markers(human_positions);
    
    // Publish pointcloud
    sensor_msgs::msg::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header = lidar_msg->header;
    pointcloud_pub_->publish(cloud_msg);
    
    // Publish occupancy grid
    occupancy_grid_.header.stamp = this->now();
    occupancy_grid_pub_->publish(occupancy_grid_);
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error in fusion callback: %s", e.what());
  }
}

void FusionNode::update_occupancy_grid(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
  // Reset grid to unknown
  std::fill(occupancy_grid_.data.begin(), occupancy_grid_.data.end(), -1);
  
  // Update grid with pointcloud data
  for (const auto& point : cloud->points) {
    int grid_x = static_cast<int>((point.x - occupancy_grid_.info.origin.position.x) / grid_resolution_);
    int grid_y = static_cast<int>((point.y - occupancy_grid_.info.origin.position.y) / grid_resolution_);
    
    if (grid_x >= 0 && grid_x < grid_width_ && grid_y >= 0 && grid_y < grid_height_) {
      int index = grid_y * grid_width_ + grid_x;
      occupancy_grid_.data[index] = 100;  // Occupied
    }
  }
}

void FusionNode::publish_human_markers(const std::vector<geometry_msgs::msg::Point>& human_positions)
{
  for (const auto& position : human_positions) {
    geometry_msgs::msg::PointStamped marker;
    marker.header.stamp = this->now();
    marker.header.frame_id = "map";
    marker.point = position;
    
    human_marker_pub_->publish(marker);
  }
}
#include "rescue_robot/perception_node.hpp"
#include <fstream>
#include <sstream>

PerceptionNode::PerceptionNode() : Node("perception_node")
{
  // Declare parameters
  this->declare_parameter("confidence_threshold", 0.5);
  this->declare_parameter("thermal_threshold", 35.0);
  
  // Get parameters
  confidence_threshold_ = this->get_parameter("confidence_threshold").as_double();
  thermal_threshold_ = this->get_parameter("thermal_threshold").as_double();
  
  // Initialize models
  initialize_yolo_model();
  
  // Create subscribers
  camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/camera/image_raw", 10,
    [this](const sensor_msgs::msg::Image::SharedPtr msg) {
      // Empty callback, using synchronized callback instead
    });
  
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
    "/lidar/scan", 10,
    [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
      // Empty callback, using synchronized callback instead
    });
  
  tof_sub_ = this->create_subscription<std_msgs::msg::Int32>(
    "/tof/distance", 10,
    [this](const std_msgs::msg::Int32::SharedPtr msg) {
      // Empty callback, using synchronized callback instead
    });
  
  thermal_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "/thermal/image", 10,
    [this](const sensor_msgs::msg::Image::SharedPtr msg) {
      // Empty callback, using synchronized callback instead
    });
  
  // Create publisher
  detection_pub_ = this->create_publisher<vision_msgs::msg::Detection2DArray>(
    "/detections", 10);
  
  // Setup synchronized subscribers
  camera_sync_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/camera/image_raw");
  lidar_sync_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::LaserScan>>(this, "/lidar/scan");
  tof_sync_sub_ = std::make_shared<message_filters::Subscriber<std_msgs::msg::Int32>>(this, "/tof/distance");
  thermal_sync_sub_ = std::make_shared<message_filters::Subscriber<sensor_msgs::msg::Image>>(this, "/thermal/image");
  
  sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
    SyncPolicy(10), *camera_sync_sub_, *lidar_sync_sub_, *tof_sync_sub_, *thermal_sync_sub_);
  sync_->registerCallback(std::bind(&PerceptionNode::synchronized_callback, this,
    std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
  
  RCLCPP_INFO(this->get_logger(), "Perception node initialized");
}

PerceptionNode::~PerceptionNode()
{
}

void PerceptionNode::initialize_yolo_model()
{
  // Load YOLO model
  try {
    std::string model_config = "config/yolov4-tiny.cfg";
    std::string model_weights = "config/yolov4-tiny.weights";
    
    yolo_net_ = cv::dnn::readNetFromDarknet(model_config, model_weights);
    yolo_net_.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);
    yolo_net_.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    
    // Load class names
    std::ifstream class_file("config/coco.names");
    if (class_file.is_open()) {
      std::string line;
      while (std::getline(class_file, line)) {
        class_names_.push_back(line);
      }
      class_file.close();
    } else {
      RCLCPP_ERROR(this->get_logger(), "Failed to open coco.names file");
    }
    
    RCLCPP_INFO(this->get_logger(), "YOLO model loaded successfully");
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load YOLO model: %s", e.what());
  }
}

void PerceptionNode::synchronized_callback(
  const sensor_msgs::msg::Image::SharedPtr camera_msg,
  const sensor_msgs::msg::LaserScan::SharedPtr lidar_msg,
  const std_msgs::msg::Int32::SharedPtr tof_msg,
  const sensor_msgs::msg::Image::SharedPtr thermal_msg)
{
  // Process all sensor data together
  try {
    // Convert ROS image to OpenCV format
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(camera_msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat camera_frame = cv_ptr->image;
    
    // Process with YOLO
    cv::Mat blob;
    cv::dnn::blobFromImage(camera_frame, blob, 1/255.0, cv::Size(416, 416), cv::Scalar(0,0,0), true, false);
    
    yolo_net_.setInput(blob);
    
    std::vector<cv::Mat> outputs;
    yolo_net_.forward(outputs, yolo_net_.getUnconnectedOutLayersNames());
    
    // Process outputs
    std::vector<int> class_ids;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;
    
    for (const auto& output : outputs) {
      auto* data = (float*)output.data;
      for (int i = 0; i < output.rows; ++i) {
        float confidence = data[4];
        if (confidence > confidence_threshold_) {
          cv::Mat scores = output.row(i).colRange(5, output.cols);
          cv::Point class_id_point;
          double max_class_score;
          cv::minMaxLoc(scores, 0, &max_class_score, 0, &class_id_point);
          
          if (max_class_score > confidence_threshold_ && class_id_point.x < class_names_.size()) {
            int center_x = (int)(data[0] * camera_frame.cols);
            int center_y = (int)(data[1] * camera_frame.rows);
            int width = (int)(data[2] * camera_frame.cols);
            int height = (int)(data[3] * camera_frame.rows);
            int left = center_x - width / 2;
            int top = center_y - height / 2;
            
            class_ids.push_back(class_id_point.x);
            confidences.push_back((float)max_class_score);
            boxes.push_back(cv::Rect(left, top, width, height));
          }
        }
        data += output.cols;
      }
    }
    
    // Apply non-maximum suppression
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, confidence_threshold_, 0.4, indices);
    
    // Create detection message
    vision_msgs::msg::Detection2DArray detections;
    detections.header = camera_msg->header;
    
    for (size_t i = 0; i < indices.size(); ++i) {
      int idx = indices[i];
      if (class_names_[class_ids[idx]] == "person") {
        vision_msgs::msg::Detection2D detection;
        detection.bbox.center.position.x = boxes[idx].x + boxes[idx].width / 2.0;
        detection.bbox.center.position.y = boxes[idx].y + boxes[idx].height / 2.0;
        detection.bbox.size_x = boxes[idx].width;
        detection.bbox.size_y = boxes[idx].height;
        detection.results.resize(1);
        detection.results[0].class_id = std::to_string(class_ids[idx]);
        detection.results[0].score = confidences[idx];
        
        detections.detections.push_back(detection);
        
        // Add to detection history for temporal filtering
        int track_id = class_ids[idx] * 1000 + i;  // Simple tracking ID
        detection_history_[track_id].push_back(
          std::make_pair(this->now(), confidences[idx]));
        
        // Keep only recent detections (last 5 seconds)
        auto it = detection_history_[track_id].begin();
        while (it != detection_history_[track_id].end()) {
          if ((this->now() - it->first).seconds() > 5.0) {
            it = detection_history_[track_id].erase(it);
          } else {
            ++it;
          }
        }
      }
    }
    
    // Publish detections
    detection_pub_->publish(detections);
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "Error in synchronized callback: %s", e.what());
  }
}
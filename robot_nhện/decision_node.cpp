#include "rescue_robot/decision_node.hpp"

DecisionNode::DecisionNode() : Node("decision_node"), current_state_(State::IDLE)
{
  // Create subscribers
  human_marker_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "/human_markers", 10,
    std::bind(&DecisionNode::human_marker_callback, this, std::placeholders::_1));
  
  occupancy_grid_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
    "/occupancy_grid", 10,
    std::bind(&DecisionNode::occupancy_grid_callback, this, std::placeholders::_1));
  
  // Create publishers
  esp32_command_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/esp32/commands", 10);
  
  lora_message_pub_ = this->create_publisher<std_msgs::msg::String>(
    "/lora/messages", 10);
  
  // State machine timer
  state_machine_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&DecisionNode::update_state_machine, this));
  
  RCLCPP_INFO(this->get_logger(), "Decision node initialized");
}

void DecisionNode::human_marker_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
  human_detections_.push_back(*msg);
  last_detection_time_ = this->now();
  
  RCLCPP_INFO(this->get_logger(), "Received human detection at (%.2f, %.2f, %.2f)",
              msg->point.x, msg->point.y, msg->point.z);
}

void DecisionNode::occupancy_grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  // Store the latest map
}

void DecisionNode::update_state_machine()
{
  switch (current_state_) {
    case State::IDLE:
      // Wait for start command or detection
      if (!human_detections_.empty()) {
        current_state_ = State::CANDIDATE;
        RCLCPP_INFO(this->get_logger(), "Transitioning to CANDIDATE state");
      } else {
        // Start sweeping if no detections
        send_command_to_esp32("SWEEP");
        current_state_ = State::SWEEP;
        RCLCPP_INFO(this->get_logger(), "Transitioning to SWEEP state");
      }
      break;
      
    case State::SWEEP:
      if (!human_detections_.empty()) {
        current_state_ = State::CANDIDATE;
        RCLCPP_INFO(this->get_logger(), "Transitioning to CANDIDATE state");
      }
      break;
      
    case State::CANDIDATE:
      if (human_detections_.size() >= 3) {
        // Multiple detections, likely a real person
        current_state_ = State::VERIFY;
        RCLCPP_INFO(this->get_logger(), "Transitioning to VERIFY state");
        
        // Command ESP32 to approach carefully
        geometry_msgs::msg::Point target = human_detections_.back().point;
        std::string command = "APPROACH " + 
                              std::to_string(target.x) + " " +
                              std::to_string(target.y) + " " +
                              std::to_string(target.z) + " SLOW";
        send_command_to_esp32(command);
      } else if ((this->now() - last_detection_time_).seconds() > 10.0) {
        // No recent detections, return to sweep
        human_detections_.clear();
        current_state_ = State::SWEEP;
        RCLCPP_INFO(this->get_logger(), "Transitioning to SWEEP state");
      }
      break;
      
    case State::VERIFY:
      if (human_detections_.size() >= 5) {
        // Confirmed detection
        current_state_ = State::FOUND;
        RCLCPP_INFO(this->get_logger(), "Transitioning to FOUND state");
        
        // Send LoRa message
        geometry_msgs::msg::Point target = human_detections_.back().point;
        std::string message = "HUMAN_FOUND " + 
                              std::to_string(target.x) + " " +
                              std::to_string(target.y) + " " +
                              std::to_string(target.z);
        send_lora_message(message);
        
        // Command ESP32 to mark position
        send_command_to_esp32("MARK " + 
                              std::to_string(target.x) + " " +
                              std::to_string(target.y) + " " +
                              std::to_string(target.z));
      } else if ((this->now() - last_detection_time_).seconds() > 5.0) {
        // Verification failed, return to candidate state
        current_state_ = State::CANDIDATE;
        RCLCPP_INFO(this->get_logger(), "Transitioning to CANDIDATE state");
      }
      break;
      
    case State::FOUND:
      // Human found, wait for further instructions
      break;
      
    case State::RESCUE_ASSIST:
      // Assist in rescue operations
      break;
      
    case State::ABORT:
      // Emergency stop
      send_command_to_esp32("STOP");
      break;
  }
}

void DecisionNode::send_command_to_esp32(const std::string& command)
{
  std_msgs::msg::String msg;
  msg.data = command;
  esp32_command_pub_->publish(msg);
  
  RCLCPP_INFO(this->get_logger(), "Sent command to ESP32: %s", command.c_str());
}

void DecisionNode::send_lora_message(const std::string& message)
{
  std_msgs::msg::String msg;
  msg.data = message;
  lora_message_pub_->publish(msg);
  
  RCLCPP_INFO(this->get_logger(), "Sent LoRa message: %s", message.c_str());
}
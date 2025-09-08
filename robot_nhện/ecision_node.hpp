#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "std_msgs/msg/string.hpp"

class DecisionNode : public rclcpp::Node
{
public:
  DecisionNode();
  ~DecisionNode();

  enum class State {
    IDLE,
    SWEEP,
    CANDIDATE,
    VERIFY,
    FOUND,
    RESCUE_ASSIST,
    ABORT
  };

private:
  void human_marker_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
  void occupancy_grid_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void update_state_machine();
  void send_command_to_esp32(const std::string& command);
  void send_lora_message(const std::string& message);
  
  // ROS2 subscribers and publishers
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr human_marker_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_sub_;
  
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr esp32_command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr lora_message_pub_;
  
  // State machine
  State current_state_;
  rclcpp::TimerBase::SharedPtr state_machine_timer_;
  
  // Detection history
  std::vector<geometry_msgs::msg::PointStamped> human_detections_;
  rclcpp::Time last_detection_time_;
};
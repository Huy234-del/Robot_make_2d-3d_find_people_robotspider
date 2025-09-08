#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class CommsNode : public rclcpp::Node
{
public:
  CommsNode();
  ~CommsNode();

private:
  void esp32_command_callback(const std_msgs::msg::String::SharedPtr msg);
  void lora_message_callback(const std_msgs::msg::String::SharedPtr msg);
  void send_serial_command(const std::string& command);
  void send_lora_packet(const std::string& message);
  
  // ROS2 subscribers and publishers
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr esp32_command_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr lora_message_sub_;
};

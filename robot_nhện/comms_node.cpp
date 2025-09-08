#include "rescue_robot/comms_node.hpp"

CommsNode::CommsNode() : Node("comms_node")
{
  // Create subscribers
  esp32_command_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/esp32/commands", 10,
    std::bind(&CommsNode::esp32_command_callback, this, std::placeholders::_1));
  
  lora_message_sub_ = this->create_subscription<std_msgs::msg::String>(
    "/lora/messages", 10,
    std::bind(&CommsNode::lora_message_callback, this, std::placeholders::_1));
  
  RCLCPP_INFO(this->get_logger(), "Comms node initialized");
}

void CommsNode::esp32_command_callback(const std_msgs::msg::String::SharedPtr msg)
{
  // In a real implementation, this would send the command via serial to ESP32
  RCLCPP_INFO(this->get_logger(), "Would send to ESP32: %s", msg->data.c_str());
}

void CommsNode::lora_message_callback(const std_msgs::msg::String::SharedPtr msg)
{
  // In a real implementation, this would send the message via LoRa
  RCLCPP_INFO(this->get_logger(), "Would send via LoRa: %s", msg->data.c_str());
}
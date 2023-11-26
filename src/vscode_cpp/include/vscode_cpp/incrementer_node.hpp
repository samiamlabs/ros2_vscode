// Copyright 2023 Samuel Lindgren

#ifndef VS_CODE_CPP__INCREMENTER_NODE_HPP_
#define VS_CODE_CPP__INCREMENTER_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

class IncrementerNode : public rclcpp::Node {
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr number_sub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr incremented_number_pub_;

  int step_size_;

public:
  IncrementerNode();
  ~IncrementerNode();

  void set_step_size(int step_size);
};

#endif // VS_CODE_CPP__INCREMENTER_NODE_HPP_
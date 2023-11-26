// Copyright 2023 Samuel Lindgren

#include "incrementer_node.hpp"

IncrementerNode::IncrementerNode() : Node("incrementer") {
  step_size_ = 1; // Default step size

  incremented_number_pub_ =
      this->create_publisher<std_msgs::msg::Int32>("incremented_number", 10);

  auto number_callback = [this](std_msgs::msg::Int32::SharedPtr msg) {
    auto incremented_number_msg = std::make_unique<std_msgs::msg::Int32>();
    incremented_number_msg->data = msg->data + step_size_;
    incremented_number_pub_->publish(std::move(incremented_number_msg));
  };

  number_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "number", 10, number_callback);
}

IncrementerNode::~IncrementerNode() {}

void IncrementerNode::set_step_size(int step_size) { step_size_ = step_size; }
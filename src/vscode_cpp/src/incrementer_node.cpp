// Copyright 2023 Samuel Lindgren

#include "incrementer_node.hpp"

IncrementerNode::IncrementerNode() : Node("incrementer") {
  RCLCPP_INFO(this->get_logger(), "Incrementer node started");

  step_size_ = 1; // Default step size

  // Allow step size to be set from the launch file
  declare_parameter("step_size", step_size_);
  get_parameter("step_size", step_size_);

  incremented_number_pub_ =
      this->create_publisher<std_msgs::msg::Int32>("incremented_number", 10);

  auto number_callback = [this](std_msgs::msg::Int32::SharedPtr msg) {
    auto incremented_number_msg = std::make_unique<std_msgs::msg::Int32>();
    int number = msg->data;
    int incremented_number = number + step_size_;
    incremented_number_msg->data = incremented_number;
    incremented_number_pub_->publish(std::move(incremented_number_msg));
  };

  number_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      "number", 10, number_callback);
}

IncrementerNode::~IncrementerNode() {}

void IncrementerNode::set_step_size(int step_size) { step_size_ = step_size; }

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<IncrementerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
// Copyright 2023 Samuel Lindgren

#include "incrementer_node.hpp"

#include <gtest/gtest.h>

using namespace std::chrono_literals;

class TestIncrementerNode : public ::testing::Test {
  void SetUp() {
    rclcpp::init(0, nullptr);
    ASSERT_TRUE(rclcpp::ok());

    default_spin_timeout_ = 3s;

    SetUpTestNode();
    SetUpIncrementerNode();
  }

  void SetUpTestNode() {
    test_node_ = std::make_shared<rclcpp::Node>("test_incrementer_node");

    number_test_publisher_ =
        test_node_->create_publisher<std_msgs::msg::Int32>("number", 10);

    auto number_test_subscription_callback =
        [this](std_msgs::msg::Int32::SharedPtr msg) {
          std::cout << "Received incremented number: " << msg->data
                    << std::endl;
          incremented_number_msg_ = msg;
        };

    incremented_number_test_subscription_ =
        test_node_->create_subscription<std_msgs::msg::Int32>(
            "incremented_number", 10, number_test_subscription_callback);

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(test_node_);
  }

  void SetUpIncrementerNode() {
    incrementer_node_ = std::make_shared<IncrementerNode>();
    executor_->add_node(incrementer_node_);
  }

  void TearDown() { rclcpp::shutdown(); }

public:
  std::chrono::seconds default_spin_timeout_;

  rclcpp::Node::SharedPtr test_node_;
  std::shared_ptr<IncrementerNode> incrementer_node_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr number_test_publisher_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr
      incremented_number_test_subscription_;

  std_msgs::msg::Int32::SharedPtr incremented_number_msg_;
};

TEST_F(TestIncrementerNode, test_relays_number) {
  auto msg = std::make_unique<std_msgs::msg::Int32>();
  msg->data = 0;
  number_test_publisher_->publish(std::move(msg));
  std::cout << "Published number" << std::endl;

  auto spin_start_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - spin_start_time <
             default_spin_timeout_ &&
         incremented_number_msg_ == nullptr) {
    executor_->spin_once(1ms);
  }

  ASSERT_TRUE(incremented_number_msg_ != nullptr)
      << "No incremented message received";
}

TEST_F(TestIncrementerNode, test_increments_number) {
  auto msg = std::make_unique<std_msgs::msg::Int32>();
  msg->data = 0;
  number_test_publisher_->publish(std::move(msg));
  std::cout << "Published number" << std::endl;

  auto spin_start_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - spin_start_time <
             default_spin_timeout_ &&
         incremented_number_msg_ == nullptr) {
    executor_->spin_once(1ms);
  }

  ASSERT_TRUE(incremented_number_msg_ != nullptr)
      << "No incremented message received";

  ASSERT_EQ(incremented_number_msg_->data, 1);
}

TEST_F(TestIncrementerNode, test_variable_step_size) {
  auto msg = std::make_unique<std_msgs::msg::Int32>();
  msg->data = 0;
  number_test_publisher_->publish(std::move(msg));
  std::cout << "Published number" << std::endl;

  auto spin_start_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - spin_start_time <
             default_spin_timeout_ &&
         incremented_number_msg_ == nullptr) {
    executor_->spin_once(1ms);
  }

  ASSERT_TRUE(incremented_number_msg_ != nullptr)
      << "No incremented message received";

  ASSERT_EQ(incremented_number_msg_->data, 1);
}

TEST_F(TestIncrementerNode, test_supports_setting_step_size) {
  incrementer_node_->set_step_size(2);

  auto msg = std::make_unique<std_msgs::msg::Int32>();
  msg->data = 1;
  number_test_publisher_->publish(std::move(msg));
  std::cout << "Published number" << std::endl;

  auto spin_start_time = std::chrono::steady_clock::now();
  while (std::chrono::steady_clock::now() - spin_start_time <
             default_spin_timeout_ &&
         incremented_number_msg_ == nullptr) {
    executor_->spin_once(1ms);
  }

  ASSERT_TRUE(incremented_number_msg_ != nullptr)
      << "No incremented message received";

  ASSERT_EQ(incremented_number_msg_->data, 3);
}
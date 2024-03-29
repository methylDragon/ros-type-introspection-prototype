#include <functional>
#include <memory>

#include <iostream>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "dynamic_typesupport_examples_msgs/msg/example_msg.hpp"

using std::placeholders::_1;
using dynamic_typesupport_examples_msgs::msg::ExampleMsg;

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<ExampleMsg>(
      "dynamic_message_test_topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const ExampleMsg & msg) const
  {
    RCLCPP_INFO(
      this->get_logger(), "\n\n%s", dynamic_typesupport_examples_msgs::msg::to_yaml(msg).c_str());
  }
  rclcpp::Subscription<ExampleMsg>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}

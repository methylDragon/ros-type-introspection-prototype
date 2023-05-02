#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "dynamic_typesupport_examples_msgs/msg/example_msg.hpp"

using namespace std::chrono_literals;
using dynamic_typesupport_examples_msgs::msg::ExampleMsg;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<ExampleMsg>("dynamic_message_test_topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = ExampleMsg();
    message.string_field = std::to_string(count_++);
    message.nested_field.doubly_nested_field.doubly_nested_float32_field = count_;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.string_field.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<ExampleMsg>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}

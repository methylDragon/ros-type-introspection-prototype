// Copyright 2022 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "detail/create_description.hpp"

#include "rosidl_runtime_c/type_description_utils.h"

#include "rcl/dynamic_message_type_support.h"
#include "rclcpp/rclcpp.hpp"

using rclcpp::dynamic_typesupport::DynamicMessage;
using rclcpp::dynamic_typesupport::DynamicMessageTypeSupport;

int g_counter = 0;

/**
 * This example demonstrates that the DynamicMessage can...
 * - Be printed
 * - Report member count (helpful for iteration)
 * - Resolve member IDs
 * - Have its members set
 * - Have its members accessed
 * - Have nested types accessed
 * - Get cloned
 *
 * All methods that take in a member ID also can take in a string name.
 *
 * NOTE: The types passed into the template args must match what is on the wire.
 *
 *       It can fail silently otherwise (FastRTPS fails silently), and there isn't a nice way to
 *       check...
 */
void msg_callback(
  DynamicMessage::SharedPtr data,
  std::shared_ptr<const rosidl_runtime_c__type_description__TypeDescription> description)
{
  (void) description;

  std::cout << "\n[MESSAGE RECEIVED]" << std::endl;
  data->print();

  std::cout << "\n[TEST ACCESS] (" << data->get_item_count() << " TOP LEVEL MEMBERS)" << std::endl;

  data->set_value<std::string>("string_field", "LOOK MA I'M BEING SET!");
  {
    DynamicMessage::SharedPtr array_data = data->loan_value(1);
    array_data->set_value<bool>(
      g_counter++ % array_data->get_item_count(),
      !array_data->get_value<bool>(0));
  }  // array_data.reset();  (Loan must be returned before print or other access is allowed)

  {
    DynamicMessage::SharedPtr nested_data = data->loan_value("nested_field");
    DynamicMessage::SharedPtr doubly_nested_data = nested_data->loan_value("doubly_nested_field");
    doubly_nested_data->set_value<float>(0, 999);
  }

  data->print();
  // data->clone();  // You may clone data (and loaned datas, as appropriate!)
  std::cout << "\n\n--END--\n" << std::endl;
}


int main(int argc, char ** argv)
{
  rosidl_runtime_c__type_description__TypeDescription * example_msg_desc =
    dynamic_typesupport_examples_create_description();

  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("dynamic_sub_node");

  // Create dynamic type support
  //   - Has middleware specific behavior
  //   - Assumes ownership of passed in descriptions (will manage lifetime)!

  // Does not take ownership of description (copies)
  auto ts = DynamicMessageTypeSupport::make_shared(*example_msg_desc);
  rosidl_runtime_c__type_description__TypeDescription__destroy(example_msg_desc);
  ts->print_description();

  auto sub = std::make_shared<rclcpp::DynamicSubscription>(
    node->get_node_base_interface().get(), ts, "dynamic_message_test_topic", 10, &msg_callback,
    rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>(),
    true  // use_take_dynamic_message
  );

  node->get_node_topics_interface()->add_subscription(sub, nullptr);
  rclcpp::spin(node);

  // The DynamicMessageTypeSupport::SharedPtr finalizes whatever needs to be finalized on
  // destruction!

  return 0;
}

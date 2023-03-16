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

#include "rosidl_runtime_c/type_description/field__functions.h"
#include "rosidl_runtime_c/type_description/field__struct.h"
#include "rosidl_runtime_c/type_description/individual_type_description__functions.h"
#include "rosidl_runtime_c/type_description/individual_type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_description__functions.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description_utils.h"

#include "rcl/rcl_dynamic_typesupport_c/message_introspection.h"
#include "rclcpp/rclcpp.hpp"


void msg_callback(std::shared_ptr<rosidl_dynamic_typesupport_dynamic_data_t> data)
{
  std::cout << "\n[MESSAGE RECEIVED]" << std::endl;
  rosidl_dynamic_typesupport_dynamic_data_print(data.get());
}


int main(int argc, char ** argv)
{
  // CREATE TYPES ==================================================================================
  // Normally a user wouldn't need to do this, since they can get it from the GetTypeDescription
  // service, or the statically included type description struct from the code gen
  //
  // ... Yes, it is painfully verbose
  //
  // NOTE(methylDragon): I am not handling the return types for brevity
  //                     Also, all append operations copy

  // FastRTPS needs these double colons (ideally we'd use / and replace them)
  std::string example_msg_inner_inner_name =
    "dynamic_typesupport_examples_msgs::msg::ExampleMsgInnerInner";

  std::string example_msg_inner_name =
    "dynamic_typesupport_examples_msgs::msg::ExampleMsgInner";

  std::string example_msg_name =
    "dynamic_typesupport_examples_msgs::msg::ExampleMsg";

  rosidl_runtime_c__type_description__TypeDescription * example_msg_desc = NULL;
  rosidl_runtime_c__type_description__Field * string_field = NULL;
  rosidl_runtime_c__type_description__Field * bool_static_array_field = NULL;
  rosidl_runtime_c__type_description__Field * nested_field = NULL;

  rosidl_runtime_c__type_description__IndividualTypeDescription * example_msg_inner_desc =
    NULL;
  rosidl_runtime_c__type_description__Field * doubly_nested_field = NULL;

  rosidl_runtime_c__type_description__IndividualTypeDescription * example_msg_inner_inner_desc =
    NULL;
  rosidl_runtime_c__type_description__Field * doubly_nested_float32_field = NULL;

  /// ExampleMsg ===================================================================================
  //   - string string_field
  //   - bool[5] bool_static_array_field
  //   - dynamic_typesupport_examples_msgs/ExampleMsgInner nested_field

  rosidl_runtime_c_type_description_utils_create_type_description(
    example_msg_name.c_str(), example_msg_name.size(), &example_msg_desc);

  // string_field
  rosidl_runtime_c_type_description_utils_create_field(
    // Name, Name Length
    // Type ID
    // Capacity, String Capacity
    // Nested Type Name, Nested Type Name Length
    // Default Value, Default Value Length
    // Field
    "string_field", strlen("string_field"),
    ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_STRING,
    0, 0,
    "", 0,
    "", 0,
    &string_field);
  rosidl_runtime_c_type_description_utils_append_field(
    &example_msg_desc->type_description, string_field);

  // bool_static_array_field
  rosidl_runtime_c_type_description_utils_create_field(
    "bool_static_array_field", strlen("bool_static_array_field"),
    ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_BOOLEAN_ARRAY,
    5, 0,
    "", 0,
    "", 0,
    &bool_static_array_field);
  rosidl_runtime_c_type_description_utils_append_field(
    &example_msg_desc->type_description, bool_static_array_field);

  // nested_field
  rosidl_runtime_c_type_description_utils_create_field(
    "nested_field", strlen("nested_field"),
    ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_NESTED_TYPE,
    0, 0,
    example_msg_inner_name.c_str(), example_msg_inner_name.size(),
    "", 0,
    &nested_field);
  rosidl_runtime_c_type_description_utils_append_field(
    &example_msg_desc->type_description, nested_field);

  rosidl_runtime_c__type_description__Field__destroy(string_field);
  rosidl_runtime_c__type_description__Field__destroy(bool_static_array_field);
  rosidl_runtime_c__type_description__Field__destroy(nested_field);

  /// ExampleMsgInner ==============================================================================
  //   - dynamic_typesupport_examples_msgs/ExampleMsgInnerInner doubly_nested_field

  rosidl_runtime_c_type_description_utils_create_individual_type_description(
    example_msg_inner_name.c_str(),
    example_msg_inner_name.size(),
    &example_msg_inner_desc);

  // doubly_nested_field
  rosidl_runtime_c_type_description_utils_create_field(
    "doubly_nested_field", strlen("doubly_nested_field"),
    ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_NESTED_TYPE,
    0, 0,
    example_msg_inner_inner_name.c_str(), example_msg_inner_inner_name.size(),
    "", 0,
    &doubly_nested_field);
  rosidl_runtime_c_type_description_utils_append_field(
    example_msg_inner_desc, doubly_nested_field);

  rosidl_runtime_c_type_description_utils_append_referenced_individual_type_description(
    example_msg_desc, example_msg_inner_desc, false);  // Don't coerce to valid
  rosidl_runtime_c__type_description__IndividualTypeDescription__destroy(
    example_msg_inner_desc);
  rosidl_runtime_c__type_description__Field__destroy(doubly_nested_field);

  /// ExampleMsgInnerInner =========================================================================
  //   - float32 doubly_nested_float32_field

  rosidl_runtime_c_type_description_utils_create_individual_type_description(
    example_msg_inner_inner_name.c_str(),
    example_msg_inner_inner_name.size(),
    &example_msg_inner_inner_desc);

  // doubly_nested_float32_field
  rosidl_runtime_c_type_description_utils_create_field(
    "doubly_nested_float32_field", strlen("doubly_nested_float32_field"),
    ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_FLOAT32,
    0, 0,
    "", 0,
    "", 0,
    &doubly_nested_float32_field);
  rosidl_runtime_c_type_description_utils_append_field(
    example_msg_inner_inner_desc, doubly_nested_float32_field);

  rosidl_runtime_c_type_description_utils_append_referenced_individual_type_description(
    example_msg_desc, example_msg_inner_inner_desc, false);  // Don't coerce to valid
  rosidl_runtime_c__type_description__IndividualTypeDescription__destroy(
    example_msg_inner_inner_desc);
  rosidl_runtime_c__type_description__Field__destroy(doubly_nested_float32_field);

  rosidl_runtime_c_type_description_utils_coerce_to_valid_type_description_in_place(
    example_msg_desc);

  rosidl_runtime_c_type_description_utils_print_type_description(example_msg_desc);


  // ROS ===========================================================================================
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("dynamic_sub_node");

  // Create dynamic type support
  //   - Has middleware specific behavior
  //   - Copies description, does not pass ownership!
  rosidl_message_type_support_t * ts =
    rcl_get_dynamic_message_typesupport_handle(nullptr, example_msg_desc);
  rosidl_runtime_c__type_description__TypeDescription__destroy(example_msg_desc);

  auto sub = std::make_shared<rclcpp::DynamicSubscription>(
    node->get_node_base_interface().get(),
    *ts,
    "dynamic_message_test_topic", 10,
    &msg_callback,
    rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>()
  );
  node->get_node_topics_interface()->add_subscription(sub, nullptr);
  rclcpp::spin(node);

  // The user has ownership of ts and must finalize!!
  auto ret = rcl_dynamic_message_typesupport_handle_fini(ts);
  (void) ret;

  return 0;
}

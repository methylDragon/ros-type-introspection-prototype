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

#include "rcl/rcl_typesupport_runtime_type_introspection_c/message_introspection.h"
#include "rclcpp/rclcpp.hpp"


void msg_callback(
  std::shared_ptr<serialization_support_t> ser, std::shared_ptr<ser_dynamic_data_t> data)
{
  std::cout << "\n[MESSAGE RECEIVED]" << std::endl;
  ser_print_dynamic_data(ser.get(), data.get());
}


int main(int argc, char ** argv)
{
  // LOAD DESCRIPTION ==============================================================================
  char * msg_path =
    g_strjoin("/", g_path_get_dirname(__FILE__), "..", "msg", "example_pub_msg.yaml", NULL);

  type_description_t * desc = create_type_description_from_yaml_file(msg_path);
  print_type_description(desc);

  // Has middleware specific behavior
  rosidl_message_type_support_t * ts =
    rcl_get_runtime_type_message_typesupport_handle(nullptr, desc);

  // ROS ===========================================================================================
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("runtime_type_sub_node");

  auto sub = std::make_shared<rclcpp::RuntimeTypeSubscription>(
    node->get_node_base_interface().get(),
    *ts,
    "runtime_type_test_topic", 10,
    &msg_callback,
    rclcpp::SubscriptionOptionsWithAllocator<std::allocator<void>>()
  );
  node->get_node_topics_interface()->add_subscription(sub, nullptr);
  rclcpp::spin(node);

  // The user has ownership of ts and must finalize!!
  auto ret = rcl_runtime_type_message_typesupport_handle_fini(ts);
  (void) ret;

  return 0;
}

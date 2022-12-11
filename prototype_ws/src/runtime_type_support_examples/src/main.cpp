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

int main()
{
  char * msg_path =
    g_strjoin("/", g_path_get_dirname(__FILE__), "..", "msg", "example_pub_msg.yaml", NULL);

    std::cout << msg_path << std::endl;
  type_description_t * desc = create_type_description_from_yaml_file(msg_path);
  print_type_description(desc);

  rosidl_message_type_support_t * ts = rcl_get_runtime_type_message_typesupport_handle(desc);

  // The user has ownership of ts and must finalize!!
  rcl_runtime_type_message_typesupport_handle_fini(ts);
  return 0;
}

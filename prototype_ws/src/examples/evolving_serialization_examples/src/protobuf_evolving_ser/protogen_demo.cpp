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

#include <glib.h>
#include <cassert>
#include <iostream>

#include <evolving_protobuf_c/protogen.h>
#include <rosidl_dynamic_typesupport/yaml_parser.h>
#include <rosidl_dynamic_typesupport/description.h>

int main()
{
  const char * main_dir = g_path_get_dirname(__FILE__);
  char * path = g_strjoin("/", main_dir, "..", "..", "msg", "nested.yaml", NULL);

  GNode * root = parse_yaml_file(path);
  type_description_t * description = create_type_description_from_yaml_file(path);

  // Three ways to get a proto file
  char * proto_str_a = generate_proto_from_yaml_file(path);
  char * proto_str_b = generate_proto_from_yaml_tree(root);
  char * proto_str_c = generate_proto_from_description(description);

  // Compare lengths to approximate equality. We can't compare for equality because the referenced
  // descriptions are ordered randomly
  assert(strlen(proto_str_a) == strlen(proto_str_b));
  assert(strlen(proto_str_b) == strlen(proto_str_c));
  assert(strlen(proto_str_c) == strlen(proto_str_a));

  std::cout << proto_str_c << std::endl;

  return 0;
}

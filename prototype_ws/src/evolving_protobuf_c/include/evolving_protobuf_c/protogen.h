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

/// Utilities for generating .proto files

#ifndef EVOLVING_PROTOBUF_C__PROTOGEN_H_
#define EVOLVING_PROTOBUF_C__PROTOGEN_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <glib.h>
#include <serialization_support_lib/description.h>


// FROM YAML TREE ==================================================================================
void
append_individual_type_description_from_yaml(GNode * description_node, char ** proto_str);

/// Generate a .proto string dynamically from a yaml tree.
/// Note: The referenced descriptions are ordered randomly
char *
generate_proto_from_yaml_tree(GNode * root);

/// Generate a .proto string dynamically from a yaml path.
/// Note: The referenced descriptions are ordered randomly
char *
generate_proto_from_yaml_file(const char * path);

// FROM DESCRIPTION STRUCT =========================================================================
void
append_individual_type_description(individual_type_description_t * description, char ** proto_str);

/// Generate a .proto string dynamically from a type_description_t.
/// Note: The referenced descriptions are ordered randomly
char *
generate_proto_from_description(type_description_t * description);


#ifdef __cplusplus
}
#endif

#endif  // EVOLVING_PROTOBUF_C__PROTOGEN_H_

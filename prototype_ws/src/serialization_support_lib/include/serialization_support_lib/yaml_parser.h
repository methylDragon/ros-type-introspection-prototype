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

///

#ifndef SERIALIZATION_SUPPORT_LIB__YAML_PARSER_H_
#define SERIALIZATION_SUPPORT_LIB__YAML_PARSER_H_

#include <stdbool.h>
#include <glib.h>
#include <yaml.h>

#ifdef __cplusplus
extern "C" {
#endif

void process_layer(yaml_parser_t * parser, GNode * data, bool seq);

/// Parse a yaml file into a GLib n-ary tree. Ownership of allocated pointer belongs to caller.
GNode * parse_yaml_file(char * path);

#ifdef __cplusplus
}
#endif

#endif  // SERIALIZATION_SUPPORT_LIB__YAML_PARSER_H_

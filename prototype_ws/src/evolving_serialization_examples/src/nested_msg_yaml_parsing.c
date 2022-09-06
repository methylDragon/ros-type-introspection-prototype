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


/* Description:
 *
 * Demonstration snippet for parsing a nested type description yaml into a
 * traversible tree, and how to traverse the tree by string references, with
 * indexing!
 *
 */

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <glib.h>

#include <evolving_serialization_lib/yaml_parser.h>
#include <evolving_serialization_lib/tree_traverse.h>

#include <rcl/types.h>

char * DESCRIPTION_FILE = "nested.yaml";

int main()
{
  // Read and parse description ================================================
  const char * main_dir = g_path_get_dirname(__FILE__);
  char * flat_yaml_path = g_strjoin("/", main_dir, "..", "msg", DESCRIPTION_FILE, NULL);
  printf("\nINPUT DESCRIPTION: %s\n\n", flat_yaml_path);

  GNode * cfg = g_node_new(flat_yaml_path);
  yaml_parser_t parser;

  FILE * source = fopen(flat_yaml_path, "rb");
  yaml_parser_initialize(&parser);
  yaml_parser_set_input_file(&parser, source);
  process_layer(&parser, cfg, false);  // Recursive parsing
  yaml_parser_delete(&parser);
  fclose(source);

  g_free(flat_yaml_path);

  // Traverse description tree =================================================
  // (g_node_traverse() )
  GNode * out = NULL;

  // // Get element of array field by string reference
  printf("\n=== Get fields of nested type description ===\n");
  assert(get_gnode_by_str_ref("type_description.fields[0].fields", cfg, &out) == RCL_RET_OK);
  g_node_traverse(out, G_PRE_ORDER, G_TRAVERSE_ALL, -1, gnode_repr_fn, NULL);

  printf("\n=== Field name of first element of fields of nested type description ===\n");
  assert(get_gnode_by_str_ref("type_description.fields[0].fields[0].field_name", cfg, &out) == RCL_RET_OK);
  g_node_traverse(out, G_PRE_ORDER, G_TRAVERSE_ALL, -1, gnode_repr_fn, NULL);

  // Cleanup
  g_node_destroy(cfg);

  return 0;
}
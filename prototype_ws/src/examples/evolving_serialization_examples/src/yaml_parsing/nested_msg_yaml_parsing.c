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

#include <rosidl_dynamic_typesupport/yaml_parser.h>
#include <rosidl_dynamic_typesupport/tree_traverse.h>
#include <rosidl_dynamic_typesupport/description.h>

char * DESCRIPTION_FILE = "nested.yaml";


int main()
{
  // Read and parse description ====================================================================
  const char * main_dir = g_path_get_dirname(__FILE__);
  char * nested_yaml_path = g_strjoin("/", main_dir, "..", "..", "msg", DESCRIPTION_FILE, NULL);
  printf("\nINPUT DESCRIPTION: %s\n\n", nested_yaml_path);

  GNode * cfg = parse_yaml_file(nested_yaml_path);
  g_free(nested_yaml_path);

  // Traverse description tree =====================================================================
  // (Using g_node_traverse())
  GNode * out = NULL;

  // Get fields of main type description
  assert(get_gnode_by_str_ref("type_description.fields[0].fields", cfg, &out) == true);

  // Print
  // g_node_traverse(out, G_PRE_ORDER, G_TRAVERSE_ALL, -1, gnode_print_fn, NULL);


  // Get field name of first elements of fields of a nested type description
  // The redirection happens automatically!
  assert(
    get_gnode_by_str_ref("type_description.fields[0].fields[0].field_name", cfg, &out) == true
  );

  // Print
  // g_node_traverse(out, G_PRE_ORDER, G_TRAVERSE_ALL, -1, gnode_print_fn, NULL);


  // Construct type_description struct =============================================================
  GNode * type_description = get_child_by_key(cfg, "type_description");

  printf(
    "\n\nCONSTRUCTING STRUCT FOR TYPE DESCRIPTION: %s\n\n",
    (char *)get_child_value_by_key(type_description, "type_name"));

  // Full description contains the main description and referenced descriptions
  type_description_t * full_description_struct = get_zero_initialized_type_description();

  populate_type_description(full_description_struct, cfg);
  print_type_description(full_description_struct);


  // Cleanup =======================================================================================
  type_description_fini(full_description_struct);
  g_node_traverse(cfg, G_PRE_ORDER, G_TRAVERSE_ALL, -1, gnode_free_node_data_fn, NULL);
  g_node_destroy(cfg);

  return 0;
}

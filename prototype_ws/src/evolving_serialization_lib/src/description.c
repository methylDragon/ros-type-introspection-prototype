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

#include <rcl/types.h>

#include <evolving_serialization_lib/description.h>
#include <evolving_serialization_lib/tree_traverse.h>
#include <evolving_serialization_lib/yaml_parser.h>


// =================================================================================================
// Structs
// =================================================================================================
// TYPE DESCRIPTION FIELD ==========================================================================
type_description_field_t *
get_zero_initialized_type_description_field(void)
{
  type_description_field_t * out = malloc(sizeof(type_description_field_t));
  if (out == NULL) {
    printf("Could not allocate type_description_field!\n");
    return NULL;
  }

  out->field_name = NULL;
  out->field_type = 0;

  out->field_array_size = 0;
  out->nested_type_name = NULL;

  return out;
}

rcl_ret_t
type_description_field_fini(type_description_field_t * type_description_field)
{
  free(type_description_field->field_name);
  free(type_description_field->nested_type_name);

  free(type_description_field);
  return RCL_RET_OK;
}


// INDIVIDUAL TYPE DESCRIPTION =====================================================================
individual_type_description_t *
get_zero_initialized_individual_type_description(void)
{
  individual_type_description_t * out = malloc(sizeof(individual_type_description_t));
  if (out == NULL) {
    printf("Could not allocate individual_type_description!\n");
    return NULL;
  }

  out->type_name = NULL;
  out->type_version_hash = NULL;

  out->fields = NULL;
  out->field_count = 0;

  return out;
}

rcl_ret_t
individual_type_description_fini(individual_type_description_t * individual_type_description)
{
  free(individual_type_description->type_name);
  free(individual_type_description->type_version_hash);

  for (size_t i = 0; i < individual_type_description->field_count; i++) {
    if (type_description_field_fini(individual_type_description->fields[i]) != RCL_RET_OK) {
      printf("Could not finalize individual_type_description!\n");
      return RCL_RET_ERROR;
    }
  }
  free(individual_type_description->fields);

  free(individual_type_description);
  return RCL_RET_OK;
}


// TYPE DESCRIPTION ================================================================================
void
_free_data(gpointer data)
{
  free(data);
}

void
_free_individual_type_description(gpointer data)
{
  if (individual_type_description_fini((individual_type_description_t *) data) != RCL_RET_OK) {
    printf("Could not finalize individual_type_description!\n");
  }
}

type_description_t *
get_zero_initialized_type_description(void)
{
  type_description_t * out = malloc(sizeof(type_description_t));
  if (out == NULL) {
    printf("Could not allocate type_description!\n");
    return NULL;
  }
  out->type_description = get_zero_initialized_individual_type_description();
  out->referenced_type_descriptions = g_hash_table_new_full(
    g_str_hash, g_str_equal, _free_data, _free_individual_type_description
  );

  return out;
}

rcl_ret_t
type_description_fini(type_description_t * type_description)
{
  g_hash_table_destroy(type_description->referenced_type_descriptions);

  if (!individual_type_description_fini(type_description->type_description)) {
    return RCL_RET_ERROR;
  }

  free(type_description);
  return RCL_RET_OK;
}


// =================================================================================================
// Construction
// =================================================================================================
individual_type_description_t *
populate_individual_type_description(
  individual_type_description_t * individual_description_struct,
  GNode * description_node)
{
  individual_description_struct->type_name =
    strdup(get_child_value_by_key(description_node, "type_name"));

  GNode * _fields = get_child_by_key(description_node, "fields");
  GNode * _field = g_node_nth_child(_fields, 0);

  individual_description_struct->field_count = g_node_n_children(_fields);
  individual_description_struct->fields =
    malloc(sizeof(type_description_field_t) * individual_description_struct->field_count);

  size_t __field_counter = 0;

  while (_field != NULL) {
    // Ideally we'd do some try-catch here to finalize the field if we failed
    type_description_field_t * _field_struct = get_zero_initialized_type_description_field();

    _field_struct->field_name = strdup(get_child_value_by_key(_field, "field_name"));
    _field_struct->field_type = atoi(get_child_value_by_key(_field, "field_type"));
    _field_struct->field_array_size = atoi(get_child_value_by_key(_field, "field_array_size"));

    // Treat "~" as null
    char * _nested_type_name =
      nullify_string(get_child_value_by_key(_field, "nested_type_name"), "~");
    if (_nested_type_name != NULL) {
      _field_struct->nested_type_name = strdup(_nested_type_name);
    }  // NULL by default from zero initialization

    individual_description_struct->fields[__field_counter++] = _field_struct;
    _field = _field->next;
  }

  return individual_description_struct;
}


type_description_t *
populate_type_description(type_description_t * description_struct, GNode * full_description_node)
{
  GNode * type_description_node = get_child_by_key(full_description_node, "type_description");
  populate_individual_type_description(description_struct->type_description, type_description_node);

  // Referenced descriptions
  GHashTable * ref_description_map = description_struct->referenced_type_descriptions;

  GNode * referenced_type_descriptions =
    get_child_by_key(full_description_node, "referenced_type_descriptions");
  GNode * ref_description = g_node_nth_child(referenced_type_descriptions, 0);

  if (g_node_n_children(ref_description) == 0) {
    return description_struct;
  }

  while (ref_description != NULL) {
    individual_type_description_t * individual_ref_decription_struct =
      get_zero_initialized_individual_type_description();

    populate_individual_type_description(individual_ref_decription_struct, ref_description);
    g_hash_table_insert(
      ref_description_map,
      get_child_value_by_key(ref_description, "type_name"),
      individual_ref_decription_struct
    );

    ref_description = ref_description->next;
  }

  return description_struct;
}


type_description_t *
create_type_description_from_yaml(const char * path)
{
  char * path_ = strdup(path);

  // Parse yaml
  GNode * cfg = g_node_new(path_);
  yaml_parser_t parser;
  FILE * source = fopen(path_, "rb");

  yaml_parser_initialize(&parser);
  yaml_parser_set_input_file(&parser, source);
  process_layer(&parser, cfg, false);  // Recursive parsing

  yaml_parser_delete(&parser);
  fclose(source);
  g_free(path_);

  // Create type description struct
  type_description_t * description_struct = get_zero_initialized_type_description();
  populate_type_description(description_struct, cfg);

  // Cleanup parsed tree (This is ok since the population function copies any relevant data)
  g_node_traverse(cfg, G_PRE_ORDER, G_TRAVERSE_ALL, -1, gnode_free_node_data_fn, NULL);
  g_node_destroy(cfg);

  return description_struct;
}

// =================================================================================================
// Printing
// =================================================================================================
void
print_type_description_field(type_description_field_t * input)
{
  printf(
    "[FIELD]\n"
    "  field_name: %s\n"
    "  field_type: %d\n"
    "  field_array_size: %ld\n"
    "  nested_type_name: %s\n",
    input->field_name, input->field_type, input->field_array_size, input->nested_type_name
  );
}


void
print_individual_type_description(individual_type_description_t * input)
{
  printf(
    "\n[INDIVIDUAL DESCRIPTION]\n"
    "  type_name: %s\n"
    "  type_version_hash: %s\n"
    "  field_count: %ld\n",
    input->type_name, input->type_version_hash, input->field_count
  );

  for (size_t i = 0; i < input->field_count; i++) {
    print_type_description_field(input->fields[i]);
  }
}


static void
_for_each_fn(gpointer key, gpointer value, gpointer user_data)
{
  (void)key;
  (void)user_data;
  print_individual_type_description((individual_type_description_t *) value);
}


void
print_type_description(type_description_t * input)
{
  printf("\n\n---\n\n");

  printf("= [PRINTING TYPE DESCRIPTION] =\n");
  printf(
    "  referenced_type_descriptions_count: %d\n",
    g_hash_table_size(input->referenced_type_descriptions)
  );

  printf("\n== [MAIN DESCRIPTION] ==\n");
  print_individual_type_description(input->type_description);

  printf("\n== [REFERENCED DESCRIPTIONS] ==\n");
  g_hash_table_foreach(input->referenced_type_descriptions, _for_each_fn, NULL);

  printf("\n---\n\n");
}

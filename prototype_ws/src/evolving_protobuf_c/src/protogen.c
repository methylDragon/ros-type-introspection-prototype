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

#define _GNU_SOURCE

#include <stdio.h>
#include "evolving_protobuf_c/protogen.h"

#include <evolving_serialization_lib/tree_traverse.h>
#include <evolving_serialization_lib/types.h>
#include <evolving_serialization_lib/yaml_parser.h>


// UTILS ===========================================================================================
#define APPEND_STR(str, literal) \
  if (_append_str(str, literal, strlen(literal)) < 0) { \
    printf("[WARNING] STRING APPEND FAILED\n"); \
  }

static int _append_str(char ** str, const char * buf, int size)
{
  char * nstr;
  if (*str == NULL) {
    nstr = (char *)malloc(size + 1);
    if (nstr == NULL) {return -1;}  // Alloc failed
    memcpy(nstr, buf, size);
    nstr[size] = '\0';
  } else {
    if (asprintf(&nstr, "%s%.*s", *str, size, buf) == -1) {return -1;}  // Alloc failed
    free(*str);
  }
  *str = nstr;
  return 0;
}


void _append_basic_field(int field_type, char ** proto_str)
{
  switch (field_type % SEQ_T_DELIMITER) {
    case BOOL_T_IDX:
      APPEND_STR(proto_str, "bool ");
      break;

    case FLOAT_32_T_IDX:
      APPEND_STR(proto_str, "float ");
      break;

    case FLOAT_64_T_IDX:
      APPEND_STR(proto_str, "double ");
      break;

    case INT_8_T_IDX:
    case INT_16_T_IDX:
    case INT_32_T_IDX:
      // This is the smallest available protobuf encoding for these types
      APPEND_STR(proto_str, "int32 ");
      break;

    case BYTE_T_IDX:
    case CHAR_T_IDX:
    case UINT_8_T_IDX:
    case UINT_16_T_IDX:
    case UINT_32_T_IDX:
      // This is the smallest available protobuf encoding for these types
      APPEND_STR(proto_str, "uint32 ");
      break;

    case INT_64_T_IDX:
      APPEND_STR(proto_str, "int64 ");
      break;

    case UINT_64_T_IDX:
      APPEND_STR(proto_str, "uint64 ");
      break;

    case STRING_T_IDX:
    case WSTRING_T_IDX:
    case BOUNDED_STRING_T_IDX:
    case BOUNDED_WSTRING_T_IDX:
      // bytes, since we can't guarantee that the strings are UTF-8 to use string
      // Also, protobuf has no concept of array bounds
      APPEND_STR(proto_str, "bytes ");
      break;

    default:
      printf("Field passed to protogen is not valid!");
  }
}


// FROM YAML TREE ==================================================================================
void _append_field_from_yaml(GNode * field_node, char ** proto_str)
{
  // Remember that this is just the on-the-wire definition
  //
  // We're limited by the types that protobuf supports!
  // https://developers.google.com/protocol-buffers/docs/proto#scalar

  int field_type = atoi((char *)get_child_value_by_key(field_node, "field_type"));
  assert(field_type != 0);

  APPEND_STR(proto_str, "  ");

  // Handle arrays/sequences
  if (field_type / SEQ_T_DELIMITER > 0) {
    APPEND_STR(proto_str, "repeated ");
  }

  if ((field_type % SEQ_T_DELIMITER) == NESTED_T_IDX) {
    char * _nested_type_name =
      nullify_string((char *)get_child_value_by_key(field_node, "nested_type_name"), "~");
    if (_nested_type_name == NULL) {
      printf("[ERROR] No nested type name found");
      return;
    }
    APPEND_STR(proto_str, _nested_type_name);
    APPEND_STR(proto_str, " ");
  } else {
    _append_basic_field(field_type % SEQ_T_DELIMITER, proto_str);
  }

  char * _field_name = (char *) get_child_value_by_key(field_node, "field_name");
  APPEND_STR(proto_str, _field_name);
}


void append_individual_type_description_from_yaml(GNode * description_node, char ** proto_str)
{
  APPEND_STR(proto_str, "message ");
  APPEND_STR(proto_str, (char *) get_child_value_by_key(description_node, "type_name"));
  APPEND_STR(proto_str, " {\n");

  GNode * _fields = get_child_by_key(description_node, "fields");
  GNode * _field = g_node_nth_child(_fields, 0);

  size_t _field_counter = 1;  // Protobuf fields are 1-indexed

  while (_field != NULL) {
    // Ideally we'd do some try-catch here to finalize the field if we failed
    _append_field_from_yaml(_field, proto_str);
    APPEND_STR(proto_str, "=");

    int length = snprintf(NULL, 0, "%ld", _field_counter);
    char * str = (char *) malloc(length + 1);
    snprintf(str, length + 1, "%ld", _field_counter);
    APPEND_STR(proto_str, str);
    free(str);

    APPEND_STR(proto_str, ";\n");

    _field = _field->next;
    _field_counter++;
  }

  APPEND_STR(proto_str, "}\n\n");
}


char * generate_proto_from_yaml_tree(GNode * root)
{
  char * proto_str = NULL;
  APPEND_STR(&proto_str, "syntax = \"proto3\";\n\n");

  GNode * main_description = get_child_by_key(root, "type_description");
  append_individual_type_description_from_yaml(main_description, &proto_str);

  GNode * ref_descriptions =
    get_child_by_key(root, "referenced_type_descriptions");
  GNode * ref_description = g_node_nth_child(ref_descriptions, 0);

  while (ref_description != NULL) {
    append_individual_type_description_from_yaml(ref_description, &proto_str);
    ref_description = ref_description->next;
  }

  if (proto_str == NULL) {
    printf("[ERROR] Could not allocate proto string");
    return NULL;
  }

  return proto_str;
}


char * generate_proto_from_yaml_file(const char * path)
{
  char * path_ = strdup(path);
  GNode * root = parse_yaml_file(path_);

  g_free(path_);

  char * out = generate_proto_from_yaml_tree(root);

  // Cleanup parsed tree (This is ok since the population function copies any relevant data)
  g_node_traverse(root, G_PRE_ORDER, G_TRAVERSE_ALL, -1, gnode_free_node_data_fn, NULL);
  g_node_destroy(root);

  return out;
}


// FROM DESCRIPTION STRUCT =========================================================================
void _append_field(type_description_field_t * field, char ** proto_str)
{
  // Remember that this is just the on-the-wire definition
  //
  // We're limited by the types that protobuf supports!
  // https://developers.google.com/protocol-buffers/docs/proto#scalar

  uint8_t field_type = field->field_type;
  assert(field_type != 0);

  APPEND_STR(proto_str, "  ");

  // Handle arrays/sequences
  if (field_type / SEQ_T_DELIMITER > 0) {
    APPEND_STR(proto_str, "repeated ");
  }

  // Handle base type
  switch (field_type % SEQ_T_DELIMITER) {
    case BOOL_T_IDX:
      APPEND_STR(proto_str, "bool ");
      break;

    case FLOAT_32_T_IDX:
      APPEND_STR(proto_str, "float ");
      break;

    case FLOAT_64_T_IDX:
      APPEND_STR(proto_str, "double ");
      break;

    case INT_8_T_IDX:
    case INT_16_T_IDX:
    case INT_32_T_IDX:
      // This is the smallest available protobuf encoding for these types
      APPEND_STR(proto_str, "int32 ");
      break;

    case BYTE_T_IDX:
    case CHAR_T_IDX:
    case UINT_8_T_IDX:
    case UINT_16_T_IDX:
    case UINT_32_T_IDX:
      // This is the smallest available protobuf encoding for these types
      APPEND_STR(proto_str, "uint32 ");
      break;

    case INT_64_T_IDX:
      APPEND_STR(proto_str, "int64 ");
      break;

    case UINT_64_T_IDX:
      APPEND_STR(proto_str, "uint64 ");
      break;

    case STRING_T_IDX:
    case WSTRING_T_IDX:
    case BOUNDED_STRING_T_IDX:
    case BOUNDED_WSTRING_T_IDX:
      // bytes, since we can't guarantee that the strings are UTF-8 to use string
      // Also, protobuf has no concept of array bounds
      APPEND_STR(proto_str, "bytes ");
      break;

    case NESTED_T_IDX:
      if (field->nested_type_name == NULL) {
        printf("[ERROR] No nested type name found");
        return;
      }
      APPEND_STR(proto_str, field->nested_type_name);
      APPEND_STR(proto_str, " ");
      break;
  }

  APPEND_STR(proto_str, field->field_name);
}


void append_individual_type_description(
  individual_type_description_t * description,
  char ** proto_str)
{
  APPEND_STR(proto_str, "message ");
  APPEND_STR(proto_str, description->type_name);
  APPEND_STR(proto_str, " {\n");

  type_description_field_t ** _fields = description->fields;

  for (size_t _field_num = 0; _field_num < description->field_count; _field_num++) {
    // Ideally we'd do some try-catch here to finalize the field if we failed
    _append_field(_fields[_field_num], proto_str);
    APPEND_STR(proto_str, "=");

    // Digit to char *
    int length = snprintf(NULL, 0, "%ld", _field_num + 1);  // proto fields are 1-indexed
    char * str = (char *) malloc(length + 1);
    snprintf(str, length + 1, "%ld", _field_num + 1);
    APPEND_STR(proto_str, str);
    free(str);

    APPEND_STR(proto_str, ";\n");
  }

  APPEND_STR(proto_str, "}\n\n");
}


// Redirection wrapper for g_hash_table_foreach
void _append_individual_type_description(void * key, void * description, void * proto_str)
{
  (void)key;
  append_individual_type_description(
    (individual_type_description_t *) description,
    (char **) proto_str);
}


char * generate_proto_from_description(type_description_t * description)
{
  char * proto_str = NULL;
  APPEND_STR(&proto_str, "syntax = \"proto3\";\n\n");

  append_individual_type_description(description->type_description, &proto_str);

  g_hash_table_foreach(
    description->referenced_type_descriptions,
    _append_individual_type_description,
    &proto_str);

  if (proto_str == NULL) {
    printf("[ERROR] Could not allocate proto string");
    return NULL;
  }

  return proto_str;
}

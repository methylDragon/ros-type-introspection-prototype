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

#include <impl/tree_traverse.h>


// =================================================================================================
// Utils
// =================================================================================================
/// Check if string is terminated by terminator
static bool
_check_terminator(const char * tok, const char * terminator)
{
  return strcmp(tok + strlen(tok) - strlen(terminator), terminator) == 0;
}


/// Get index from suffixed char pointers, like "5]", where "]" is the terminator
static int
_get_idx_from_idx_token(const char * tok, const char * terminator)
{
  int idx;
  int idx_len = strlen(tok) - strlen(terminator);

  char * idx_str = malloc(sizeof(char) * idx_len + 1);
  strncpy(idx_str, tok, idx_len);
  idx_str[idx_len] = '\0';

  idx = atoi(idx_str);
  free(idx_str);

  return idx;
}


/// Get field_type index from a field_type GNode
static int
_get_field_type(GNode * field_type_node)
{
  // Only works with field_type nodes!
  assert(strcmp(field_type_node->data, "field_type") == 0);
  return atoi(field_type_node->children[0].data);
}


GNode *
get_child_by_key(GNode * node, char * key)
{
  GNode * _child_node = g_node_str_child_find(node, G_LEVEL_ORDER, G_TRAVERSE_ALL, key);

  if (_child_node == NULL)
  {
    printf("Could not find child GNode: %s\n", key);
    return NULL;
  }

  return _child_node;
}


gpointer
get_child_value_by_key(GNode * node, char * key)
{
  GNode * _child_node = get_child_by_key(node, key);
  return _child_node->children[0].data;
}


gboolean gnode_repr_fn(GNode * node, gpointer data)
{
  (void)data;

  int i = g_node_depth(node) - 1;
  if (i == 0) { return FALSE; }
  if (i == 1) {printf("\n");}

  while (i-- > 0) {printf("  ");}

  if (!node->children || strcmp((char *)node->data, "-") == 0) {
    printf("%s\n", (char *)node->data);
  } else {
    printf("%s:\n", (char *)node->data);
  }
  return FALSE;
}


// =================================================================================================
// glib Tree Traversal Functions
// =================================================================================================
/// Per-node comparison function to pass to _g_node_str_find to extract the matching node
static gboolean
_g_node_data_strcmp(GNode * node, gpointer data)
{
  gpointer * d = data;
  if (strcmp(*d, node->data) != 0) {return FALSE;}
  *(++d) = node;
  return TRUE;
}


/// Find node in glib N-ary tree with node data matching a string
GNode *
_g_node_str_depth_find(GNode * root, GTraverseType order, GTraverseFlags flags, char * str, int depth)
{
  gpointer d[2];

  g_return_val_if_fail(root != NULL, NULL);
  g_return_val_if_fail(order <= G_LEVEL_ORDER, NULL);
  g_return_val_if_fail(flags <= G_TRAVERSE_MASK, NULL);

  d[0] = str;
  d[1] = NULL;

  g_node_traverse(root, order, flags, depth, _g_node_data_strcmp, d);

  return d[1];
}


GNode *
g_node_str_tree_find(GNode * root, GTraverseType order, GTraverseFlags flags, char * str)
{
  return _g_node_str_depth_find(root, order, flags, str, -1);
}


GNode *
g_node_str_child_find(GNode * root, GTraverseType order, GTraverseFlags flags, char * str)
{
  return _g_node_str_depth_find(root, order, flags, str, 2);
}


rcl_ret_t
get_gnode_by_str_ref(char * ref, GNode * root, GNode ** out_node)
{
  char * _r = strdup(ref);
  char * _tok = _r, * _end = _r;
  GNode * _traverse_node = root;
  GNode * _ref_table_node = g_node_str_tree_find(
    root, G_LEVEL_ORDER, G_TRAVERSE_ALL, "referenced_type_descriptions"
  );

  while ((_tok = strsep(&_end, ".["))) {
    // Ignore empty tokens
    if (strcmp(_tok, "") == 0) {
      continue;
    }

    // Handle index tokens, like "5]"
    if (_check_terminator(_tok, "]")) {
      if (!_traverse_node) {
        printf("Cannot index into children for a NULL node!\n");
        free(_r);
        *out_node = NULL;
        return RCL_RET_ERROR;
      }

      int _idx = _get_idx_from_idx_token(_tok, "]");
      int _n_children = g_node_n_children(_traverse_node);

      if (_n_children <= _idx || _n_children + _idx < 0) { // Enforce child bounds
        printf("Index [%d] out of bounds for node with [%d] children!\n", _idx, _n_children);
        free(_r);
        *out_node = NULL;
        return RCL_RET_ERROR;
      }

      _idx = _idx < 0 ? _n_children + _idx : _idx;  // Normalize index
      _traverse_node = g_node_nth_child(_traverse_node, _idx);
    }

    // Otherwise, handle name tokens
    else {
      GNode * _prev_traverse_node = _traverse_node;
      _traverse_node = g_node_str_child_find(_traverse_node, G_LEVEL_ORDER, G_TRAVERSE_ALL, _tok);

      // If unresolved, try searching the referenced type descriptions via a
      // short-circuit
      if (_traverse_node == NULL) {
        GNode * _field_type_node = g_node_str_child_find(
          _prev_traverse_node, G_LEVEL_ORDER, G_TRAVERSE_ALL, "field_type"
        );

        // Not redirectable
        if (_field_type_node == NULL) {
          printf("Token [%s] unresolvable explicitly!\n", _tok);
          free(_r);
          *out_node = NULL;
          return RCL_RET_ERROR;
        }

        // Redirectable, but no referenced types
        if (_ref_table_node == NULL)
        {
          printf("Field token %s unresolvable explicitly, and no referenced types found!\n", _tok);
          free(_r);
          *out_node = NULL;
          return RCL_RET_ERROR;
        }

        if (_get_field_type(_field_type_node) == NESTED_TYPE_IDX) {
          char * _nested_type_name = (char *)get_child_value_by_key(_prev_traverse_node,
                                                                     "nested_type_name");

          // No nested type name to redirect to
          if (_nested_type_name == NULL)
          {
            printf("Token [%s] unresolvable explicitly, and no nested_type_name found on attempted "
                   "redirection to referenced type.\n", _tok);
            free(_r);
            *out_node = NULL;
            return RCL_RET_ERROR;
          }

          printf("Redirecting to nested_type description: [%s]\n", _nested_type_name);
          GNode * _ref_node = g_node_str_tree_find(
            _ref_table_node, G_LEVEL_ORDER, G_TRAVERSE_ALL, _nested_type_name
          )->parent->parent;

          // No referenced type entry for the referenced type
          if (_ref_node == NULL)
          {
            printf("Token [%s] unresolvable explicitly. "
                   "Could not redirect to nested_type_description for [%s].",
                   _tok, _nested_type_name);
            free(_r);
            *out_node = NULL;
            return RCL_RET_ERROR;
          }

          _traverse_node = g_node_str_child_find(_ref_node, G_LEVEL_ORDER, G_TRAVERSE_ALL, _tok);
        }
      }
    }

  }  // end while

  free(_r);

  if (_traverse_node) {
    *out_node = _traverse_node;
    return RCL_RET_OK;
  }

  *out_node = NULL;
  return RCL_RET_ERROR;
}

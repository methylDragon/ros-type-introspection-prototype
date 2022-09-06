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

/// Utilities for traversing a GLib N-ary tree

#include <regex.h>

#include <glib.h>
#include <rcl/types.h>

#define NESTED_TYPE_IDX 1


// =================================================================================================
// Utils
// =================================================================================================
/// Index into a node's children via key-value semantics and get node
GNode *
get_child_by_key(GNode * node, char * key);

/// Index into a node's children via key-value semantics and get key
gpointer
get_child_value_by_key(GNode * node, char * key);

/// String representation printing function for GNodes
gboolean gnode_repr_fn(GNode * n, gpointer data);


// =================================================================================================
// glib Tree Traversal Functions
// =================================================================================================
/// Find node in glib N-ary tree with node data matching a string
GNode *
g_node_str_tree_find(GNode * root, GTraverseType order, GTraverseFlags flags, char * data);

/// Find immediate child node in glib N-ary tree with node data matching a string
GNode *
g_node_str_child_find(GNode * root, GTraverseType order, GTraverseFlags flags, char * data);

/// Get GNode from glib N-ary tree by string reference
/// Also redirects nested types to referenced_type_descriptions!
rcl_ret_t
get_gnode_by_str_ref(char * ref, GNode * root, GNode ** out_node);

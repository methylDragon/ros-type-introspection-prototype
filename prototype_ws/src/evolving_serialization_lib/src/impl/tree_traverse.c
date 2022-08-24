#include <impl/tree_traverse.h>


// =================================================================================================
// Internal glib Tree Traversal Functions
// =================================================================================================
/// Per-node comparison function to pass to _g_node_str_find to extract the matching node
static gboolean
_g_node_str_find_func(GNode * node, gpointer data)
{
  gpointer * d = data;
  if (strcmp(*d, node->data) != 0) {return FALSE;}
  *(++d) = node;
  return TRUE;
}


// Get index from suffixed char pointers, like "5]", where "]" is the terminator
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


// Check if string is terminated by terminator
static bool
_check_terminator(const char * tok, const char * terminator)
{
  return strcmp(tok + strlen(tok) - strlen(terminator), terminator) == 0;
}


// =================================================================================================
// glib Tree Traversal Functions
// =================================================================================================
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

  g_node_traverse(root, order, flags, depth, _g_node_str_find_func, d);

  return d[1];
}


// Find node in glib N-ary tree with node data matching a string
GNode *
g_node_str_tree_find(GNode * root, GTraverseType order, GTraverseFlags flags, char * str)
{
  return _g_node_str_depth_find(root, order, flags, str, -1);
}


// Find immediate child node in glib N-ary tree with node data matching a string
GNode *
g_node_str_child_find(GNode * root, GTraverseType order, GTraverseFlags flags, char * str)
{
  return _g_node_str_depth_find(root, order, flags, str, 2);
}


/// Get GNode from glib N-ary tree by string reference
rcl_ret_t
get_gnode_by_str_ref(char * ref, GNode * root, GNode ** out_node)
{
  char * r = strdup(ref);
  char * tok = r, * end = r;
  GNode * traverse_node = root;

  while ((tok = strsep(&end, ".["))) {
    // printf(
    //   "tok: %s | curr_node: %s | children: %d\n",
    //   tok, (char *)traverse_node->data, g_node_n_children(traverse_node));

    // Ignore empty tokens
    if (strcmp(tok, "") == 0) {
      continue;
    }

    // Handle index tokens, like "5]"
    if (_check_terminator(tok, "]")) {
      if (!traverse_node) {
        printf("Cannot index into children for a NULL node!");

        free(r);
        *out_node = NULL;
        return RCL_RET_ERROR;
      }

      int idx = _get_idx_from_idx_token(tok, "]");
      int n_children = g_node_n_children(traverse_node);

      if (n_children <= idx || n_children + idx < 0) { // Enforce child bounds
        printf("Index [%d] out of bounds for node with [%d] children!\n", idx, n_children);

        free(r);
        *out_node = NULL;
        return RCL_RET_ERROR;
      }

      idx = idx < 0 ? n_children + idx : idx;  // Normalize index
      traverse_node = g_node_nth_child(traverse_node, idx);

      // g_node_n_children

      // traverse_node =
    }
    // Handle name tokens
    else {
      traverse_node = g_node_str_child_find(traverse_node, G_LEVEL_ORDER, G_TRAVERSE_ALL, tok);
    }

    // if (!traverse_node) {
    //   free(r);
    //   *out_node = NULL;
    //   return RCL_RET_ERROR;
    // }
  }

  free(r);

  if (traverse_node) {
    *out_node = traverse_node;
    return RCL_RET_OK;
  }

  *out_node = NULL;
  return RCL_RET_ERROR;
}


// =================================================================================================
// Internal Regex Functions
// =================================================================================================

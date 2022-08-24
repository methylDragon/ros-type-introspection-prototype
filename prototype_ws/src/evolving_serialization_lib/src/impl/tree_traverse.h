#include <regex.h>

#include <glib.h>
#include <rcl/types.h>

GNode *
g_node_str_tree_find(GNode * root, GTraverseType order, GTraverseFlags flags, char * data);

GNode *
g_node_str_child_find(GNode * root, GTraverseType order, GTraverseFlags flags, char * data);

rcl_ret_t
get_gnode_by_str_ref(char * ref, GNode * root, GNode ** out_node);

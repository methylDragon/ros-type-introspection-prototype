#include <stdbool.h>
#include <yaml.h>
#include <glib.h>

void process_layer(yaml_parser_t * parser, GNode * data, bool seq);
gboolean gnode_repr(GNode * n, gpointer data);

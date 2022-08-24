// Modified from source: https://stackoverflow.com/a/621451
// Modified to support sequence elements and switch cases

#include <impl/yaml_parser.h>
#include <stdio.h>
#include <stdbool.h>
#include <assert.h>


enum storage_flags { VAR, VAL }; // "Store as" switch

void process_layer(yaml_parser_t * parser, GNode * data, bool seq)
{
  GNode * last_leaf = data;
  yaml_event_t event;
  int storage = VAR;   // mapping cannot start with VAL definition w/o VAR key

  while (1) {
    yaml_parser_parse(parser, &event);
    assert(event.type != YAML_NO_EVENT /* MESSAGE */ );  // Invalid yaml

    switch (event.type) {
      case YAML_SCALAR_EVENT:
        if (storage & VAL && seq) {
          // If this is just a bare value in a sequence, append it, populated
          data = g_node_append(last_leaf, g_node_new(g_strdup((gchar *) "-")));
          g_node_append(data, g_node_new(g_strdup((gchar *) event.data.scalar.value)));
          break;
        }

        if (storage & VAL) {
          // Add data
          g_node_append_data(last_leaf, g_strdup((gchar *) event.data.scalar.value));
        } else {
          // Point data to new node with data
          last_leaf = g_node_append(data, g_node_new(g_strdup((gchar *) event.data.scalar.value)));
        }
        storage ^= VAL;    // Flip VAR/VAL switch for the next event
        break;

      case YAML_SEQUENCE_START_EVENT:
        seq = true;
        break;

      case YAML_SEQUENCE_END_EVENT:
        seq = false;
        break;

      case YAML_MAPPING_START_EVENT:
        // Add new layer if sequence encountered
        if (seq) {
          process_layer(
            parser,
            g_node_append(last_leaf, g_node_new(g_strdup((gchar *) "-"))),
            false
          );
        } else {
          // Otherwise, append to last leaf
          process_layer(parser, last_leaf, seq);
        }
        storage ^= VAL;  // Flip VAR/VAL
        break;

      case YAML_MAPPING_END_EVENT:
      case YAML_STREAM_END_EVENT:
        yaml_event_delete(&event);
        return;

      default:  // Ignore the rest
        break;
    }
  }
}


gboolean gnode_repr(GNode * node, gpointer data)
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

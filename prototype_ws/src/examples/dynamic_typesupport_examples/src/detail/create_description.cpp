#include "detail/create_description.hpp"

#include <string>

#include "rosidl_dynamic_typesupport/types.h"

#include "rosidl_runtime_c/type_description/field__functions.h"
#include "rosidl_runtime_c/type_description/field__struct.h"
#include "rosidl_runtime_c/type_description/individual_type_description__functions.h"
#include "rosidl_runtime_c/type_description/individual_type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_description__functions.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description_utils.h"

rosidl_runtime_c__type_description__TypeDescription *
dynamic_typesupport_examples_create_description()
{
  // CREATE TYPES ==================================================================================
  // Normally a user wouldn't need to do this, since they can get it from the GetTypeDescription
  // service, or the statically included type description struct from the code gen
  //
  // ... Yes, it is painfully verbose
  //
  // NOTE(methylDragon): I am not handling the return types for brevity
  //                     Also, all append operations copy

  // NOTE(methylDragon):
  // FastRTPS needs these double colons
  // But it doesn't matter since the rosidl_dynamic_typesupport library will convert internally on
  // dynamic type construction!!
  //
  // What **is** a requirement though, is to always have the <package>/<interface type>/<typename>
  // structure
  //
  // I demonstrate that functionality by mixing the delimiters! :D
  std::string example_msg_inner_inner_name =
    "dynamic_typesupport_examples_msgs/msg/ExampleMsgInnerInner";

  std::string example_msg_inner_name =
    "dynamic_typesupport_examples_msgs::msg::ExampleMsgInner";

  std::string example_msg_name =
    "dynamic_typesupport_examples_msgs/msg/ExampleMsg";

  rosidl_runtime_c__type_description__TypeDescription * example_msg_desc = NULL;
  rosidl_runtime_c__type_description__Field * string_field = NULL;
  rosidl_runtime_c__type_description__Field * bool_static_array_field = NULL;
  rosidl_runtime_c__type_description__Field * nested_field = NULL;

  rosidl_runtime_c__type_description__IndividualTypeDescription * example_msg_inner_desc = NULL;
  rosidl_runtime_c__type_description__Field * doubly_nested_field = NULL;

  rosidl_runtime_c__type_description__IndividualTypeDescription * example_msg_inner_inner_desc =
    NULL;
  rosidl_runtime_c__type_description__Field * doubly_nested_float32_field = NULL;

  /// ExampleMsg ===================================================================================
  //   - string string_field
  //   - bool[5] bool_static_array_field
  //   - dynamic_typesupport_examples_msgs/ExampleMsgInner nested_field

  rosidl_runtime_c_type_description_utils_create_type_description(
    example_msg_name.c_str(), example_msg_name.size(), &example_msg_desc);

  // string_field
  rosidl_runtime_c_type_description_utils_create_field(
    // Name, Name Length
    // Type ID
    // Capacity, String Capacity
    // Nested Type Name, Nested Type Name Length
    // Default Value, Default Value Length
    // Field
    "string_field", strlen("string_field"),
    ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_STRING,
    0, 0,
    "", 0,
    "", 0,
    &string_field);
  rosidl_runtime_c_type_description_utils_append_field(
    &example_msg_desc->type_description, string_field);

  // bool_static_array_field
  rosidl_runtime_c_type_description_utils_create_field(
    "bool_static_array_field", strlen("bool_static_array_field"),
    ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_BOOLEAN_ARRAY,
    5, 0,
    "", 0,
    "", 0,
    &bool_static_array_field);
  rosidl_runtime_c_type_description_utils_append_field(
    &example_msg_desc->type_description, bool_static_array_field);

  // nested_field
  rosidl_runtime_c_type_description_utils_create_field(
    "nested_field", strlen("nested_field"),
    ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_NESTED_TYPE,
    0, 0,
    example_msg_inner_name.c_str(), example_msg_inner_name.size(),
    "", 0,
    &nested_field);
  rosidl_runtime_c_type_description_utils_append_field(
    &example_msg_desc->type_description, nested_field);

  rosidl_runtime_c__type_description__Field__destroy(string_field);
  rosidl_runtime_c__type_description__Field__destroy(bool_static_array_field);
  rosidl_runtime_c__type_description__Field__destroy(nested_field);

  /// ExampleMsgInner ==============================================================================
  //   - dynamic_typesupport_examples_msgs/ExampleMsgInnerInner doubly_nested_field

  rosidl_runtime_c_type_description_utils_create_individual_type_description(
    example_msg_inner_name.c_str(),
    example_msg_inner_name.size(),
    &example_msg_inner_desc);

  // doubly_nested_field
  rosidl_runtime_c_type_description_utils_create_field(
    "doubly_nested_field", strlen("doubly_nested_field"),
    ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_NESTED_TYPE,
    0, 0,
    example_msg_inner_inner_name.c_str(), example_msg_inner_inner_name.size(),
    "", 0,
    &doubly_nested_field);
  rosidl_runtime_c_type_description_utils_append_field(
    example_msg_inner_desc, doubly_nested_field);

  rosidl_runtime_c_type_description_utils_append_referenced_individual_type_description(
    example_msg_desc, example_msg_inner_desc, false);  // Don't coerce to valid
  rosidl_runtime_c__type_description__IndividualTypeDescription__destroy(
    example_msg_inner_desc);
  rosidl_runtime_c__type_description__Field__destroy(doubly_nested_field);

  /// ExampleMsgInnerInner =========================================================================
  //   - float32 doubly_nested_float32_field

  rosidl_runtime_c_type_description_utils_create_individual_type_description(
    example_msg_inner_inner_name.c_str(),
    example_msg_inner_inner_name.size(),
    &example_msg_inner_inner_desc);

  // doubly_nested_float32_field
  rosidl_runtime_c_type_description_utils_create_field(
    "doubly_nested_float32_field", strlen("doubly_nested_float32_field"),
    ROSIDL_DYNAMIC_TYPESUPPORT_FIELD_TYPE_FLOAT32,
    0, 0,
    "", 0,
    "", 0,
    &doubly_nested_float32_field);
  rosidl_runtime_c_type_description_utils_append_field(
    example_msg_inner_inner_desc, doubly_nested_float32_field);

  rosidl_runtime_c_type_description_utils_append_referenced_individual_type_description(
    example_msg_desc, example_msg_inner_inner_desc, false);  // Don't coerce to valid
  rosidl_runtime_c__type_description__IndividualTypeDescription__destroy(
    example_msg_inner_inner_desc);
  rosidl_runtime_c__type_description__Field__destroy(doubly_nested_float32_field);

  rosidl_runtime_c_type_description_utils_coerce_to_valid_type_description_in_place(
    example_msg_desc);

return example_msg_desc;
}

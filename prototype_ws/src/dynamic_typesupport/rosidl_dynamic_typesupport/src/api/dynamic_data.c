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

#include <stdlib.h>
#include <rosidl_dynamic_typesupport/api/dynamic_data.h>
#include <rosidl_dynamic_typesupport/types.h>


// =================================================================================================
// DYNAMIC DATA
// =================================================================================================

// DYNAMIC DATA UTILS ==============================================================================
void
rosidl_dynamic_typesupport_dynamic_data_clear_all_values(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data)
{
  (serialization_support->interface->clear_all_values)(serialization_support->impl, dynamic_data->impl);
}


void
rosidl_dynamic_typesupport_dynamic_data_clear_nonkey_values(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data)
{
  (serialization_support->interface->clear_nonkey_values)(serialization_support->impl, dynamic_data->impl);
}


void
rosidl_dynamic_typesupport_dynamic_data_clear_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, rosidl_dynamic_typesupport_member_id_t id)
{
  (serialization_support->interface->clear_value)(serialization_support->impl, dynamic_data->impl, id);
}


bool
rosidl_dynamic_typesupport_dynamic_data_equals(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, const rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, const rosidl_dynamic_typesupport_dynamic_data_t * other)
{
  return (serialization_support->interface->dynamic_data_equals)(serialization_support->impl, dynamic_data->impl, other->impl);
}


uint32_t
rosidl_dynamic_typesupport_dynamic_data_get_item_count(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, const rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data)
{
  return (serialization_support->interface->get_dynamic_data_item_count)(serialization_support->impl, dynamic_data->impl);
}


rosidl_dynamic_typesupport_member_id_t
rosidl_dynamic_typesupport_dynamic_data_get_member_id_by_name(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, const rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, const char * name)
{
  return (serialization_support->interface->get_dynamic_data_member_id_by_name)(serialization_support->impl, dynamic_data->impl, name);
}


rosidl_dynamic_typesupport_member_id_t
rosidl_dynamic_typesupport_dynamic_data_get_member_id_at_index(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, const rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, uint32_t index)
{
  return (serialization_support->interface->get_dynamic_data_member_id_at_index)(serialization_support->impl, dynamic_data->impl, index);
}


rosidl_dynamic_typesupport_member_id_t
rosidl_dynamic_typesupport_dynamic_data_get_array_index(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, uint32_t index)
{
  return (serialization_support->interface->get_array_index)(serialization_support->impl, dynamic_data->impl, index);
}


rosidl_dynamic_typesupport_dynamic_data_t *
rosidl_dynamic_typesupport_dynamic_data_loan_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, rosidl_dynamic_typesupport_member_id_t id)
{
  rosidl_dynamic_typesupport_dynamic_data_t * out = calloc(1, sizeof(rosidl_dynamic_typesupport_dynamic_data_t));
  out->serialization_support = dynamic_data->serialization_support;
  out->impl = (serialization_support->interface->loan_value)(serialization_support->impl, dynamic_data->impl, id);
  return out;
}


void
rosidl_dynamic_typesupport_dynamic_data_return_loaned_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * outer_dynamic_data, rosidl_dynamic_typesupport_dynamic_data_t * inner_dynamic_data)
{
  (serialization_support->interface->return_loaned_value)(serialization_support->impl, outer_dynamic_data->impl, inner_dynamic_data->impl);
  free(inner_dynamic_data);
}


void
rosidl_dynamic_typesupport_dynamic_data_print(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data)
{
  (serialization_support->interface->dynamic_data_print)(serialization_support->impl, dynamic_data->impl);
}


// DYNAMIC DATA CONSTRUCTION =======================================================================
rosidl_dynamic_typesupport_dynamic_data_t *
rosidl_dynamic_typesupport_dynamic_data_init_from_dynamic_type_builder(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_type_builder_t * dynamic_type_builder)
{
  rosidl_dynamic_typesupport_dynamic_data_t * out = calloc(1, sizeof(rosidl_dynamic_typesupport_dynamic_data_t));
  out->serialization_support = dynamic_type_builder->serialization_support;
  out->impl = (serialization_support->interface->dynamic_data_init_from_builder)(serialization_support->impl, dynamic_type_builder->impl);
  return out;
}


rosidl_dynamic_typesupport_dynamic_data_t *
rosidl_dynamic_typesupport_dynamic_data_init_from_dynamic_type(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_type_t * dynamic_type)
{
  rosidl_dynamic_typesupport_dynamic_data_t * out = calloc(1, sizeof(rosidl_dynamic_typesupport_dynamic_data_t));
  out->serialization_support = dynamic_type->serialization_support;
  out->impl = (serialization_support->interface->dynamic_data_init_from_dynamic_type)(serialization_support->impl, dynamic_type->impl);
  return out;
}


rosidl_dynamic_typesupport_dynamic_data_t *
rosidl_dynamic_typesupport_dynamic_data_clone(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, const rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data)
{
  rosidl_dynamic_typesupport_dynamic_data_t * out = calloc(1, sizeof(rosidl_dynamic_typesupport_dynamic_data_t));
  out->serialization_support = dynamic_data->serialization_support;
  out->impl = (serialization_support->interface->dynamic_data_clone)(serialization_support->impl, dynamic_data->impl);
  return out;
}


void
rosidl_dynamic_typesupport_dynamic_data_fini(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data)
{
  (serialization_support->interface->dynamic_data_fini)(serialization_support->impl, dynamic_data->impl);
  free(dynamic_data->impl);
}


// DYNAMIC DATA PRIMITIVE MEMBER GETTERS ===========================================================
void
rosidl_dynamic_typesupport_dynamic_data_get_bool_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, const rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, bool * value, rosidl_dynamic_typesupport_member_id_t id)
{
  (serialization_support->interface->get_bool_value)(serialization_support->impl, dynamic_data->impl, value, id);
}


void
rosidl_dynamic_typesupport_dynamic_data_get_byte_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, const rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, uint8_t * value, rosidl_dynamic_typesupport_member_id_t id)
{
  (serialization_support->interface->get_byte_value)(serialization_support->impl, dynamic_data->impl, value, id);
}


void
rosidl_dynamic_typesupport_dynamic_data_get_char_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, const rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, char * value, rosidl_dynamic_typesupport_member_id_t id)
{
  (serialization_support->interface->get_char_value)(serialization_support->impl, dynamic_data->impl, value, id);
}


void
rosidl_dynamic_typesupport_dynamic_data_get_float32_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, const rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, float * value, rosidl_dynamic_typesupport_member_id_t id)
{
  (serialization_support->interface->get_float32_value)(serialization_support->impl, dynamic_data->impl, value, id);
}


void
rosidl_dynamic_typesupport_dynamic_data_get_float64_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, const rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, double * value, rosidl_dynamic_typesupport_member_id_t id)
{
  (serialization_support->interface->get_float64_value)(serialization_support->impl, dynamic_data->impl, value, id);
}


void
rosidl_dynamic_typesupport_dynamic_data_get_int8_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, const rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, int8_t * value, rosidl_dynamic_typesupport_member_id_t id)
{
  (serialization_support->interface->get_int8_value)(serialization_support->impl, dynamic_data->impl, value, id);
}


void
rosidl_dynamic_typesupport_dynamic_data_get_uint8_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, const rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, uint8_t * value, rosidl_dynamic_typesupport_member_id_t id)
{
  (serialization_support->interface->get_uint8_value)(serialization_support->impl, dynamic_data->impl, value, id);
}


void
rosidl_dynamic_typesupport_dynamic_data_get_int16_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, const rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, int16_t * value, rosidl_dynamic_typesupport_member_id_t id)
{
  (serialization_support->interface->get_int16_value)(serialization_support->impl, dynamic_data->impl, value, id);
}


void
rosidl_dynamic_typesupport_dynamic_data_get_uint16_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, const rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, uint16_t * value, rosidl_dynamic_typesupport_member_id_t id)
{
  (serialization_support->interface->get_uint16_value)(serialization_support->impl, dynamic_data->impl, value, id);
}


void
rosidl_dynamic_typesupport_dynamic_data_get_int32_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, const rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, int32_t * value, rosidl_dynamic_typesupport_member_id_t id)
{
  (serialization_support->interface->get_int32_value)(serialization_support->impl, dynamic_data->impl, value, id);
}


void
rosidl_dynamic_typesupport_dynamic_data_get_uint32_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, const rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, uint32_t * value, rosidl_dynamic_typesupport_member_id_t id)
{
  (serialization_support->interface->get_uint32_value)(serialization_support->impl, dynamic_data->impl, value, id);
}


void
rosidl_dynamic_typesupport_dynamic_data_get_int64_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, const rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, int64_t * value, rosidl_dynamic_typesupport_member_id_t id)
{
  (serialization_support->interface->get_int64_value)(serialization_support->impl, dynamic_data->impl, value, id);
}


void
rosidl_dynamic_typesupport_dynamic_data_get_uint64_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, const rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, uint64_t * value, rosidl_dynamic_typesupport_member_id_t id)
{
  (serialization_support->interface->get_uint64_value)(serialization_support->impl, dynamic_data->impl, value, id);
}


void
rosidl_dynamic_typesupport_dynamic_data_get_string_value(
  rosidl_dynamic_typesupport_serialization_support_t * serialization_support, const rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, const char ** value, rosidl_dynamic_typesupport_member_id_t id)
{
  (serialization_support->interface->get_string_value)(serialization_support->impl, dynamic_data->impl, value, id);
}


void
rosidl_dynamic_typesupport_dynamic_data_get_wstring_value(
  rosidl_dynamic_typesupport_serialization_support_t * serialization_support, const rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, const wchar_t ** value, rosidl_dynamic_typesupport_member_id_t id)
{
  (serialization_support->interface->get_wstring_value)(serialization_support->impl, dynamic_data->impl, value, id);
}


// DYNAMIC DATA PRIMITIVE MEMBER SETTERS ===========================================================
void
rosidl_dynamic_typesupport_dynamic_data_set_bool_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, bool value, rosidl_dynamic_typesupport_member_id_t id)
{
  (serialization_support->interface->set_bool_value)(serialization_support->impl, dynamic_data->impl, value, id);
}


void
rosidl_dynamic_typesupport_dynamic_data_set_byte_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, uint8_t value, rosidl_dynamic_typesupport_member_id_t id)
{
  (serialization_support->interface->set_byte_value)(serialization_support->impl, dynamic_data->impl, value, id);
}


void
rosidl_dynamic_typesupport_dynamic_data_set_char_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, char value, rosidl_dynamic_typesupport_member_id_t id)
{
  (serialization_support->interface->set_char_value)(serialization_support->impl, dynamic_data->impl, value, id);
}


void
rosidl_dynamic_typesupport_dynamic_data_set_float32_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, float value, rosidl_dynamic_typesupport_member_id_t id)
{
  (serialization_support->interface->set_float32_value)(serialization_support->impl, dynamic_data->impl, value, id);
}


void
rosidl_dynamic_typesupport_dynamic_data_set_float64_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, double value, rosidl_dynamic_typesupport_member_id_t id)
{
  (serialization_support->interface->set_float64_value)(serialization_support->impl, dynamic_data->impl, value, id);
}


void
rosidl_dynamic_typesupport_dynamic_data_set_int8_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, int8_t value, rosidl_dynamic_typesupport_member_id_t id)
{
  (serialization_support->interface->set_int8_value)(serialization_support->impl, dynamic_data->impl, value, id);
}


void
rosidl_dynamic_typesupport_dynamic_data_set_uint8_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, uint8_t value, rosidl_dynamic_typesupport_member_id_t id)
{
  (serialization_support->interface->set_uint8_value)(serialization_support->impl, dynamic_data->impl, value, id);
}


void
rosidl_dynamic_typesupport_dynamic_data_set_int16_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, int16_t value, rosidl_dynamic_typesupport_member_id_t id)
{
  (serialization_support->interface->set_int16_value)(serialization_support->impl, dynamic_data->impl, value, id);
}


void
rosidl_dynamic_typesupport_dynamic_data_set_uint16_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, uint16_t value, rosidl_dynamic_typesupport_member_id_t id)
{
  (serialization_support->interface->set_uint16_value)(serialization_support->impl, dynamic_data->impl, value, id);
}


void
rosidl_dynamic_typesupport_dynamic_data_set_int32_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, int32_t value, rosidl_dynamic_typesupport_member_id_t id)
{
  (serialization_support->interface->set_int32_value)(serialization_support->impl, dynamic_data->impl, value, id);
}


void
rosidl_dynamic_typesupport_dynamic_data_set_uint32_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, uint32_t value, rosidl_dynamic_typesupport_member_id_t id)
{
  (serialization_support->interface->set_uint32_value)(serialization_support->impl, dynamic_data->impl, value, id);
}


void
rosidl_dynamic_typesupport_dynamic_data_set_int64_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, int64_t value, rosidl_dynamic_typesupport_member_id_t id)
{
  (serialization_support->interface->set_int64_value)(serialization_support->impl, dynamic_data->impl, value, id);
}


void
rosidl_dynamic_typesupport_dynamic_data_set_uint64_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, uint64_t value, rosidl_dynamic_typesupport_member_id_t id)
{
  (serialization_support->interface->set_uint64_value)(serialization_support->impl, dynamic_data->impl, value, id);
}


void
rosidl_dynamic_typesupport_dynamic_data_set_string_value(
  rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, const char * value, rosidl_dynamic_typesupport_member_id_t id)
{
  (serialization_support->interface->set_string_value)(serialization_support->impl, dynamic_data->impl, value, id);
}


void
rosidl_dynamic_typesupport_dynamic_data_set_wstring_value(
  rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, const wchar_t * value, rosidl_dynamic_typesupport_member_id_t id)
{
  (serialization_support->interface->set_wstring_value)(serialization_support->impl, dynamic_data->impl, value, id);
}


// DYNAMIC DATA SEQUENCES ==========================================================================
void
rosidl_dynamic_typesupport_dynamic_data_clear_sequence_data(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data)
{
  (serialization_support->interface->clear_sequence_data)(serialization_support->impl, dynamic_data->impl);
}


void
rosidl_dynamic_typesupport_dynamic_data_remove_sequence_data(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, rosidl_dynamic_typesupport_member_id_t id)
{
  (serialization_support->interface->remove_sequence_data)(serialization_support->impl, dynamic_data->impl, id);
}


void
rosidl_dynamic_typesupport_dynamic_data_insert_sequence_data(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (serialization_support->interface->insert_sequence_data)(serialization_support->impl, dynamic_data->impl, out_id);
}


void
rosidl_dynamic_typesupport_dynamic_data_insert_bool_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, bool value, rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (serialization_support->interface->insert_bool_value)(serialization_support->impl, dynamic_data->impl, value, out_id);
}


void
rosidl_dynamic_typesupport_dynamic_data_insert_byte_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, uint8_t value, rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (serialization_support->interface->insert_byte_value)(serialization_support->impl, dynamic_data->impl, value, out_id);
}


void
rosidl_dynamic_typesupport_dynamic_data_insert_char_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, char value, rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (serialization_support->interface->insert_char_value)(serialization_support->impl, dynamic_data->impl, value, out_id);
}


void
rosidl_dynamic_typesupport_dynamic_data_insert_float32_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, float value, rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (serialization_support->interface->insert_float32_value)(serialization_support->impl, dynamic_data->impl, value, out_id);
}


void
rosidl_dynamic_typesupport_dynamic_data_insert_float64_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, double value, rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (serialization_support->interface->insert_float64_value)(serialization_support->impl, dynamic_data->impl, value, out_id);
}


void
rosidl_dynamic_typesupport_dynamic_data_insert_int16_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, int16_t value, rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (serialization_support->interface->insert_int16_value)(serialization_support->impl, dynamic_data->impl, value, out_id);
}


void
rosidl_dynamic_typesupport_dynamic_data_insert_uint16_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, uint16_t value, rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (serialization_support->interface->insert_uint16_value)(serialization_support->impl, dynamic_data->impl, value, out_id);
}


void
rosidl_dynamic_typesupport_dynamic_data_insert_int32_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, int32_t value, rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (serialization_support->interface->insert_int32_value)(serialization_support->impl, dynamic_data->impl, value, out_id);
}


void
rosidl_dynamic_typesupport_dynamic_data_insert_uint32_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, uint32_t value, rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (serialization_support->interface->insert_uint32_value)(serialization_support->impl, dynamic_data->impl, value, out_id);
}


void
rosidl_dynamic_typesupport_dynamic_data_insert_int64_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, int64_t value, rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (serialization_support->interface->insert_int64_value)(serialization_support->impl, dynamic_data->impl, value, out_id);
}


void
rosidl_dynamic_typesupport_dynamic_data_insert_uint64_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, uint64_t value, rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (serialization_support->interface->insert_uint64_value)(serialization_support->impl, dynamic_data->impl, value, out_id);
}


void
rosidl_dynamic_typesupport_dynamic_data_insert_string_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, const char * value, rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (serialization_support->interface->insert_string_value)(serialization_support->impl, dynamic_data->impl, value, out_id);
}


void
rosidl_dynamic_typesupport_dynamic_data_insert_wstring_value(
  rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, const wchar_t * value, rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (serialization_support->interface->insert_wstring_value)(serialization_support->impl, dynamic_data->impl, value, out_id);
}


// DYNAMIC DATA NESTED =============================================================================
void
rosidl_dynamic_typesupport_dynamic_data_get_complex_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, const rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, rosidl_dynamic_typesupport_dynamic_data_t ** value, rosidl_dynamic_typesupport_member_id_t id)
{
  (*value)->serialization_support = dynamic_data->serialization_support;
  (serialization_support->interface->get_complex_value)(serialization_support->impl, dynamic_data->impl, &(*value)->impl, id);
}


void
rosidl_dynamic_typesupport_dynamic_data_set_complex_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, rosidl_dynamic_typesupport_dynamic_data_t * value, rosidl_dynamic_typesupport_member_id_t id)
{
  (serialization_support->interface->set_complex_value)(serialization_support->impl, dynamic_data->impl, value->impl, id);
}


void
rosidl_dynamic_typesupport_dynamic_data_insert_const_complex_value(
  rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, const rosidl_dynamic_typesupport_dynamic_data_t * value, rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (serialization_support->interface->insert_const_complex_value)(serialization_support->impl, dynamic_data->impl, value->impl, out_id);
}


void
rosidl_dynamic_typesupport_dynamic_data_insert_complex_value(rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, rosidl_dynamic_typesupport_dynamic_data_t * value, rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (serialization_support->interface->insert_complex_value)(serialization_support->impl, dynamic_data->impl, value->impl, out_id);
}


void
rosidl_dynamic_typesupport_dynamic_data_insert_complex_value_ptr(
  rosidl_dynamic_typesupport_serialization_support_t * serialization_support, rosidl_dynamic_typesupport_dynamic_data_t * dynamic_data, rosidl_dynamic_typesupport_dynamic_data_t * value, rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (serialization_support->interface->insert_complex_value_ptr)(serialization_support->impl, dynamic_data->impl, value->impl, out_id);
}
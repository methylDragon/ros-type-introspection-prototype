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
#include <serialization_support_lib/api/dynamic_data.h>


// =================================================================================================
// DYNAMIC DATA
// =================================================================================================

// DYNAMIC DATA UTILS ==============================================================================
void
ser_clear_all_values(serialization_support_t * ser, ser_dynamic_data_t * data)
{
  (ser->interface->clear_all_values)(ser->impl, data);
}


void
ser_clear_nonkey_values(serialization_support_t * ser, ser_dynamic_data_t * data)
{
  (ser->interface->clear_nonkey_values)(ser->impl, data);
}


void
ser_clear_value(serialization_support_t * ser, ser_dynamic_data_t * data, MemberId id)
{
  (ser->interface->clear_value)(ser->impl, data, id);
}


bool
ser_data_equals(serialization_support_t * ser, const ser_dynamic_data_t * data, const ser_dynamic_data_t * other)
{
  return (ser->interface->data_equals)(ser->impl, data, other);
}


uint32_t
ser_get_data_item_count(serialization_support_t * ser, const ser_dynamic_data_t * data)
{
  return (ser->interface->get_data_item_count)(ser->impl, data);
}


MemberId
ser_get_data_member_id_by_name(serialization_support_t * ser, const ser_dynamic_data_t * data, const char * name)
{
  return (ser->interface->get_data_member_id_by_name)(ser->impl, data, name);
}


MemberId
ser_get_data_member_id_at_index(serialization_support_t * ser, const ser_dynamic_data_t * data, uint32_t index)
{
  return (ser->interface->get_data_member_id_at_index)(ser->impl, data, index);
}


MemberId
ser_get_array_index(serialization_support_t * ser, ser_dynamic_data_t * data, uint32_t index)
{
  return (ser->interface->get_array_index)(ser->impl, data, index);
}


ser_dynamic_data_t *
ser_loan_value(serialization_support_t * ser, ser_dynamic_data_t * data, MemberId id)
{
  return (ser->interface->loan_value)(ser->impl, data, id);
}


void
ser_return_loaned_value(serialization_support_t * ser, ser_dynamic_data_t * data, ser_dynamic_data_t * inner_data)
{
  (ser->interface->return_loaned_value)(ser->impl, data, inner_data);
  free(inner_data);
}


void
ser_print_dynamic_data(serialization_support_t * ser, ser_dynamic_data_t * data)
{
  (ser->interface->print_dynamic_data)(ser->impl, data);
}


// DYNAMIC DATA CONSTRUCTION =======================================================================
ser_dynamic_data_t *
ser_data_init_from_builder(serialization_support_t * ser, ser_type_builder_t * builder)
{
  return (ser->interface->data_init_from_builder)(ser->impl, builder);
}


ser_dynamic_data_t *
ser_data_init_from_type(serialization_support_t * ser, ser_dynamic_type_t * type)
{
  return (ser->interface->data_init_from_type)(ser->impl, type);
}


ser_dynamic_data_t *
ser_data_clone(serialization_support_t * ser, const ser_dynamic_data_t * data)
{
  return (ser->interface->data_clone)(ser->impl, data);
}


void
ser_data_fini(serialization_support_t * ser, ser_dynamic_data_t * data)
{
  (ser->interface->data_fini)(ser->impl, data);
  free(data);
}


// DYNAMIC DATA PRIMITIVE MEMBER GETTERS ===========================================================
void
ser_get_bool_value(serialization_support_t * ser, const ser_dynamic_data_t * data, bool * value, MemberId id)
{
  (ser->interface->get_bool_value)(ser->impl, data, value, id);
}


void
ser_get_byte_value(serialization_support_t * ser, const ser_dynamic_data_t * data, uint8_t * value, MemberId id)
{
  (ser->interface->get_byte_value)(ser->impl, data, value, id);
}


void
ser_get_char_value(serialization_support_t * ser, const ser_dynamic_data_t * data, char * value, MemberId id)
{
  (ser->interface->get_char_value)(ser->impl, data, value, id);
}


void
ser_get_float32_value(serialization_support_t * ser, const ser_dynamic_data_t * data, float * value, MemberId id)
{
  (ser->interface->get_float32_value)(ser->impl, data, value, id);
}


void
ser_get_float64_value(serialization_support_t * ser, const ser_dynamic_data_t * data, double * value, MemberId id)
{
  (ser->interface->get_float64_value)(ser->impl, data, value, id);
}


void
ser_get_int8_value(serialization_support_t * ser, const ser_dynamic_data_t * data, int8_t * value, MemberId id)
{
  (ser->interface->get_int8_value)(ser->impl, data, value, id);
}


void
ser_get_uint8_value(serialization_support_t * ser, const ser_dynamic_data_t * data, uint8_t * value, MemberId id)
{
  (ser->interface->get_uint8_value)(ser->impl, data, value, id);
}


void
ser_get_int16_value(serialization_support_t * ser, const ser_dynamic_data_t * data, int16_t * value, MemberId id)
{
  (ser->interface->get_int16_value)(ser->impl, data, value, id);
}


void
ser_get_uint16_value(serialization_support_t * ser, const ser_dynamic_data_t * data, uint16_t * value, MemberId id)
{
  (ser->interface->get_uint16_value)(ser->impl, data, value, id);
}


void
ser_get_int32_value(serialization_support_t * ser, const ser_dynamic_data_t * data, int32_t * value, MemberId id)
{
  (ser->interface->get_int32_value)(ser->impl, data, value, id);
}


void
ser_get_uint32_value(serialization_support_t * ser, const ser_dynamic_data_t * data, uint32_t * value, MemberId id)
{
  (ser->interface->get_uint32_value)(ser->impl, data, value, id);
}


void
ser_get_int64_value(serialization_support_t * ser, const ser_dynamic_data_t * data, int64_t * value, MemberId id)
{
  (ser->interface->get_int64_value)(ser->impl, data, value, id);
}


void
ser_get_uint64_value(serialization_support_t * ser, const ser_dynamic_data_t * data, uint64_t * value, MemberId id)
{
  (ser->interface->get_uint64_value)(ser->impl, data, value, id);
}


void
ser_get_string_value(
  serialization_support_t * ser, const ser_dynamic_data_t * data, const char ** value, MemberId id)
{
  (ser->interface->get_string_value)(ser->impl, data, value, id);
}


void
ser_get_wstring_value(
  serialization_support_t * ser, const ser_dynamic_data_t * data, const wchar_t ** value, MemberId id)
{
  (ser->interface->get_wstring_value)(ser->impl, data, value, id);
}


// DYNAMIC DATA PRIMITIVE MEMBER SETTERS ===========================================================
void
ser_set_bool_value(serialization_support_t * ser, ser_dynamic_data_t * data, bool value, MemberId id)
{
  (ser->interface->set_bool_value)(ser->impl, data, value, id);
}


void
ser_set_byte_value(serialization_support_t * ser, ser_dynamic_data_t * data, uint8_t value, MemberId id)
{
  (ser->interface->set_byte_value)(ser->impl, data, value, id);
}


void
ser_set_char_value(serialization_support_t * ser, ser_dynamic_data_t * data, char value, MemberId id)
{
  (ser->interface->set_char_value)(ser->impl, data, value, id);
}


void
ser_set_float32_value(serialization_support_t * ser, ser_dynamic_data_t * data, float value, MemberId id)
{
  (ser->interface->set_float32_value)(ser->impl, data, value, id);
}


void
ser_set_float64_value(serialization_support_t * ser, ser_dynamic_data_t * data, double value, MemberId id)
{
  (ser->interface->set_float64_value)(ser->impl, data, value, id);
}


void
ser_set_int8_value(serialization_support_t * ser, ser_dynamic_data_t * data, int8_t value, MemberId id)
{
  (ser->interface->set_int8_value)(ser->impl, data, value, id);
}


void
ser_set_uint8_value(serialization_support_t * ser, ser_dynamic_data_t * data, uint8_t value, MemberId id)
{
  (ser->interface->set_uint8_value)(ser->impl, data, value, id);
}


void
ser_set_int16_value(serialization_support_t * ser, ser_dynamic_data_t * data, int16_t value, MemberId id)
{
  (ser->interface->set_int16_value)(ser->impl, data, value, id);
}


void
ser_set_uint16_value(serialization_support_t * ser, ser_dynamic_data_t * data, uint16_t value, MemberId id)
{
  (ser->interface->set_uint16_value)(ser->impl, data, value, id);
}


void
ser_set_int32_value(serialization_support_t * ser, ser_dynamic_data_t * data, int32_t value, MemberId id)
{
  (ser->interface->set_int32_value)(ser->impl, data, value, id);
}


void
ser_set_uint32_value(serialization_support_t * ser, ser_dynamic_data_t * data, uint32_t value, MemberId id)
{
  (ser->interface->set_uint32_value)(ser->impl, data, value, id);
}


void
ser_set_int64_value(serialization_support_t * ser, ser_dynamic_data_t * data, int64_t value, MemberId id)
{
  (ser->interface->set_int64_value)(ser->impl, data, value, id);
}


void
ser_set_uint64_value(serialization_support_t * ser, ser_dynamic_data_t * data, uint64_t value, MemberId id)
{
  (ser->interface->set_uint64_value)(ser->impl, data, value, id);
}


void
ser_set_string_value(
  serialization_support_t * ser, ser_dynamic_data_t * data, const char * value, MemberId id)
{
  (ser->interface->set_string_value)(ser->impl, data, value, id);
}


void
ser_set_wstring_value(
  serialization_support_t * ser, ser_dynamic_data_t * data, const wchar_t * value, MemberId id)
{
  (ser->interface->set_wstring_value)(ser->impl, data, value, id);
}


// DYNAMIC DATA SEQUENCES ==========================================================================
void
ser_clear_sequence_data(serialization_support_t * ser, ser_dynamic_data_t * data)
{
  (ser->interface->clear_sequence_data)(ser->impl, data);
}


void
ser_remove_sequence_data(serialization_support_t * ser, ser_dynamic_data_t * data, MemberId id)
{
  (ser->interface->remove_sequence_data)(ser->impl, data, id);
}


void
ser_insert_sequence_data(serialization_support_t * ser, ser_dynamic_data_t * data, MemberId * out_id)
{
  (ser->interface->insert_sequence_data)(ser->impl, data, out_id);
}


void
ser_insert_bool_value(serialization_support_t * ser, ser_dynamic_data_t * data, bool value, MemberId * out_id)
{
  (ser->interface->insert_bool_value)(ser->impl, data, value, out_id);
}


void
ser_insert_byte_value(serialization_support_t * ser, ser_dynamic_data_t * data, uint8_t value, MemberId * out_id)
{
  (ser->interface->insert_byte_value)(ser->impl, data, value, out_id);
}


void
ser_insert_char_value(serialization_support_t * ser, ser_dynamic_data_t * data, char value, MemberId * out_id)
{
  (ser->interface->insert_char_value)(ser->impl, data, value, out_id);
}


void
ser_insert_float32_value(serialization_support_t * ser, ser_dynamic_data_t * data, float value, MemberId * out_id)
{
  (ser->interface->insert_float32_value)(ser->impl, data, value, out_id);
}


void
ser_insert_float64_value(serialization_support_t * ser, ser_dynamic_data_t * data, double value, MemberId * out_id)
{
  (ser->interface->insert_float64_value)(ser->impl, data, value, out_id);
}


void
ser_insert_int16_value(serialization_support_t * ser, ser_dynamic_data_t * data, int16_t value, MemberId * out_id)
{
  (ser->interface->insert_int16_value)(ser->impl, data, value, out_id);
}


void
ser_insert_uint16_value(serialization_support_t * ser, ser_dynamic_data_t * data, uint16_t value, MemberId * out_id)
{
  (ser->interface->insert_uint16_value)(ser->impl, data, value, out_id);
}


void
ser_insert_int32_value(serialization_support_t * ser, ser_dynamic_data_t * data, int32_t value, MemberId * out_id)
{
  (ser->interface->insert_int32_value)(ser->impl, data, value, out_id);
}


void
ser_insert_uint32_value(serialization_support_t * ser, ser_dynamic_data_t * data, uint32_t value, MemberId * out_id)
{
  (ser->interface->insert_uint32_value)(ser->impl, data, value, out_id);
}


void
ser_insert_int64_value(serialization_support_t * ser, ser_dynamic_data_t * data, int64_t value, MemberId * out_id)
{
  (ser->interface->insert_int64_value)(ser->impl, data, value, out_id);
}


void
ser_insert_uint64_value(serialization_support_t * ser, ser_dynamic_data_t * data, uint64_t value, MemberId * out_id)
{
  (ser->interface->insert_uint64_value)(ser->impl, data, value, out_id);
}


void
ser_insert_string_value(serialization_support_t * ser, ser_dynamic_data_t * data, const char * value, MemberId * out_id)
{
  (ser->interface->insert_string_value)(ser->impl, data, value, out_id);
}


void
ser_insert_wstring_value(
  serialization_support_t * ser, ser_dynamic_data_t * data, const wchar_t * value, MemberId * out_id)
{
  (ser->interface->insert_wstring_value)(ser->impl, data, value, out_id);
}


// DYNAMIC DATA NESTED =============================================================================
void
ser_get_complex_value(serialization_support_t * ser, const ser_dynamic_data_t * data, ser_dynamic_data_t ** value, MemberId id)
{
  (ser->interface->get_complex_value)(ser->impl, data, value, id);
}


void
ser_set_complex_value(serialization_support_t * ser, ser_dynamic_data_t * data, ser_dynamic_data_t * value, MemberId id)
{
  (ser->interface->set_complex_value)(ser->impl, data, value, id);
}


void
ser_insert_const_complex_value(
  serialization_support_t * ser, ser_dynamic_data_t * data, const ser_dynamic_data_t * value, MemberId * out_id)
{
  (ser->interface->insert_const_complex_value)(ser->impl, data, value, out_id);
}


void
ser_insert_complex_value(serialization_support_t * ser, ser_dynamic_data_t * data, ser_dynamic_data_t * value, MemberId * out_id)
{
  (ser->interface->insert_complex_value)(ser->impl, data, value, out_id);
}


void
ser_insert_complex_value_ptr(
  serialization_support_t * ser, ser_dynamic_data_t * data, ser_dynamic_data_t * value, MemberId * out_id)
{
  (ser->interface->insert_complex_value_ptr)(ser->impl, data, value, out_id);
}

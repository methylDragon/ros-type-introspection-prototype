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

#ifndef ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS__DETAIL__DYNAMIC_DATA_H_
#define ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS__DETAIL__DYNAMIC_DATA_H_

#include <rosidl_dynamic_typesupport/api/serialization_support_interface.h>


#ifdef __cplusplus
extern "C" {
#endif


// =================================================================================================
// DYNAMIC DATA
// =================================================================================================

// DYNAMIC DATA UTILS ==============================================================================
void
fastrtps__clear_all_values(rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl);

// Clears all values, except for aggregated type keys
void
fastrtps__clear_nonkey_values(rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl);

void
fastrtps__clear_value(rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, rosidl_dynamic_typesupport_member_id_t id);

bool
fastrtps__dynamic_data_equals(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * other_data_impl);

// Can be used to get sequence/array length, and also number of members for struct
uint32_t
fastrtps__get_dynamic_data_item_count(rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl);

rosidl_dynamic_typesupport_member_id_t
fastrtps__get_dynamic_data_member_id_by_name(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, const char * name);

rosidl_dynamic_typesupport_member_id_t
fastrtps__get_dynamic_data_member_id_at_index(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, uint32_t index);

rosidl_dynamic_typesupport_member_id_t
fastrtps__get_dynamic_data_member_id_at_index(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, uint32_t index);

rosidl_dynamic_typesupport_member_id_t
fastrtps__get_array_index(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, uint32_t index);

rosidl_dynamic_typesupport_dynamic_data_impl_t *
fastrtps__loan_value(rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, rosidl_dynamic_typesupport_member_id_t id);

// The passed 'inner_data_impl' arg to return must be the immediate child of the passed 'data_impl' arg
void
fastrtps__return_loaned_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * inner_data_impl);

void
fastrtps__dynamic_data_print(rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl);


// DYNAMIC DATA CONSTRUCTION =======================================================================
rosidl_dynamic_typesupport_dynamic_data_impl_t *
fastrtps__dynamic_data_init_from_builder(rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl);

rosidl_dynamic_typesupport_dynamic_data_impl_t *
fastrtps__dynamic_data_init_from_dynamic_type(rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_type_impl_t * type_impl);

rosidl_dynamic_typesupport_dynamic_data_impl_t *
fastrtps__dynamic_data_clone(rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl);

void
fastrtps__dynamic_data_fini(rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl);


// DYNAMIC DATA PRIMITIVE MEMBERS GETTERS ==========================================================
void
fastrtps__get_bool_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, bool * value, rosidl_dynamic_typesupport_member_id_t id);

void
fastrtps__get_byte_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, uint8_t * value, rosidl_dynamic_typesupport_member_id_t id);

void
fastrtps__get_char_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, char * value, rosidl_dynamic_typesupport_member_id_t id);

void
fastrtps__get_float32_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, float * value, rosidl_dynamic_typesupport_member_id_t id);

void
fastrtps__get_float64_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, double * value, rosidl_dynamic_typesupport_member_id_t id);

void
fastrtps__get_int8_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, int8_t * value, rosidl_dynamic_typesupport_member_id_t id);

void
fastrtps__get_uint8_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, uint8_t * value, rosidl_dynamic_typesupport_member_id_t id);

void
fastrtps__get_int16_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, int16_t * value, rosidl_dynamic_typesupport_member_id_t id);

void
fastrtps__get_uint16_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, uint16_t * value, rosidl_dynamic_typesupport_member_id_t id);

void
fastrtps__get_int32_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, int32_t * value, rosidl_dynamic_typesupport_member_id_t id);

void
fastrtps__get_uint32_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, uint32_t * value, rosidl_dynamic_typesupport_member_id_t id);

void
fastrtps__get_int64_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, int64_t * value, rosidl_dynamic_typesupport_member_id_t id);

void
fastrtps__get_uint64_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, uint64_t * value, rosidl_dynamic_typesupport_member_id_t id);

void
fastrtps__get_string_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, const char ** value, rosidl_dynamic_typesupport_member_id_t id);

void
fastrtps__get_wstring_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, const wchar_t ** value, rosidl_dynamic_typesupport_member_id_t id);


// DYNAMIC DATA PRIMITIVE MEMBERS SETTERS ==========================================================
void
fastrtps__set_bool_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, bool value, rosidl_dynamic_typesupport_member_id_t id);

void
fastrtps__set_byte_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, uint8_t value, rosidl_dynamic_typesupport_member_id_t id);

void
fastrtps__set_char_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, char value, rosidl_dynamic_typesupport_member_id_t id);

void
fastrtps__set_float32_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, float value, rosidl_dynamic_typesupport_member_id_t id);

void
fastrtps__set_float64_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, double value, rosidl_dynamic_typesupport_member_id_t id);

void
fastrtps__set_int8_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, int8_t value, rosidl_dynamic_typesupport_member_id_t id);

void
fastrtps__set_uint8_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, uint8_t value, rosidl_dynamic_typesupport_member_id_t id);

void
fastrtps__set_int16_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, int16_t value, rosidl_dynamic_typesupport_member_id_t id);

void
fastrtps__set_uint16_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, uint16_t value, rosidl_dynamic_typesupport_member_id_t id);

void
fastrtps__set_int32_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, int32_t value, rosidl_dynamic_typesupport_member_id_t id);

void
fastrtps__set_uint32_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, uint32_t value, rosidl_dynamic_typesupport_member_id_t id);

void
fastrtps__set_int64_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, int64_t value, rosidl_dynamic_typesupport_member_id_t id);

void
fastrtps__set_uint64_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, uint64_t value, rosidl_dynamic_typesupport_member_id_t id);

void
fastrtps__set_string_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, const char * value, rosidl_dynamic_typesupport_member_id_t id);

void
fastrtps__set_wstring_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, const wchar_t * value, rosidl_dynamic_typesupport_member_id_t id);


// DYNAMIC DATA SEQUENCES ==========================================================================
void
fastrtps__clear_sequence_data(rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl);

void
fastrtps__remove_sequence_data(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, rosidl_dynamic_typesupport_member_id_t id);

void
fastrtps__insert_sequence_data(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, rosidl_dynamic_typesupport_member_id_t * out_id);

void
fastrtps__insert_bool_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, bool value, rosidl_dynamic_typesupport_member_id_t * out_id);

void
fastrtps__insert_byte_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, uint8_t value, rosidl_dynamic_typesupport_member_id_t * out_id);

void
fastrtps__insert_char_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, char value, rosidl_dynamic_typesupport_member_id_t * out_id);

void
fastrtps__insert_float32_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, float value, rosidl_dynamic_typesupport_member_id_t * out_id);

void
fastrtps__insert_float64_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, double value, rosidl_dynamic_typesupport_member_id_t * out_id);

void
fastrtps__insert_int16_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, int16_t value, rosidl_dynamic_typesupport_member_id_t * out_id);

void
fastrtps__insert_uint16_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, uint16_t value, rosidl_dynamic_typesupport_member_id_t * out_id);

void
fastrtps__insert_int32_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, int32_t value, rosidl_dynamic_typesupport_member_id_t * out_id);

void
fastrtps__insert_uint32_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, uint32_t value, rosidl_dynamic_typesupport_member_id_t * out_id);

void
fastrtps__insert_int64_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, int64_t value, rosidl_dynamic_typesupport_member_id_t * out_id);

void
fastrtps__insert_uint64_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, uint64_t value, rosidl_dynamic_typesupport_member_id_t * out_id);

void
fastrtps__insert_string_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, const char * value, rosidl_dynamic_typesupport_member_id_t * out_id);

void
fastrtps__insert_wstring_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, const wchar_t * value,
  rosidl_dynamic_typesupport_member_id_t * out_id);


// DYNAMIC DATA NESTED MEMBERS =====================================================================
void
fastrtps__get_complex_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t ** value, rosidl_dynamic_typesupport_member_id_t id);

void
fastrtps__set_complex_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * value, rosidl_dynamic_typesupport_member_id_t id);

void
fastrtps__insert_const_complex_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * value, rosidl_dynamic_typesupport_member_id_t * out_id);

void
fastrtps__insert_complex_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * value, rosidl_dynamic_typesupport_member_id_t * out_id);

void
fastrtps__insert_complex_value_ptr(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * value, rosidl_dynamic_typesupport_member_id_t * out_id);


#ifdef __cplusplus
}
#endif

#endif  // ROSIDL_DYNAMIC_TYPESUPPORT_FASTRTPS__DETAIL__DYNAMIC_DATA_H_

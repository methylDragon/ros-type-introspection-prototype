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

#ifndef SERIALIZATION_SUPPORT_FASTRTPS_C__DYNAMIC_DATA_H_
#define SERIALIZATION_SUPPORT_FASTRTPS_C__DYNAMIC_DATA_H_

#include <serialization_support_fastrtps_c/serialization_impl.h>


#ifdef __cplusplus
extern "C" {
#endif


// =================================================================================================
// DYNAMIC DATA
// =================================================================================================

// DYNAMIC DATA UTILS ==============================================================================
void
fastrtps__clear_all_values(ser_impl_t * ser_impl, ser_dynamic_data_t * data);

// Clears all values, except for aggregated type keys
void
fastrtps__clear_nonkey_values(ser_impl_t * ser_impl, ser_dynamic_data_t * data);

void
fastrtps__clear_value(ser_impl_t * ser_impl, ser_dynamic_data_t * data, MemberId id);

bool
fastrtps__data_equals(
  ser_impl_t * ser_impl, const ser_dynamic_data_t * data, const ser_dynamic_data_t * other);

// Can be used to get sequence/array length, and also number of members for struct
uint32_t
fastrtps__get_data_item_count(ser_impl_t * ser_impl, const ser_dynamic_data_t * data);

MemberId
fastrtps__get_data_member_id_by_name(
  ser_impl_t * ser_impl, const ser_dynamic_data_t * data, const char * name);

MemberId
fastrtps__get_data_member_id_at_index(
  ser_impl_t * ser_impl, const ser_dynamic_data_t * data, uint32_t index);

MemberId
fastrtps__get_data_member_id_at_index(
  ser_impl_t * ser_impl, const ser_dynamic_data_t * data, uint32_t index);

MemberId
fastrtps__get_array_index(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, uint32_t index);

ser_dynamic_data_t *
fastrtps__loan_value(ser_impl_t * ser_impl, ser_dynamic_data_t * data, MemberId id);

// The passed 'inner_data' arg to return must be the immediate child of the passed 'data' arg
void
fastrtps__return_loaned_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, const ser_dynamic_data_t * inner_data);

void
fastrtps__print_dynamic_data(ser_impl_t * ser_impl, ser_dynamic_data_t * data);


// DYNAMIC DATA CONSTRUCTION =======================================================================
ser_dynamic_data_t *
fastrtps__data_init_from_builder(ser_impl_t * ser_impl, ser_type_builder_t * builder);

ser_dynamic_data_t *
fastrtps__data_init_from_type(ser_impl_t * ser_impl, ser_dynamic_type_t * type);

ser_dynamic_data_t *
fastrtps__data_clone(ser_impl_t * ser_impl, const ser_dynamic_data_t * data);

void
fastrtps__data_fini(ser_impl_t * ser_impl, ser_dynamic_data_t * data);


// DYNAMIC DATA PRIMITIVE MEMBERS GETTERS ==========================================================
void
fastrtps__get_bool_value(
  ser_impl_t * ser_impl, const ser_dynamic_data_t * data, bool * value, MemberId id);

void
fastrtps__get_byte_value(
  ser_impl_t * ser_impl, const ser_dynamic_data_t * data, uint8_t * value, MemberId id);

void
fastrtps__get_char_value(
  ser_impl_t * ser_impl, const ser_dynamic_data_t * data, char * value, MemberId id);

void
fastrtps__get_float32_value(
  ser_impl_t * ser_impl, const ser_dynamic_data_t * data, float * value, MemberId id);

void
fastrtps__get_float64_value(
  ser_impl_t * ser_impl, const ser_dynamic_data_t * data, double * value, MemberId id);

void
fastrtps__get_int8_value(
  ser_impl_t * ser_impl, const ser_dynamic_data_t * data, int8_t * value, MemberId id);

void
fastrtps__get_uint8_value(
  ser_impl_t * ser_impl, const ser_dynamic_data_t * data, uint8_t * value, MemberId id);

void
fastrtps__get_int16_value(
  ser_impl_t * ser_impl, const ser_dynamic_data_t * data, int16_t * value, MemberId id);

void
fastrtps__get_uint16_value(
  ser_impl_t * ser_impl, const ser_dynamic_data_t * data, uint16_t * value, MemberId id);

void
fastrtps__get_int32_value(
  ser_impl_t * ser_impl, const ser_dynamic_data_t * data, int32_t * value, MemberId id);

void
fastrtps__get_uint32_value(
  ser_impl_t * ser_impl, const ser_dynamic_data_t * data, uint32_t * value, MemberId id);

void
fastrtps__get_int64_value(
  ser_impl_t * ser_impl, const ser_dynamic_data_t * data, int64_t * value, MemberId id);

void
fastrtps__get_uint64_value(
  ser_impl_t * ser_impl, const ser_dynamic_data_t * data, uint64_t * value, MemberId id);

void
fastrtps__get_string_value(
  ser_impl_t * ser_impl, const ser_dynamic_data_t * data, const char ** value, MemberId id);

void
fastrtps__get_wstring_value(
  ser_impl_t * ser_impl,
  const ser_dynamic_data_t * data, const wchar_t ** value, MemberId id);


// DYNAMIC DATA PRIMITIVE MEMBERS SETTERS ==========================================================
void
fastrtps__set_bool_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, bool value, MemberId id);

void
fastrtps__set_byte_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, uint8_t value, MemberId id);

void
fastrtps__set_char_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, char value, MemberId id);

void
fastrtps__set_float32_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, float value, MemberId id);

void
fastrtps__set_float64_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, double value, MemberId id);

void
fastrtps__set_int8_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, int8_t value, MemberId id);

void
fastrtps__set_uint8_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, uint8_t value, MemberId id);

void
fastrtps__set_int16_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, int16_t value, MemberId id);

void
fastrtps__set_uint16_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, uint16_t value, MemberId id);

void
fastrtps__set_int32_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, int32_t value, MemberId id);

void
fastrtps__set_uint32_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, uint32_t value, MemberId id);

void
fastrtps__set_int64_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, int64_t value, MemberId id);

void
fastrtps__set_uint64_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, uint64_t value, MemberId id);

void
fastrtps__set_string_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, const char * value, MemberId id);

void
fastrtps__set_wstring_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, const wchar_t * value, MemberId id);


// DYNAMIC DATA SEQUENCES ==========================================================================
void
fastrtps__clear_sequence_data(ser_impl_t * ser_impl, ser_dynamic_data_t * data);

void
fastrtps__remove_sequence_data(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, MemberId id);

void
fastrtps__insert_sequence_data(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, MemberId * out_id);

void
fastrtps__insert_bool_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, bool value, MemberId * out_id);

void
fastrtps__insert_byte_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, uint8_t value, MemberId * out_id);

void
fastrtps__insert_char_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, char value, MemberId * out_id);

void
fastrtps__insert_float32_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, float value, MemberId * out_id);

void
fastrtps__insert_float64_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, double value, MemberId * out_id);

void
fastrtps__insert_int16_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, int16_t value, MemberId * out_id);

void
fastrtps__insert_uint16_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, uint16_t value, MemberId * out_id);

void
fastrtps__insert_int32_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, int32_t value, MemberId * out_id);

void
fastrtps__insert_uint32_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, uint32_t value, MemberId * out_id);

void
fastrtps__insert_int64_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, int64_t value, MemberId * out_id);

void
fastrtps__insert_uint64_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, uint64_t value, MemberId * out_id);

void
fastrtps__insert_string_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, const char * value, MemberId * out_id);

void
fastrtps__insert_wstring_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, const wchar_t * value,
  MemberId * out_id);


// DYNAMIC DATA NESTED MEMBERS =====================================================================
void
fastrtps__get_complex_value(
  ser_impl_t * ser_impl, const ser_dynamic_data_t * data, ser_dynamic_data_t ** value, MemberId id);

void
fastrtps__set_complex_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, ser_dynamic_data_t * value, MemberId id);

void
fastrtps__insert_const_complex_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, const ser_dynamic_data_t * value, MemberId * out_id);

void
fastrtps__insert_complex_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, ser_dynamic_data_t * value, MemberId * out_id);

void
fastrtps__insert_complex_value_ptr(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, ser_dynamic_data_t * value, MemberId * out_id);


#ifdef __cplusplus
}
#endif

#endif  // SERIALIZATION_SUPPORT_FASTRTPS_C__DYNAMIC_DATA_H_

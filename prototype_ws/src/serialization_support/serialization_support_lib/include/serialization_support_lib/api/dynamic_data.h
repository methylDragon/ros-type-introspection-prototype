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

/// Polymorphic serialization support interface
/// Downstream middlewares should populate this interface as appropriate

#ifndef SERIALIZATION_SUPPORT_LIB__ETS__DYNAMIC_DATA_H_
#define SERIALIZATION_SUPPORT_LIB__ETS__DYNAMIC_DATA_H_

#include <serialization_support_lib/api/serialization_support_interface.h>


#ifdef __cplusplus
extern "C" {
#endif

// ===============================================================================================
// DYNAMIC DATA
// ===============================================================================================

// DYNAMIC DATA UTILS ==============================================================================
void
ser_clear_all_values(serialization_support_t * ser, ser_dynamic_data_t * data);

void
ser_clear_nonkey_values(serialization_support_t * ser, ser_dynamic_data_t * data);

void
ser_clear_value(serialization_support_t * ser, ser_dynamic_data_t * data, MemberId id);

bool
ser_data_equals(serialization_support_t * ser, const ser_dynamic_data_t * data, const ser_dynamic_data_t * other);

uint32_t
ser_get_data_item_count(serialization_support_t * ser, const ser_dynamic_data_t * data);

MemberId
ser_get_data_member_id_by_name(serialization_support_t * ser, const ser_dynamic_data_t * data, const char * name);

MemberId
ser_get_data_member_id_at_index(serialization_support_t * ser, const ser_dynamic_data_t * data, uint32_t index);

// You must use this for arrays
MemberId
ser_get_array_index(serialization_support_t * ser, ser_dynamic_data_t * data, uint32_t index);

ser_dynamic_data_t *
ser_loan_value(serialization_support_t * ser, ser_dynamic_data_t * data, MemberId id);

void
ser_return_loaned_value(serialization_support_t * ser, ser_dynamic_data_t * data, ser_dynamic_data_t * inner_data);

void
ser_print_dynamic_data(serialization_support_t * ser, ser_dynamic_data_t * data);

// DYNAMIC DATA CONSTRUCTION =======================================================================
ser_dynamic_data_t *
ser_data_init_from_builder(serialization_support_t * ser, ser_type_builder_t * builder);

ser_dynamic_data_t *
ser_data_init_from_type(serialization_support_t * ser, ser_dynamic_type_t * type);

ser_dynamic_data_t *
ser_data_clone(serialization_support_t * ser, const ser_dynamic_data_t * data);

void
ser_data_fini(serialization_support_t * ser, ser_dynamic_data_t * data);

// DYNAMIC DATA PRIMITIVE MEMBER GETTERS ===========================================================
void
ser_get_bool_value(serialization_support_t * ser, const ser_dynamic_data_t * data, bool * value, MemberId id);

void
ser_get_byte_value(serialization_support_t * ser, const ser_dynamic_data_t * data, uint8_t * value, MemberId id);

void
ser_get_char_value(serialization_support_t * ser, const ser_dynamic_data_t * data, char * value, MemberId id);

void
ser_get_float32_value(serialization_support_t * ser, const ser_dynamic_data_t * data, float * value, MemberId id);

void
ser_get_float64_value(serialization_support_t * ser, const ser_dynamic_data_t * data, double * value, MemberId id);

void
ser_get_int8_value(serialization_support_t * ser, const ser_dynamic_data_t * data, int8_t * value, MemberId id);

void
ser_get_uint8_value(serialization_support_t * ser, const ser_dynamic_data_t * data, uint8_t * value, MemberId id);

void
ser_get_int16_value(serialization_support_t * ser, const ser_dynamic_data_t * data, int16_t * value, MemberId id);

void
ser_get_uint16_value(serialization_support_t * ser, const ser_dynamic_data_t * data, uint16_t * value, MemberId id);

void
ser_get_int32_value(serialization_support_t * ser, const ser_dynamic_data_t * data, int32_t * value, MemberId id);

void
ser_get_uint32_value(serialization_support_t * ser, const ser_dynamic_data_t * data, uint32_t * value, MemberId id);

void
ser_get_int64_value(serialization_support_t * ser, const ser_dynamic_data_t * data, int64_t * value, MemberId id);

void
ser_get_uint64_value(serialization_support_t * ser, const ser_dynamic_data_t * data, uint64_t * value, MemberId id);

void
ser_get_string_value(
  serialization_support_t * ser, const ser_dynamic_data_t * data, const char ** value, MemberId id);

void
ser_get_wstring_value(
  serialization_support_t * ser, const ser_dynamic_data_t * data, const wchar_t ** value, MemberId id);

// DYNAMIC DATA PRIMITIVE MEMBER SETTERS ===========================================================
void
ser_set_bool_value(serialization_support_t * ser, ser_dynamic_data_t * data, bool value, MemberId id);

void
ser_set_byte_value(serialization_support_t * ser, ser_dynamic_data_t * data, uint8_t value, MemberId id);

void
ser_set_char_value(serialization_support_t * ser, ser_dynamic_data_t * data, char value, MemberId id);

void
ser_set_float32_value(serialization_support_t * ser, ser_dynamic_data_t * data, float value, MemberId id);

void
ser_set_float64_value(serialization_support_t * ser, ser_dynamic_data_t * data, double value, MemberId id);

void
ser_set_int8_value(serialization_support_t * ser, ser_dynamic_data_t * data, int8_t value, MemberId id);

void
ser_set_uint8_value(serialization_support_t * ser, ser_dynamic_data_t * data, uint8_t value, MemberId id);

void
ser_set_int16_value(serialization_support_t * ser, ser_dynamic_data_t * data, int16_t value, MemberId id);

void
ser_set_uint16_value(serialization_support_t * ser, ser_dynamic_data_t * data, uint16_t value, MemberId id);

void
ser_set_int32_value(serialization_support_t * ser, ser_dynamic_data_t * data, int32_t value, MemberId id);

void
ser_set_uint32_value(serialization_support_t * ser, ser_dynamic_data_t * data, uint32_t value, MemberId id);

void
ser_set_int64_value(serialization_support_t * ser, ser_dynamic_data_t * data, int64_t value, MemberId id);

void
ser_set_uint64_value(serialization_support_t * ser, ser_dynamic_data_t * data, uint64_t value, MemberId id);

void
ser_set_string_value(
  serialization_support_t * ser, ser_dynamic_data_t * data, const char * value, MemberId id);

void
ser_set_wstring_value(
  serialization_support_t * ser, ser_dynamic_data_t * data, const wchar_t * value, MemberId id);

// DYNAMIC DATA SEQUENCES ==========================================================================
void
ser_clear_sequence_data(serialization_support_t * ser, ser_dynamic_data_t * data);

void
ser_remove_sequence_data(serialization_support_t * ser, ser_dynamic_data_t * data, MemberId id);

void
ser_insert_sequence_data(serialization_support_t * ser, ser_dynamic_data_t * data, MemberId * out_id);

void
ser_insert_bool_value(serialization_support_t * ser, ser_dynamic_data_t * data, bool value, MemberId * out_id);

void
ser_insert_byte_value(serialization_support_t * ser, ser_dynamic_data_t * data, uint8_t value, MemberId * out_id);

void
ser_insert_char_value(serialization_support_t * ser, ser_dynamic_data_t * data, char value, MemberId * out_id);

void
ser_insert_float32_value(serialization_support_t * ser, ser_dynamic_data_t * data, float value, MemberId * out_id);

void
ser_insert_float64_value(serialization_support_t * ser, ser_dynamic_data_t * data, double value, MemberId * out_id);

void
ser_insert_int16_value(serialization_support_t * ser, ser_dynamic_data_t * data, int16_t value, MemberId * out_id);

void
ser_insert_uint16_value(serialization_support_t * ser, ser_dynamic_data_t * data, uint16_t value, MemberId * out_id);

void
ser_insert_int32_value(serialization_support_t * ser, ser_dynamic_data_t * data, int32_t value, MemberId * out_id);

void
ser_insert_uint32_value(serialization_support_t * ser, ser_dynamic_data_t * data, uint32_t value, MemberId * out_id);

void
ser_insert_int64_value(serialization_support_t * ser, ser_dynamic_data_t * data, int64_t value, MemberId * out_id);

void
ser_insert_uint64_value(serialization_support_t * ser, ser_dynamic_data_t * data, uint64_t value, MemberId * out_id);

void
ser_insert_string_value(serialization_support_t * ser, ser_dynamic_data_t * data, const char * value, MemberId * out_id);

void
ser_insert_wstring_value(
  serialization_support_t * ser, ser_dynamic_data_t * data, const wchar_t * value, MemberId * out_id);

// DYNAMIC DATA NESTED =============================================================================
void
ser_get_complex_value(serialization_support_t * ser, const ser_dynamic_data_t * data, ser_dynamic_data_t ** value, MemberId id);

void
ser_set_complex_value(serialization_support_t * ser, ser_dynamic_data_t * data, ser_dynamic_data_t * value, MemberId id);

void
ser_insert_const_complex_value(
  serialization_support_t * ser, ser_dynamic_data_t * data, const ser_dynamic_data_t * value, MemberId * out_id);

void
ser_insert_complex_value(serialization_support_t * ser, ser_dynamic_data_t * data, ser_dynamic_data_t * value, MemberId * out_id);

void
ser_insert_complex_value_ptr(
  serialization_support_t * ser, ser_dynamic_data_t * data, ser_dynamic_data_t * value, MemberId * out_id);

#ifdef __cplusplus
}
#endif

#endif  // SERIALIZATION_SUPPORT_LIB__ETS__DYNAMIC_DATA_H_

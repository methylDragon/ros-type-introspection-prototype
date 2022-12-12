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

#ifndef SERIALIZATION_SUPPORT_LIB__ETS__DYNAMIC_TYPE_H_
#define SERIALIZATION_SUPPORT_LIB__ETS__DYNAMIC_TYPE_H_

#include <serialization_support_lib/api/serialization_support_interface.h>


#ifdef __cplusplus
extern "C" {
#endif


// =================================================================================================
// DYNAMIC TYPE
// =================================================================================================

// DYNAMIC TYPE UTILS ==============================================================================
bool
ser_type_equals(serialization_support_t * ser, ser_dynamic_type_t * type, ser_dynamic_type_t * other);

uint32_t
ser_get_type_member_count(serialization_support_t * ser, const ser_dynamic_type_t * type);


// DYNAMIC TYPE CONSTRUCTION =======================================================================
ser_type_builder_t *
ser_struct_type_builder_init(serialization_support_t * ser, const char * name);

void
ser_struct_type_builder_fini(serialization_support_t * ser, ser_type_builder_t * builder);

ser_dynamic_type_t *
ser_build_struct_type(serialization_support_t * ser, ser_type_builder_t * builder);

ser_dynamic_type_t *
ser_construct_type_from_description(serialization_support_t * ser, type_description_t * description);

void
ser_type_fini(serialization_support_t * ser, ser_dynamic_type_t * type);


// DYNAMIC TYPE PRIMITIVE MEMBERS ==================================================================
void
ser_add_bool_member(serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name);

void
ser_add_byte_member(serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name);

void
ser_add_char_member(serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name);

void
ser_add_float32_member(serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name);

void
ser_add_float64_member(serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name);

void
ser_add_int8_member(serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name);

void
ser_add_uint8_member(serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name);

void
ser_add_int16_member(serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name);

void
ser_add_uint16_member(serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name);

void
ser_add_int32_member(serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name);

void
ser_add_uint32_member(serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name);

void
ser_add_int64_member(serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name);

void
ser_add_uint64_member(serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name);

void
ser_add_string_member(serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name);

void
ser_add_wstring_member(serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name);

void
ser_add_bounded_string_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);

void
ser_add_bounded_wstring_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);


// DYNAMIC TYPE STATIC ARRAY MEMBERS ===============================================================
void
ser_add_bool_static_array_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);

void
ser_add_byte_static_array_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);

void
ser_add_char_static_array_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);

void
ser_add_float32_static_array_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);

void
ser_add_float64_static_array_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);

void
ser_add_int8_static_array_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);

void
ser_add_uint8_static_array_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);

void
ser_add_int16_static_array_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);

void
ser_add_uint16_static_array_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);

void
ser_add_int32_static_array_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);

void
ser_add_uint32_static_array_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);

void
ser_add_int64_static_array_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);

void
ser_add_uint64_static_array_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);

void
ser_add_string_static_array_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);

void
ser_add_wstring_static_array_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);

void
ser_add_bounded_string_static_array_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t str_bound, uint32_t bound);

void
ser_add_bounded_wstring_static_array_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t str_bound, uint32_t bound);


// DYNAMIC TYPE UNBOUNDED SEQUENCE MEMBERS =========================================================
void
ser_add_bool_unbounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name);

void
ser_add_byte_unbounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name);

void
ser_add_char_unbounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name);

void
ser_add_float32_unbounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name);

void
ser_add_float64_unbounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name);

void
ser_add_int8_unbounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name);

void
ser_add_uint8_unbounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name);

void
ser_add_int16_unbounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name);

void
ser_add_uint16_unbounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name);

void
ser_add_int32_unbounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name);

void
ser_add_uint32_unbounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name);

void
ser_add_int64_unbounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name);

void
ser_add_uint64_unbounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name);

void
ser_add_string_unbounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name);

void
ser_add_wstring_unbounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name);

void
ser_add_bounded_string_unbounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t str_bound);

void
ser_add_bounded_wstring_unbounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t str_bound);


// DYNAMIC TYPE BOUNDED SEQUENCE MEMBERS ===========================================================
void
ser_add_bool_bounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);

void
ser_add_byte_bounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);

void
ser_add_char_bounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);

void
ser_add_float32_bounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);

void
ser_add_float64_bounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);

void
ser_add_int8_bounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);

void
ser_add_uint8_bounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);

void
ser_add_int16_bounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);

void
ser_add_uint16_bounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);

void
ser_add_int32_bounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);

void
ser_add_uint32_bounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);

void
ser_add_int64_bounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);

void
ser_add_uint64_bounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);

void
ser_add_string_bounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);

void
ser_add_wstring_bounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);

void
ser_add_bounded_string_bounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t str_bound, uint32_t bound);

void
ser_add_bounded_wstring_bounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t str_bound, uint32_t bound);


// DYNAMIC TYPE NESTED MEMBERS =====================================================================
void
ser_add_nested_struct_member(
  serialization_support_t * ser, ser_type_builder_t * builder,
  MemberId id, const char * name, ser_dynamic_type_t * nested_struct);

void
ser_add_nested_struct_static_array_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name,
  ser_dynamic_type_t * nested_struct, uint32_t bound);

void
ser_add_nested_struct_unbounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, ser_dynamic_type_t * nested_struct);

void
ser_add_nested_struct_bounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name,
  ser_dynamic_type_t * nested_struct, uint32_t bound);


#ifdef __cplusplus
}
#endif

#endif  // SERIALIZATION_SUPPORT_LIB__ETS__DYNAMIC_TYPE_H_

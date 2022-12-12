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

#ifndef SERIALIZATION_SUPPORT_FASTRTPS_C__DYNAMIC_TYPE_H_
#define SERIALIZATION_SUPPORT_FASTRTPS_C__DYNAMIC_TYPE_H_

#include <serialization_support_fastrtps_c/serialization_impl.h>


#ifdef __cplusplus
extern "C" {
#endif


// =================================================================================================
// DYNAMIC TYPE
// =================================================================================================

// DYNAMIC TYPE UTILS =======================================================================
bool
fastrtps__type_equals(
  ser_impl_t * ser_impl, const ser_dynamic_type_t * type, const ser_dynamic_type_t * other);

uint32_t
fastrtps__get_type_member_count(ser_impl_t * ser_impl, const ser_dynamic_type_t * type);


// DYNAMIC TYPE CONSTRUCTION =======================================================================
ser_type_builder_t *
fastrtps__struct_type_builder_init(ser_impl_t * ser_impl, const char * name);

void
fastrtps__struct_type_builder_fini(ser_impl_t * ser_impl, ser_type_builder_t * builder);

ser_dynamic_type_t *
fastrtps__build_struct_type(ser_impl_t * ser_impl, ser_type_builder_t * builder);

ser_dynamic_type_t *
fastrtps__construct_type_from_description(
  ser_impl_t * ser_impl, type_description_t * description);

void
fastrtps__type_fini(ser_impl_t * ser_impl, ser_dynamic_type_t * type);


// DYNAMIC TYPE PRIMITIVE MEMBERS ==================================================================
void
fastrtps__add_bool_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name);

void
fastrtps__add_byte_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name);

void
fastrtps__add_char_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name);

void
fastrtps__add_float32_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name);

void
fastrtps__add_float64_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name);

void
fastrtps__add_int8_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name);

void
fastrtps__add_uint8_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name);

void
fastrtps__add_int16_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name);

void
fastrtps__add_uint16_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name);

void
fastrtps__add_int32_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name);

void
fastrtps__add_uint32_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name);

void
fastrtps__add_int64_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name);

void
fastrtps__add_uint64_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name);

void
fastrtps__add_string_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name);

void
fastrtps__add_wstring_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name);

void
fastrtps__add_bounded_string_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder,
  MemberId id, const char * name,
  uint32_t bound);

void
fastrtps__add_bounded_wstring_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder,
  MemberId id, const char * name,
  uint32_t bound);


// DYNAMIC TYPE STATIC ARRAY MEMBERS ===============================================================
void
fastrtps__add_bool_static_array_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound);

void
fastrtps__add_byte_static_array_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound);

void
fastrtps__add_char_static_array_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound);

void
fastrtps__add_float32_static_array_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound);

void
fastrtps__add_float64_static_array_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound);

void
fastrtps__add_int8_static_array_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound);

void
fastrtps__add_uint8_static_array_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound);

void
fastrtps__add_int16_static_array_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound);

void
fastrtps__add_uint16_static_array_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound);

void
fastrtps__add_int32_static_array_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound);

void
fastrtps__add_uint32_static_array_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound);

void
fastrtps__add_int64_static_array_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound);

void
fastrtps__add_uint64_static_array_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound);

void
fastrtps__add_string_static_array_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound);

void
fastrtps__add_wstring_static_array_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound);

void
fastrtps__add_bounded_string_static_array_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t str_bound, uint32_t bound);

void
fastrtps__add_bounded_wstring_static_array_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t str_bound, uint32_t bound);


// DYNAMIC TYPE UNBOUNDED SEQUENCE MEMBERS =========================================================
void
fastrtps__add_bool_unbounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name);

void
fastrtps__add_byte_unbounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name);

void
fastrtps__add_char_unbounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name);

void
fastrtps__add_float32_unbounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name);

void
fastrtps__add_float64_unbounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name);

void
fastrtps__add_int8_unbounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name);

void
fastrtps__add_uint8_unbounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name);

void
fastrtps__add_int16_unbounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name);

void
fastrtps__add_uint16_unbounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name);

void
fastrtps__add_int32_unbounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name);

void
fastrtps__add_uint32_unbounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name);

void
fastrtps__add_int64_unbounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name);

void
fastrtps__add_uint64_unbounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name);

void
fastrtps__add_string_unbounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name);

void
fastrtps__add_wstring_unbounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name);

void
fastrtps__add_bounded_string_unbounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t str_bound);

void
fastrtps__add_bounded_wstring_unbounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t str_bound);


// DYNAMIC TYPE BOUNDED SEQUENCE MEMBERS ===========================================================
void
fastrtps__add_bool_bounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound);

void
fastrtps__add_byte_bounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound);

void
fastrtps__add_char_bounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound);

void
fastrtps__add_float32_bounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound);

void
fastrtps__add_float64_bounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound);

void
fastrtps__add_int8_bounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound);

void
fastrtps__add_uint8_bounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound);

void
fastrtps__add_int16_bounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound);

void
fastrtps__add_uint16_bounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound);

void
fastrtps__add_int32_bounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound);

void
fastrtps__add_uint32_bounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound);

void
fastrtps__add_int64_bounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound);

void
fastrtps__add_uint64_bounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound);

void
fastrtps__add_string_bounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound);

void
fastrtps__add_wstring_bounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound);

void
fastrtps__add_bounded_string_bounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t str_bound, uint32_t bound);

void
fastrtps__add_bounded_wstring_bounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t str_bound, uint32_t bound);


// DYNAMIC TYPE NESTED MEMBERS =====================================================================
void
fastrtps__add_nested_struct_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder,
  MemberId id, const char * name, ser_dynamic_type_t * nested_struct);

void
fastrtps__add_nested_struct_static_array_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  ser_dynamic_type_t * nested_struct, uint32_t bound);

void
fastrtps__add_nested_struct_unbounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  ser_dynamic_type_t * nested_struct);

void
fastrtps__add_nested_struct_bounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  ser_dynamic_type_t * nested_struct, uint32_t bound);


#ifdef __cplusplus
}
#endif

#endif  // SERIALIZATION_SUPPORT_FASTRTPS_C__DYNAMIC_TYPE_H_

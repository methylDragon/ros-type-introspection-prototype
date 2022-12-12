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
#include <serialization_support_lib/api/dynamic_type.h>


// =================================================================================================
// DYNAMIC TYPE
// =================================================================================================

// DYNAMIC TYPE UTILS ==============================================================================
bool
ser_type_equals(serialization_support_t * ser, ser_dynamic_type_t * type, ser_dynamic_type_t * other)
{
  return (ser->interface->type_equals)(ser->impl, type, other);
}


uint32_t
ser_get_type_member_count(serialization_support_t * ser, const ser_dynamic_type_t * type)
{
  return (ser->interface->get_type_member_count)(ser->impl, type);
}


// DYNAMIC TYPE CONSTRUCTION =======================================================================
ser_type_builder_t *
ser_struct_type_builder_init(serialization_support_t * ser, const char * name)
{
  return (ser->interface->struct_type_builder_init)(ser->impl, name);
}


void
ser_struct_type_builder_fini(serialization_support_t * ser, ser_type_builder_t * builder)
{
  (ser->interface->struct_type_builder_fini)(ser->impl, builder);
  free(builder);
}


ser_dynamic_type_t *
ser_build_struct_type(serialization_support_t * ser, ser_type_builder_t * builder)
{
  return (ser->interface->build_struct_type)(ser->impl, builder);
}


ser_dynamic_type_t *
ser_construct_type_from_description(serialization_support_t * ser, type_description_t * description)
{
  return (ser->interface->construct_type_from_description)(ser->impl, description);
}


void
ser_type_fini(serialization_support_t * ser, ser_dynamic_type_t * type)
{
  (ser->interface->type_fini)(ser->impl, type);
  free(type);
}


// DYNAMIC TYPE PRIMITIVE MEMBERS ==================================================================
void
ser_add_bool_member(serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name)
{
  (ser->interface->add_bool_member)(ser->impl, builder, id, name);
}


void
ser_add_byte_member(serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name)
{
  (ser->interface->add_byte_member)(ser->impl, builder, id, name);
}


void
ser_add_char_member(serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name)
{
  (ser->interface->add_char_member)(ser->impl, builder, id, name);
}


void
ser_add_float32_member(serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name)
{
  (ser->interface->add_float32_member)(ser->impl, builder, id, name);
}


void
ser_add_float64_member(serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name)
{
  (ser->interface->add_float64_member)(ser->impl, builder, id, name);
}


void
ser_add_int8_member(serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name)
{
  (ser->interface->add_int8_member)(ser->impl, builder, id, name);
}


void
ser_add_uint8_member(serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name)
{
  (ser->interface->add_uint8_member)(ser->impl, builder, id, name);
}


void
ser_add_int16_member(serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name)
{
  (ser->interface->add_int16_member)(ser->impl, builder, id, name);
}


void
ser_add_uint16_member(serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name)
{
  (ser->interface->add_uint16_member)(ser->impl, builder, id, name);
}


void
ser_add_int32_member(serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name)
{
  (ser->interface->add_int32_member)(ser->impl, builder, id, name);
}


void
ser_add_uint32_member(serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name)
{
  (ser->interface->add_uint32_member)(ser->impl, builder, id, name);
}


void
ser_add_int64_member(serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name)
{
  (ser->interface->add_int64_member)(ser->impl, builder, id, name);
}


void
ser_add_uint64_member(serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name)
{
  (ser->interface->add_uint64_member)(ser->impl, builder, id, name);
}


void
ser_add_string_member(serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name)
{
  (ser->interface->add_string_member)(ser->impl, builder, id, name);
}


void
ser_add_wstring_member(
  serialization_support_t * ser, ser_type_builder_t * builder,
  MemberId id, const char * name)
{
  (ser->interface->add_wstring_member)(ser->impl, builder, id, name);
}


void
ser_add_bounded_string_member(
  serialization_support_t * ser, ser_type_builder_t * builder,
  MemberId id, const char * name, uint32_t bound)
{
  (ser->interface->add_bounded_string_member)(ser->impl, builder, id, name, bound);
}


void
ser_add_bounded_wstring_member(
  serialization_support_t * ser, ser_type_builder_t * builder,
  MemberId id, const char * name, uint32_t bound)
{
  (ser->interface->add_bounded_wstring_member)(ser->impl, builder, id, name, bound);
}


// DYNAMIC TYPE STATIC ARRAY MEMBERS ===============================================================
void
ser_add_bool_static_array_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound)
{
  (ser->interface->add_bool_static_array_member)(ser->impl, builder, id, name, bound);
}


void
ser_add_byte_static_array_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound)
{
  (ser->interface->add_byte_static_array_member)(ser->impl, builder, id, name, bound);
}


void
ser_add_char_static_array_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound)
{
  (ser->interface->add_char_static_array_member)(ser->impl, builder, id, name, bound);
}


void
ser_add_float32_static_array_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound)
{
  (ser->interface->add_float32_static_array_member)(ser->impl, builder, id, name, bound);
}


void
ser_add_float64_static_array_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound)
{
  (ser->interface->add_float64_static_array_member)(ser->impl, builder, id, name, bound);
}


void
ser_add_int8_static_array_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound)
{
  (ser->interface->add_int8_static_array_member)(ser->impl, builder, id, name, bound);
}


void
ser_add_uint8_static_array_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound)
{
  (ser->interface->add_uint8_static_array_member)(ser->impl, builder, id, name, bound);
}


void
ser_add_int16_static_array_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound)
{
  (ser->interface->add_int16_static_array_member)(ser->impl, builder, id, name, bound);
}


void
ser_add_uint16_static_array_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound)
{
  (ser->interface->add_uint16_static_array_member)(ser->impl, builder, id, name, bound);
}


void
ser_add_int32_static_array_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound)
{
  (ser->interface->add_int32_static_array_member)(ser->impl, builder, id, name, bound);
}


void
ser_add_uint32_static_array_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound)
{
  (ser->interface->add_uint32_static_array_member)(ser->impl, builder, id, name, bound);
}


void
ser_add_int64_static_array_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound)
{
  (ser->interface->add_int64_static_array_member)(ser->impl, builder, id, name, bound);
}


void
ser_add_uint64_static_array_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound)
{
  (ser->interface->add_uint64_static_array_member)(ser->impl, builder, id, name, bound);
}


void
ser_add_string_static_array_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound)
{
  (ser->interface->add_string_static_array_member)(ser->impl, builder, id, name, bound);
}


void
ser_add_wstring_static_array_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound)
{
  (ser->interface->add_wstring_static_array_member)(ser->impl, builder, id, name, bound);
}


void
ser_add_bounded_string_static_array_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t str_bound, uint32_t bound)
{
  (ser->interface->add_bounded_string_static_array_member)(
    ser->impl, builder, id, name, str_bound, bound);
}


void
ser_add_bounded_wstring_static_array_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t str_bound, uint32_t bound)
{
  (ser->interface->add_bounded_wstring_static_array_member)(
    ser->impl, builder, id, name, str_bound, bound);
}


// DYNAMIC TYPE UNBOUNDED SEQUENCE MEMBERS =========================================================
void
ser_add_bool_unbounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name)
{
  (ser->interface->add_bool_unbounded_sequence_member)(ser->impl, builder, id, name);
}


void
ser_add_byte_unbounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name)
{
  (ser->interface->add_byte_unbounded_sequence_member)(ser->impl, builder, id, name);
}


void
ser_add_char_unbounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name)
{
  (ser->interface->add_char_unbounded_sequence_member)(ser->impl, builder, id, name);
}


void
ser_add_float32_unbounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name)
{
  (ser->interface->add_float32_unbounded_sequence_member)(ser->impl, builder, id, name);
}


void
ser_add_float64_unbounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name)
{
  (ser->interface->add_float64_unbounded_sequence_member)(ser->impl, builder, id, name);
}


void
ser_add_int8_unbounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name)
{
  (ser->interface->add_int8_unbounded_sequence_member)(ser->impl, builder, id, name);
}


void
ser_add_uint8_unbounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name)
{
  (ser->interface->add_uint8_unbounded_sequence_member)(ser->impl, builder, id, name);
}


void
ser_add_int16_unbounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name)
{
  (ser->interface->add_int16_unbounded_sequence_member)(ser->impl, builder, id, name);
}


void
ser_add_uint16_unbounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name)
{
  (ser->interface->add_uint16_unbounded_sequence_member)(ser->impl, builder, id, name);
}


void
ser_add_int32_unbounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name)
{
  (ser->interface->add_int32_unbounded_sequence_member)(ser->impl, builder, id, name);
}


void
ser_add_uint32_unbounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name)
{
  (ser->interface->add_uint32_unbounded_sequence_member)(ser->impl, builder, id, name);
}


void
ser_add_int64_unbounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name)
{
  (ser->interface->add_int64_unbounded_sequence_member)(ser->impl, builder, id, name);
}


void
ser_add_uint64_unbounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name)
{
  (ser->interface->add_uint64_unbounded_sequence_member)(ser->impl, builder, id, name);
}


void
ser_add_string_unbounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name)
{
  (ser->interface->add_string_unbounded_sequence_member)(ser->impl, builder, id, name);
}


void
ser_add_wstring_unbounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name)
{
  (ser->interface->add_wstring_unbounded_sequence_member)(ser->impl, builder, id, name);
}


void
ser_add_bounded_string_unbounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t str_bound)
{
  (ser->interface->add_bounded_string_unbounded_sequence_member)(
    ser->impl, builder, id, name, str_bound);
}


void
ser_add_bounded_wstring_unbounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t str_bound)
{
  (ser->interface->add_bounded_wstring_unbounded_sequence_member)(
    ser->impl, builder, id, name, str_bound);
}


// DYNAMIC TYPE BOUNDED SEQUENCE MEMBERS ===========================================================
void
ser_add_bool_bounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound)
{
  (ser->interface->add_bool_bounded_sequence_member)(ser->impl, builder, id, name, bound);
}


void
ser_add_byte_bounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound)
{
  (ser->interface->add_byte_bounded_sequence_member)(ser->impl, builder, id, name, bound);
}


void
ser_add_char_bounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound)
{
  (ser->interface->add_char_bounded_sequence_member)(ser->impl, builder, id, name, bound);
}


void
ser_add_float32_bounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound)
{
  (ser->interface->add_float32_bounded_sequence_member)(ser->impl, builder, id, name, bound);
}


void
ser_add_float64_bounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound)
{
  (ser->interface->add_float64_bounded_sequence_member)(ser->impl, builder, id, name, bound);
}


void
ser_add_int8_bounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound)
{
  (ser->interface->add_int8_bounded_sequence_member)(ser->impl, builder, id, name, bound);
}


void
ser_add_uint8_bounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound)
{
  (ser->interface->add_uint8_bounded_sequence_member)(ser->impl, builder, id, name, bound);
}


void
ser_add_int16_bounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound)
{
  (ser->interface->add_int16_bounded_sequence_member)(ser->impl, builder, id, name, bound);
}


void
ser_add_uint16_bounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound)
{
  (ser->interface->add_uint16_bounded_sequence_member)(ser->impl, builder, id, name, bound);
}


void
ser_add_int32_bounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound)
{
  (ser->interface->add_int32_bounded_sequence_member)(ser->impl, builder, id, name, bound);
}


void
ser_add_uint32_bounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound)
{
  (ser->interface->add_uint32_bounded_sequence_member)(ser->impl, builder, id, name, bound);
}


void
ser_add_int64_bounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound)
{
  (ser->interface->add_int64_bounded_sequence_member)(ser->impl, builder, id, name, bound);
}


void
ser_add_uint64_bounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound)
{
  (ser->interface->add_uint64_bounded_sequence_member)(ser->impl, builder, id, name, bound);
}


void
ser_add_string_bounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound)
{
  (ser->interface->add_string_bounded_sequence_member)(ser->impl, builder, id, name, bound);
}


void
ser_add_wstring_bounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound)
{
  (ser->interface->add_wstring_bounded_sequence_member)(ser->impl, builder, id, name, bound);
}


void
ser_add_bounded_string_bounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t str_bound, uint32_t bound)
{
  (ser->interface->add_bounded_string_bounded_sequence_member)(
    ser->impl, builder, id, name, str_bound, bound);
}


void
ser_add_bounded_wstring_bounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t str_bound, uint32_t bound)
{
  (ser->interface->add_bounded_wstring_bounded_sequence_member)(
    ser->impl, builder, id, name, str_bound, bound);
}


// DYNAMIC TYPE NESTED MEMBERS =====================================================================
void
ser_add_nested_struct_member(
  serialization_support_t * ser, ser_type_builder_t * builder,
  MemberId id, const char * name, ser_dynamic_type_t * nested_struct)
{
  (ser->interface->add_nested_struct_member)(ser->impl, builder, id, name, nested_struct);
}


void
ser_add_nested_struct_static_array_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name,
  ser_dynamic_type_t * nested_struct, uint32_t bound)
{
  (ser->interface->add_nested_struct_static_array_member)(
    ser->impl, builder, id, name, nested_struct, bound);
}


void
ser_add_nested_struct_unbounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name, ser_dynamic_type_t * nested_struct)
{
  (ser->interface->add_nested_struct_unbounded_sequence_member)(
    ser->impl, builder, id, name, nested_struct);
}


void
ser_add_nested_struct_bounded_sequence_member(
  serialization_support_t * ser, ser_type_builder_t * builder, MemberId id, const char * name,
  ser_dynamic_type_t * nested_struct, uint32_t bound)
{
  (ser->interface->add_nested_struct_bounded_sequence_member)(
    ser->impl, builder, id, name, nested_struct, bound);
}

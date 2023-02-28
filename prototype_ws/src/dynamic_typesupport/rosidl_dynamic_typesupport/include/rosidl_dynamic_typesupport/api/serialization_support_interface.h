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

#ifndef ROSIDL_DYNAMIC_TYPESUPPORT__API__SERIALIZATION_INTERFACE_H_
#define ROSIDL_DYNAMIC_TYPESUPPORT__API__SERIALIZATION_INTERFACE_H_

#include <stdint.h>
#include <wchar.h>

#include <rosidl_dynamic_typesupport/types.h>

#ifdef __cplusplus
extern "C" {
#endif


/// This interface must be adopted by all downstream serialization library implementations

// =================================================================================================
// Interface
// =================================================================================================
struct rosidl_dynamic_typesupport_serialization_support_interface_s
{
  /// Interfaces mimicking the XTypes spec (Section 7.5: Language Binding)
  /// https://www.omg.org/spec/DDS-XTypes/1.1/PDF
  ///
  /// Luckily for us, FastRTPS mimics the spec quite well

  /* TODOS (though these are just bonuses...)
   *
   * DynamicType::get_name
   * DynamicType::get_members_count
   * DynamicType::get_type_descriptor / DynamicType::get_descriptor (and the TypeDescriptor class)
   *
   *
   */

  // CORE
  const char * library_identifier;
  void (* serialization_support_impl_handle_fini)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl);


  // ===============================================================================================
  // DYNAMIC TYPE
  // ===============================================================================================

  // DYNAMIC TYPE UTILS
  bool (* dynamic_type_equals)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, const rosidl_dynamic_typesupport_dynamic_type_impl_t * type, const rosidl_dynamic_typesupport_dynamic_type_impl_t * other);
  uint32_t (* get_dynamic_type_member_count)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, const rosidl_dynamic_typesupport_dynamic_type_impl_t * type);  // "member" from XTypes spec


  // DYNAMIC TYPE CONSTRUCTION
  rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * (* struct_type_builder_init)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, const char * name);
  void (* struct_type_builder_fini)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder);
  rosidl_dynamic_typesupport_dynamic_type_impl_t * (* build_struct_type)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder);

  rosidl_dynamic_typesupport_dynamic_type_impl_t * (* dynamic_type_init_from_description)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, type_description_t * description);
  void (* dynamic_type_fini)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_impl_t * type);


  // DYNAMIC TYPE PRIMITIVE MEMBERS
  void (* add_bool_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name);
  void (* add_byte_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name);
  void (* add_char_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name);
  void (* add_float32_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name);
  void (* add_float64_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name);
  void (* add_int8_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name);
  void (* add_uint8_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name);
  void (* add_int16_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name);
  void (* add_uint16_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name);
  void (* add_int32_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name);
  void (* add_uint32_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name);
  void (* add_int64_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name);
  void (* add_uint64_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name);
  void (* add_string_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name);
  void (* add_wstring_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name);
  void (* add_bounded_string_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, uint32_t bound);
  void (* add_bounded_wstring_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, uint32_t bound);


  // DYNAMIC TYPE STATIC ARRAY MEMBERS
  void (* add_bool_array_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, uint32_t bound);
  void (* add_byte_array_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, uint32_t bound);
  void (* add_char_array_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, uint32_t bound);
  void (* add_float32_array_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, uint32_t bound);
  void (* add_float64_array_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, uint32_t bound);
  void (* add_int8_array_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, uint32_t bound);
  void (* add_uint8_array_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, uint32_t bound);
  void (* add_int16_array_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, uint32_t bound);
  void (* add_uint16_array_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, uint32_t bound);
  void (* add_int32_array_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, uint32_t bound);
  void (* add_uint32_array_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, uint32_t bound);
  void (* add_int64_array_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, uint32_t bound);
  void (* add_uint64_array_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, uint32_t bound);
  void (* add_string_array_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, uint32_t bound);
  void (* add_wstring_array_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, uint32_t bound);
  void (* add_bounded_string_array_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, uint32_t str_bound, uint32_t bound);
  void (* add_bounded_wstring_array_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, uint32_t str_bound, uint32_t bound);


  // DYNAMIC TYPE UNBOUNDED SEQUENCE MEMBERS
  void (* add_bool_unbounded_sequence_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name);
  void (* add_byte_unbounded_sequence_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name);
  void (* add_char_unbounded_sequence_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name);
  void (* add_float32_unbounded_sequence_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name);
  void (* add_float64_unbounded_sequence_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name);
  void (* add_int8_unbounded_sequence_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name);
  void (* add_uint8_unbounded_sequence_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name);
  void (* add_int16_unbounded_sequence_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name);
  void (* add_uint16_unbounded_sequence_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name);
  void (* add_int32_unbounded_sequence_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name);
  void (* add_uint32_unbounded_sequence_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name);
  void (* add_int64_unbounded_sequence_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name);
  void (* add_uint64_unbounded_sequence_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name);
  void (* add_string_unbounded_sequence_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name);
  void (* add_wstring_unbounded_sequence_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name);
  void (* add_bounded_string_unbounded_sequence_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, uint32_t str_bound);
  void (* add_bounded_wstring_unbounded_sequence_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, uint32_t str_bound);


  // DYNAMIC TYPE BOUNDED SEQUENCE MEMBERS
  void (* add_bool_bounded_sequence_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, uint32_t bound);
  void (* add_byte_bounded_sequence_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, uint32_t bound);
  void (* add_char_bounded_sequence_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, uint32_t bound);
  void (* add_float32_bounded_sequence_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, uint32_t bound);
  void (* add_float64_bounded_sequence_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, uint32_t bound);
  void (* add_int8_bounded_sequence_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, uint32_t bound);
  void (* add_uint8_bounded_sequence_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, uint32_t bound);
  void (* add_int16_bounded_sequence_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, uint32_t bound);
  void (* add_uint16_bounded_sequence_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, uint32_t bound);
  void (* add_int32_bounded_sequence_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, uint32_t bound);
  void (* add_uint32_bounded_sequence_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, uint32_t bound);
  void (* add_int64_bounded_sequence_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, uint32_t bound);
  void (* add_uint64_bounded_sequence_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, uint32_t bound);
  void (* add_string_bounded_sequence_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, uint32_t bound);
  void (* add_wstring_bounded_sequence_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, uint32_t bound);
  void (* add_bounded_string_bounded_sequence_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, uint32_t str_bound, uint32_t bound);
  void (* add_bounded_wstring_bounded_sequence_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, uint32_t str_bound, uint32_t bound);


  // DYNAMIC TYPE NESTED MEMBERS
  void (* add_nested_struct_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, rosidl_dynamic_typesupport_dynamic_type_impl_t * nested_struct);
  void (* add_nested_struct_array_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, rosidl_dynamic_typesupport_dynamic_type_impl_t * nested_struct, uint32_t bound);
  void (* add_nested_struct_unbounded_sequence_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, rosidl_dynamic_typesupport_dynamic_type_impl_t * nested_struct);
  void (* add_nested_struct_bounded_sequence_member)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder, rosidl_dynamic_typesupport_member_id_t id, const char * name, rosidl_dynamic_typesupport_dynamic_type_impl_t * nested_struct, uint32_t bound);


  // ===============================================================================================
  // DYNAMIC DATA
  // ===============================================================================================

  // DYNAMIC DATA UTILS
  void (* clear_all_values)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data);
  void (* clear_nonkey_values)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data);
  void (* clear_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data, rosidl_dynamic_typesupport_member_id_t id);

  bool (* dynamic_data_equals)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data, const rosidl_dynamic_typesupport_dynamic_data_impl_t * other);

  uint32_t (* get_dynamic_data_item_count)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data);  // "item" from XTypes
  rosidl_dynamic_typesupport_member_id_t (* get_dynamic_data_member_id_by_name)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data, const char * name);
  rosidl_dynamic_typesupport_member_id_t (* get_dynamic_data_member_id_at_index)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data, uint32_t index);
  rosidl_dynamic_typesupport_member_id_t (* get_array_index)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data, uint32_t index);

  rosidl_dynamic_typesupport_dynamic_data_impl_t * (* loan_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data, rosidl_dynamic_typesupport_member_id_t id);
  void (* return_loaned_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data, const rosidl_dynamic_typesupport_dynamic_data_impl_t * inner_data);

  void (* dynamic_data_print)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data);


  // DYNAMIC DATA CONSTRUCTION
  rosidl_dynamic_typesupport_dynamic_data_impl_t * (* dynamic_data_init_from_builder)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * dynamic_type_builder);
  rosidl_dynamic_typesupport_dynamic_data_impl_t * (* dynamic_data_init_from_dynamic_type)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_type_impl_t * type);
  rosidl_dynamic_typesupport_dynamic_data_impl_t * (* dynamic_data_clone)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data);
  void (* dynamic_data_fini)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data);


  // DYNAMIC DATA PRIMITIVE MEMBER GETTERS
  void (* get_bool_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data, bool * value, rosidl_dynamic_typesupport_member_id_t id);
  void (* get_byte_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data, uint8_t * value, rosidl_dynamic_typesupport_member_id_t id);
  void (* get_char_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data, char * value, rosidl_dynamic_typesupport_member_id_t id);
  void (* get_float32_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data, float * value, rosidl_dynamic_typesupport_member_id_t id);
  void (* get_float64_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data, double * value, rosidl_dynamic_typesupport_member_id_t id);
  void (* get_int8_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data, int8_t * value, rosidl_dynamic_typesupport_member_id_t id);
  void (* get_uint8_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data, uint8_t * value, rosidl_dynamic_typesupport_member_id_t id);
  void (* get_int16_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data, int16_t * value, rosidl_dynamic_typesupport_member_id_t id);
  void (* get_uint16_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data, uint16_t * value, rosidl_dynamic_typesupport_member_id_t id);
  void (* get_int32_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data, int32_t * value, rosidl_dynamic_typesupport_member_id_t id);
  void (* get_uint32_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data, uint32_t * value, rosidl_dynamic_typesupport_member_id_t id);
  void (* get_int64_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data, int64_t * value, rosidl_dynamic_typesupport_member_id_t id);
  void (* get_uint64_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data, uint64_t * value, rosidl_dynamic_typesupport_member_id_t id);
  void (* get_string_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data, const char ** value, rosidl_dynamic_typesupport_member_id_t id);
  void (* get_wstring_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data, const wchar_t ** value, rosidl_dynamic_typesupport_member_id_t id);


  // DYNAMIC DATA PRIMITIVE MEMBER SETTERS
  void (* set_bool_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data, bool value, rosidl_dynamic_typesupport_member_id_t id);
  void (* set_byte_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data, uint8_t value, rosidl_dynamic_typesupport_member_id_t id);
  void (* set_char_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data, char value, rosidl_dynamic_typesupport_member_id_t id);
  void (* set_float32_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data, float value, rosidl_dynamic_typesupport_member_id_t id);
  void (* set_float64_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data, double value, rosidl_dynamic_typesupport_member_id_t id);
  void (* set_int8_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data, int8_t value, rosidl_dynamic_typesupport_member_id_t id);
  void (* set_uint8_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data, uint8_t value, rosidl_dynamic_typesupport_member_id_t id);
  void (* set_int16_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data, int16_t value, rosidl_dynamic_typesupport_member_id_t id);
  void (* set_uint16_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data, uint16_t value, rosidl_dynamic_typesupport_member_id_t id);
  void (* set_int32_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data, int32_t value, rosidl_dynamic_typesupport_member_id_t id);
  void (* set_uint32_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data, uint32_t value, rosidl_dynamic_typesupport_member_id_t id);
  void (* set_int64_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data, int64_t value, rosidl_dynamic_typesupport_member_id_t id);
  void (* set_uint64_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data, uint64_t value, rosidl_dynamic_typesupport_member_id_t id);
  void (* set_string_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data, const char * value, rosidl_dynamic_typesupport_member_id_t id);
  void (* set_wstring_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data, const wchar_t * value, rosidl_dynamic_typesupport_member_id_t id);


  // DYNAMIC DATA SEQUENCES
  void (* clear_sequence_data)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data);
  void (* remove_sequence_data)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data, rosidl_dynamic_typesupport_member_id_t id);
  void (* insert_sequence_data)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data, rosidl_dynamic_typesupport_member_id_t * out_id);

  void (* insert_bool_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data, bool value, rosidl_dynamic_typesupport_member_id_t * out_id);
  void (* insert_byte_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data, uint8_t value, rosidl_dynamic_typesupport_member_id_t * out_id);
  void (* insert_char_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data, char value, rosidl_dynamic_typesupport_member_id_t * out_id);
  void (* insert_float32_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data, float value, rosidl_dynamic_typesupport_member_id_t * out_id);
  void (* insert_float64_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data, double value, rosidl_dynamic_typesupport_member_id_t * out_id);
  // void (* insert_int8_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data, int8_t value, rosidl_dynamic_typesupport_member_id_t * out_id);  // TODO
  // void (* insert_uint8_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data, uint8_t value, rosidl_dynamic_typesupport_member_id_t * out_id);  // TODO
  void (* insert_int16_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data, int16_t value, rosidl_dynamic_typesupport_member_id_t * out_id);
  void (* insert_uint16_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data, uint16_t value, rosidl_dynamic_typesupport_member_id_t * out_id);
  void (* insert_int32_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data, int32_t value, rosidl_dynamic_typesupport_member_id_t * out_id);
  void (* insert_uint32_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data, uint32_t value, rosidl_dynamic_typesupport_member_id_t * out_id);
  void (* insert_int64_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data, int64_t value, rosidl_dynamic_typesupport_member_id_t * out_id);
  void (* insert_uint64_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data, uint64_t value, rosidl_dynamic_typesupport_member_id_t * out_id);
  void (* insert_string_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data, const char * value, rosidl_dynamic_typesupport_member_id_t * out_id);
  void (* insert_wstring_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data, const wchar_t * value, rosidl_dynamic_typesupport_member_id_t * out_id);


  // DYNAMIC DATA NESTED
  void (* get_complex_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data, rosidl_dynamic_typesupport_dynamic_data_impl_t ** value, rosidl_dynamic_typesupport_member_id_t id);  // Copies
  void (* set_complex_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data, rosidl_dynamic_typesupport_dynamic_data_impl_t * value, rosidl_dynamic_typesupport_member_id_t id);

  void (* insert_const_complex_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data, const rosidl_dynamic_typesupport_dynamic_data_impl_t * value, rosidl_dynamic_typesupport_member_id_t * out_id);
  void (* insert_complex_value)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data, rosidl_dynamic_typesupport_dynamic_data_impl_t * value, rosidl_dynamic_typesupport_member_id_t * out_id);
  void (* insert_complex_value_ptr)(rosidl_dynamic_typesupport_serialization_support_impl_t * impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data, rosidl_dynamic_typesupport_dynamic_data_impl_t * value, rosidl_dynamic_typesupport_member_id_t * out_id);

  // TODO:
  // Getting something that can be passed to pubsub
};


#ifdef __cplusplus
}
#endif

#endif  // ROSIDL_DYNAMIC_TYPESUPPORT__API__SERIALIZATION_INTERFACE_H_

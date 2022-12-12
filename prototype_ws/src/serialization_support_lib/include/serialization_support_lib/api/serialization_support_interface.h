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

#ifndef SERIALIZATION_SUPPORT_LIB__API__SERIALIZATION_INTERFACE_H_
#define SERIALIZATION_SUPPORT_LIB__API__SERIALIZATION_INTERFACE_H_

#include <stdint.h>
#include <wchar.h>

#include <serialization_support_lib/types.h>

#ifdef __cplusplus
extern "C" {
#endif

/// This interface must be adopted by all downstream serialization library implementations

// TODO(methylDragon): Potential improvement

// =================================================================================================
// void * Containers
// =================================================================================================
/// All these are meant to store library specific objects (hence why they contain void *)

/// For anything necessary or useful for the operation of the serialization lib
/// (e.g. singleton dynamic type and dynamic data factories)
typedef struct ser_impl_s
{
  void * impl;
} ser_impl_t;

inline
ser_impl_t ser_get_zero_initialized_impl() { return (ser_impl_t){0}; }


/// For type builders
typedef struct ser_type_builder_s
{
  void * impl;
} ser_type_builder_t;

inline
ser_type_builder_t ser_get_zero_initialized_type_builder() { return (ser_type_builder_t){0}; }


/// For dynamic type
typedef struct ser_dynamic_type_s
{
  void * impl;
} ser_dynamic_type_t;

inline
ser_dynamic_type_t ser_get_zero_initialized_dynamic_type() { return (ser_dynamic_type_t){0}; }


/// For dynamic data
typedef struct ser_dynamic_data_s
{
  void * impl;
} ser_dynamic_data_t;

inline
ser_dynamic_data_t ser_get_zero_initialized_dynamic_data() { return (ser_dynamic_data_t){0}; }


// =================================================================================================
// Interface
// =================================================================================================
typedef struct serialization_support_interface_s
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
  void (* ser_support_impl_fini)(ser_impl_t * impl);


  // ===============================================================================================
  // DYNAMIC TYPE
  // ===============================================================================================

  // DYNAMIC TYPE UTILS
  bool (* type_equals)(
    ser_impl_t * impl, const ser_dynamic_type_t * type, const ser_dynamic_type_t * other);
  uint32_t (* get_type_member_count)(ser_impl_t * impl, const ser_dynamic_type_t * type);  // "member" from XTypes


  // DYNAMIC TYPE CONSTRUCTION
  ser_type_builder_t * (*struct_type_builder_init)(ser_impl_t * impl, const char * name);
  void (* struct_type_builder_fini)(ser_impl_t * impl, ser_type_builder_t * builder);

  ser_dynamic_type_t * (*build_struct_type)(ser_impl_t * impl, ser_type_builder_t * builder);
  ser_dynamic_type_t * (*construct_type_from_description)(
    ser_impl_t * impl, type_description_t * description);
  void (* type_fini)(ser_impl_t * impl, ser_dynamic_type_t * type);


  // DYNAMIC TYPE PRIMITIVE MEMBERS
  void (* add_bool_member)(ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name);
  void (* add_byte_member)(ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name);
  void (* add_char_member)(ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name);
  void (* add_float32_member)(ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name);
  void (* add_float64_member)(ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name);
  void (* add_int8_member)(ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name);
  void (* add_uint8_member)(ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name);
  void (* add_int16_member)(ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name);
  void (* add_uint16_member)(ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name);
  void (* add_int32_member)(ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name);
  void (* add_uint32_member)(ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name);
  void (* add_int64_member)(ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name);
  void (* add_uint64_member)(ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name);
  void (* add_string_member)(ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name);
  void (* add_wstring_member)(ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name);
  void (* add_bounded_string_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);
  void (* add_bounded_wstring_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);


  // DYNAMIC TYPE STATIC ARRAY MEMBERS
  void (* add_bool_static_array_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);
  void (* add_byte_static_array_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);
  void (* add_char_static_array_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);
  void (* add_float32_static_array_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);
  void (* add_float64_static_array_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);
  void (* add_int8_static_array_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);
  void (* add_uint8_static_array_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);
  void (* add_int16_static_array_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);
  void (* add_uint16_static_array_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);
  void (* add_int32_static_array_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);
  void (* add_uint32_static_array_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);
  void (* add_int64_static_array_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);
  void (* add_uint64_static_array_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);
  void (* add_string_static_array_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);
  void (* add_wstring_static_array_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);
  void (* add_bounded_string_static_array_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t str_bound,
    uint32_t bound);
  void (* add_bounded_wstring_static_array_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t str_bound,
    uint32_t bound);


  // DYNAMIC TYPE UNBOUNDED SEQUENCE MEMBERS
  void (* add_bool_unbounded_sequence_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name);
  void (* add_byte_unbounded_sequence_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name);
  void (* add_char_unbounded_sequence_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name);
  void (* add_float32_unbounded_sequence_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name);
  void (* add_float64_unbounded_sequence_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name);
  void (* add_int8_unbounded_sequence_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name);
  void (* add_uint8_unbounded_sequence_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name);
  void (* add_int16_unbounded_sequence_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name);
  void (* add_uint16_unbounded_sequence_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name);
  void (* add_int32_unbounded_sequence_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name);
  void (* add_uint32_unbounded_sequence_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name);
  void (* add_int64_unbounded_sequence_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name);
  void (* add_uint64_unbounded_sequence_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name);
  void (* add_string_unbounded_sequence_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name);
  void (* add_wstring_unbounded_sequence_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name);
  void (* add_bounded_string_unbounded_sequence_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t str_bound);
  void (* add_bounded_wstring_unbounded_sequence_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t str_bound);


  // DYNAMIC TYPE BOUNDED SEQUENCE MEMBERS
  void (* add_bool_bounded_sequence_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);
  void (* add_byte_bounded_sequence_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);
  void (* add_char_bounded_sequence_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);
  void (* add_float32_bounded_sequence_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);
  void (* add_float64_bounded_sequence_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);
  void (* add_int8_bounded_sequence_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);
  void (* add_uint8_bounded_sequence_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);
  void (* add_int16_bounded_sequence_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);
  void (* add_uint16_bounded_sequence_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);
  void (* add_int32_bounded_sequence_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);
  void (* add_uint32_bounded_sequence_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);
  void (* add_int64_bounded_sequence_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);
  void (* add_uint64_bounded_sequence_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);
  void (* add_string_bounded_sequence_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);
  void (* add_wstring_bounded_sequence_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t bound);
  void (* add_bounded_string_bounded_sequence_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t str_bound,
    uint32_t bound);
  void (* add_bounded_wstring_bounded_sequence_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, uint32_t str_bound,
    uint32_t bound);


  // DYNAMIC TYPE NESTED MEMBERS
  void (* add_nested_struct_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, ser_dynamic_type_t * nested_struct);
  void (* add_nested_struct_static_array_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, ser_dynamic_type_t * nested_struct,
    uint32_t bound);
  void (* add_nested_struct_unbounded_sequence_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, ser_dynamic_type_t * nested_struct);
  void (* add_nested_struct_bounded_sequence_member)(
    ser_impl_t * impl, ser_type_builder_t * builder, MemberId id, const char * name, ser_dynamic_type_t * nested_struct,
    uint32_t bound);


  // ===============================================================================================
  // DYNAMIC DATA
  // ===============================================================================================

  // DYNAMIC DATA UTILS
  void (* clear_all_values)(ser_impl_t * impl, ser_dynamic_data_t * data);
  void (* clear_nonkey_values)(ser_impl_t * impl, ser_dynamic_data_t * data);
  void (* clear_value)(ser_impl_t * impl, ser_dynamic_data_t * data, MemberId id);

  bool (* data_equals)(ser_impl_t * impl, const ser_dynamic_data_t * data, const ser_dynamic_data_t * other);

  uint32_t (* get_data_item_count)(ser_impl_t * impl, const ser_dynamic_data_t * data);  // "item" from XTypes
  MemberId (* get_data_member_id_by_name)(ser_impl_t * impl, const ser_dynamic_data_t * data, const char * name);
  MemberId (* get_data_member_id_at_index)(ser_impl_t * impl, const ser_dynamic_data_t * data, uint32_t index);
  MemberId (* get_array_index)(ser_impl_t * impl, ser_dynamic_data_t * data, uint32_t index);

  ser_dynamic_data_t * (* loan_value)(ser_impl_t * impl, ser_dynamic_data_t * data, MemberId id);
  void (* return_loaned_value)(ser_impl_t * impl, ser_dynamic_data_t * data, const ser_dynamic_data_t * inner_data);

  void (* print_dynamic_data)(ser_impl_t * impl, ser_dynamic_data_t * data);


  // DYNAMIC DATA CONSTRUCTION
  ser_dynamic_data_t * (* data_init_from_builder)(ser_impl_t * impl, ser_type_builder_t * builder);
  ser_dynamic_data_t * (* data_init_from_type)(ser_impl_t * impl, ser_dynamic_type_t * type);
  ser_dynamic_data_t * (* data_clone)(ser_impl_t * impl, const ser_dynamic_data_t * data);
  void (* data_fini)(ser_impl_t * impl, ser_dynamic_data_t * data);


  // DYNAMIC DATA PRIMITIVE MEMBER GETTERS
  void (* get_bool_value)(ser_impl_t * impl, const ser_dynamic_data_t * data, bool * value, MemberId id);
  void (* get_byte_value)(ser_impl_t * impl, const ser_dynamic_data_t * data, uint8_t * value, MemberId id);
  void (* get_char_value)(ser_impl_t * impl, const ser_dynamic_data_t * data, char * value, MemberId id);
  void (* get_float32_value)(ser_impl_t * impl, const ser_dynamic_data_t * data, float * value, MemberId id);
  void (* get_float64_value)(ser_impl_t * impl, const ser_dynamic_data_t * data, double * value, MemberId id);
  void (* get_int8_value)(ser_impl_t * impl, const ser_dynamic_data_t * data, int8_t * value, MemberId id);
  void (* get_uint8_value)(ser_impl_t * impl, const ser_dynamic_data_t * data, uint8_t * value, MemberId id);
  void (* get_int16_value)(ser_impl_t * impl, const ser_dynamic_data_t * data, int16_t * value, MemberId id);
  void (* get_uint16_value)(ser_impl_t * impl, const ser_dynamic_data_t * data, uint16_t * value, MemberId id);
  void (* get_int32_value)(ser_impl_t * impl, const ser_dynamic_data_t * data, int32_t * value, MemberId id);
  void (* get_uint32_value)(ser_impl_t * impl, const ser_dynamic_data_t * data, uint32_t * value, MemberId id);
  void (* get_int64_value)(ser_impl_t * impl, const ser_dynamic_data_t * data, int64_t * value, MemberId id);
  void (* get_uint64_value)(ser_impl_t * impl, const ser_dynamic_data_t * data, uint64_t * value, MemberId id);
  void (* get_string_value)(ser_impl_t * impl, const ser_dynamic_data_t * data, const char ** value, MemberId id);
  void (* get_wstring_value)(
    ser_impl_t * impl, const ser_dynamic_data_t * data, const wchar_t ** value, MemberId id);


  // DYNAMIC DATA PRIMITIVE MEMBER SETTERS
  void (* set_bool_value)(ser_impl_t * impl, ser_dynamic_data_t * data, bool value, MemberId id);
  void (* set_byte_value)(ser_impl_t * impl, ser_dynamic_data_t * data, uint8_t value, MemberId id);
  void (* set_char_value)(ser_impl_t * impl, ser_dynamic_data_t * data, char value, MemberId id);
  void (* set_float32_value)(ser_impl_t * impl, ser_dynamic_data_t * data, float value, MemberId id);
  void (* set_float64_value)(ser_impl_t * impl, ser_dynamic_data_t * data, double value, MemberId id);
  void (* set_int8_value)(ser_impl_t * impl, ser_dynamic_data_t * data, int8_t value, MemberId id);
  void (* set_uint8_value)(ser_impl_t * impl, ser_dynamic_data_t * data, uint8_t value, MemberId id);
  void (* set_int16_value)(ser_impl_t * impl, ser_dynamic_data_t * data, int16_t value, MemberId id);
  void (* set_uint16_value)(ser_impl_t * impl, ser_dynamic_data_t * data, uint16_t value, MemberId id);
  void (* set_int32_value)(ser_impl_t * impl, ser_dynamic_data_t * data, int32_t value, MemberId id);
  void (* set_uint32_value)(ser_impl_t * impl, ser_dynamic_data_t * data, uint32_t value, MemberId id);
  void (* set_int64_value)(ser_impl_t * impl, ser_dynamic_data_t * data, int64_t value, MemberId id);
  void (* set_uint64_value)(ser_impl_t * impl, ser_dynamic_data_t * data, uint64_t value, MemberId id);
  void (* set_string_value)(ser_impl_t * impl, ser_dynamic_data_t * data, const char * value, MemberId id);
  void (* set_wstring_value)(ser_impl_t * impl, ser_dynamic_data_t * data, const wchar_t * value, MemberId id);


  // DYNAMIC DATA SEQUENCES
  void (* clear_sequence_data)(ser_impl_t * impl, ser_dynamic_data_t * data);
  void (* remove_sequence_data)(ser_impl_t * impl, ser_dynamic_data_t * data, MemberId id);
  void (* insert_sequence_data)(ser_impl_t * impl, ser_dynamic_data_t * data, MemberId * out_id);

  void (* insert_bool_value)(ser_impl_t * impl, ser_dynamic_data_t * data, bool value, MemberId * out_id);
  void (* insert_byte_value)(ser_impl_t * impl, ser_dynamic_data_t * data, uint8_t value, MemberId * out_id);
  void (* insert_char_value)(ser_impl_t * impl, ser_dynamic_data_t * data, char value, MemberId * out_id);
  void (* insert_float32_value)(ser_impl_t * impl, ser_dynamic_data_t * data, float value, MemberId * out_id);
  void (* insert_float64_value)(ser_impl_t * impl, ser_dynamic_data_t * data, double value, MemberId * out_id);
  // void (* insert_int8_value)(ser_impl_t * impl, ser_dynamic_data_t * data, int8_t value, MemberId * out_id);
  // void (* insert_uint8_value)(ser_impl_t * impl, ser_dynamic_data_t * data, uint8_t value, MemberId * out_id);
  void (* insert_int16_value)(ser_impl_t * impl, ser_dynamic_data_t * data, int16_t value, MemberId * out_id);
  void (* insert_uint16_value)(ser_impl_t * impl, ser_dynamic_data_t * data, uint16_t value, MemberId * out_id);
  void (* insert_int32_value)(ser_impl_t * impl, ser_dynamic_data_t * data, int32_t value, MemberId * out_id);
  void (* insert_uint32_value)(ser_impl_t * impl, ser_dynamic_data_t * data, uint32_t value, MemberId * out_id);
  void (* insert_int64_value)(ser_impl_t * impl, ser_dynamic_data_t * data, int64_t value, MemberId * out_id);
  void (* insert_uint64_value)(ser_impl_t * impl, ser_dynamic_data_t * data, uint64_t value, MemberId * out_id);
  void (* insert_string_value)(ser_impl_t * impl, ser_dynamic_data_t * data, const char * value, MemberId * out_id);
  void (* insert_wstring_value)(
    ser_impl_t * impl, ser_dynamic_data_t * data, const wchar_t * value, MemberId * out_id);


  // DYNAMIC DATA NESTED
  void (* get_complex_value)(ser_impl_t * impl, const ser_dynamic_data_t * data, ser_dynamic_data_t ** value, MemberId id);  // Copies
  void (* set_complex_value)(ser_impl_t * impl, ser_dynamic_data_t * data, ser_dynamic_data_t * value, MemberId id);

  void (* insert_const_complex_value)(
    ser_impl_t * impl, ser_dynamic_data_t * data, const ser_dynamic_data_t * value, MemberId * out_id);
  void (* insert_complex_value)(
    ser_impl_t * impl, ser_dynamic_data_t * data, ser_dynamic_data_t * value, MemberId * out_id);
  void (* insert_complex_value_ptr)(
    ser_impl_t * impl, ser_dynamic_data_t * data, ser_dynamic_data_t * value, MemberId * out_id);

  // TODO:
  // Getting something that can be passed to pubsub
} serialization_support_interface_t;


typedef struct serialization_support_s
{
  ser_impl_t * impl;
  const serialization_support_interface_t * interface;
} serialization_support_t;


#ifdef __cplusplus
}
#endif

#endif  // SERIALIZATION_SUPPORT_LIB__API__SERIALIZATION_INTERFACE_H_

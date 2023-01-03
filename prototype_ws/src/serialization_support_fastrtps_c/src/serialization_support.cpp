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

#include <serialization_support_lib/api/serialization_support_interface.h>
#include <serialization_support_lib/types.h>

#include <serialization_support_fastrtps_c/dynamic_data.h>
#include <serialization_support_fastrtps_c/dynamic_type.h>
#include <serialization_support_fastrtps_c/identifier.h>
#include <serialization_support_fastrtps_c/serialization_support.h>
#include <serialization_support_fastrtps_c/serialization_impl.h>


// CORE ============================================================================================
serialization_support_interface_t *
create_fastrtps_ser_interface()
{
  serialization_support_interface_t * fastrtps_ser_interface =
    (serialization_support_interface_t *) calloc(1, sizeof(serialization_support_interface_t));


  // CORE ==========================================================================================
  fastrtps_ser_interface->library_identifier = fastrtps_serialization_support_library_identifier;

  fastrtps_ser_interface->ser_impl_fini =
    (void (*)(ser_impl_t *))
    fastrtps__ser_impl_fini;


  // ===============================================================================================
  // DYNAMIC TYPE METHODS
  // ===============================================================================================
  // DYNAMIC TYPE UTILS
  fastrtps_ser_interface->type_equals =
    (bool (*)(ser_impl_t *, const ser_dynamic_type_t *, const ser_dynamic_type_t *))
    fastrtps__type_equals;

  fastrtps_ser_interface->get_type_member_count =
    (uint32_t (*)(ser_impl_t *, const ser_dynamic_type_t *))
    fastrtps__get_type_member_count;


  // DYNAMIC TYPE CONSTRUCTION
  fastrtps_ser_interface->struct_type_builder_init =
    (ser_type_builder_t * (*)(ser_impl_t *, const char *))
    fastrtps__struct_type_builder_init;

  fastrtps_ser_interface->struct_type_builder_fini =
    (void (*)(ser_impl_t *, ser_type_builder_t *))
    fastrtps__struct_type_builder_fini;

  fastrtps_ser_interface->build_struct_type =
    (ser_dynamic_type_t * (*)(ser_impl_t *, ser_type_builder_t *))
    fastrtps__build_struct_type;

  fastrtps_ser_interface->construct_type_from_description =
    (ser_dynamic_type_t * (*)(ser_impl_t *, type_description_t *))
    fastrtps__construct_type_from_description;

  fastrtps_ser_interface->type_fini =
    (void (*)(ser_impl_t *, ser_dynamic_type_t *))
    fastrtps__type_fini;


  // DYNAMIC TYPE PRIMITIVE MEMBERS
  fastrtps_ser_interface->add_bool_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *))
    fastrtps__add_bool_member;

  fastrtps_ser_interface->add_byte_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *))
    fastrtps__add_byte_member;

  fastrtps_ser_interface->add_char_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *))
    fastrtps__add_char_member;

  fastrtps_ser_interface->add_float32_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *))
    fastrtps__add_float32_member;

  fastrtps_ser_interface->add_float64_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *))
    fastrtps__add_float64_member;

  fastrtps_ser_interface->add_int8_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *))
    fastrtps__add_int8_member;

  fastrtps_ser_interface->add_uint8_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *))
    fastrtps__add_uint8_member;

  fastrtps_ser_interface->add_int16_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *))
    fastrtps__add_int16_member;

  fastrtps_ser_interface->add_uint16_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *))
    fastrtps__add_uint16_member;

  fastrtps_ser_interface->add_int32_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *))
    fastrtps__add_int32_member;

  fastrtps_ser_interface->add_uint32_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *))
    fastrtps__add_uint32_member;

  fastrtps_ser_interface->add_int64_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *))
    fastrtps__add_int64_member;

  fastrtps_ser_interface->add_uint64_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *))
    fastrtps__add_uint64_member;

  fastrtps_ser_interface->add_string_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *))
    fastrtps__add_string_member;

  fastrtps_ser_interface->add_wstring_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *))
    fastrtps__add_wstring_member;

  fastrtps_ser_interface->add_bounded_string_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, uint32_t))
    fastrtps__add_bounded_string_member;

  fastrtps_ser_interface->add_bounded_wstring_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, uint32_t))
    fastrtps__add_bounded_wstring_member;


  // DYNAMIC TYPE STATIC ARRAY MEMBERS
  fastrtps_ser_interface->add_bool_static_array_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, uint32_t))
    fastrtps__add_bool_static_array_member;
  fastrtps_ser_interface->add_byte_static_array_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, uint32_t))
    fastrtps__add_byte_static_array_member;
  fastrtps_ser_interface->add_char_static_array_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, uint32_t))
    fastrtps__add_char_static_array_member;

  fastrtps_ser_interface->add_float32_static_array_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, uint32_t))
    fastrtps__add_float32_static_array_member;

  fastrtps_ser_interface->add_float64_static_array_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, uint32_t))
    fastrtps__add_float64_static_array_member;

  fastrtps_ser_interface->add_int8_static_array_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, uint32_t))
    fastrtps__add_int8_static_array_member;

  fastrtps_ser_interface->add_uint8_static_array_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, uint32_t))
    fastrtps__add_uint8_static_array_member;

  fastrtps_ser_interface->add_int16_static_array_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, uint32_t))
    fastrtps__add_int16_static_array_member;

  fastrtps_ser_interface->add_uint16_static_array_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, uint32_t))
    fastrtps__add_uint16_static_array_member;

  fastrtps_ser_interface->add_int32_static_array_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, uint32_t))
    fastrtps__add_int32_static_array_member;

  fastrtps_ser_interface->add_uint32_static_array_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, uint32_t))
    fastrtps__add_uint32_static_array_member;

  fastrtps_ser_interface->add_int64_static_array_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, uint32_t))
    fastrtps__add_int64_static_array_member;

  fastrtps_ser_interface->add_uint64_static_array_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, uint32_t))
    fastrtps__add_uint64_static_array_member;

  fastrtps_ser_interface->add_string_static_array_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, uint32_t))
    fastrtps__add_string_static_array_member;

  fastrtps_ser_interface->add_wstring_static_array_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, uint32_t))
    fastrtps__add_wstring_static_array_member;

  fastrtps_ser_interface->add_bounded_string_static_array_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, uint32_t, uint32_t))
    fastrtps__add_bounded_string_static_array_member;

  fastrtps_ser_interface->add_bounded_wstring_static_array_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, uint32_t, uint32_t))
    fastrtps__add_bounded_wstring_static_array_member;


  // DYNAMIC TYPE UNBOUNDED SEQUENCE MEMBERS
  fastrtps_ser_interface->add_bool_unbounded_sequence_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *))
    fastrtps__add_bool_unbounded_sequence_member;

  fastrtps_ser_interface->add_byte_unbounded_sequence_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *))
    fastrtps__add_byte_unbounded_sequence_member;

  fastrtps_ser_interface->add_char_unbounded_sequence_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *))
    fastrtps__add_char_unbounded_sequence_member;

  fastrtps_ser_interface->add_float32_unbounded_sequence_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *))
    fastrtps__add_float32_unbounded_sequence_member;

  fastrtps_ser_interface->add_float64_unbounded_sequence_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *))
    fastrtps__add_float64_unbounded_sequence_member;

  fastrtps_ser_interface->add_int8_unbounded_sequence_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *))
    fastrtps__add_int8_unbounded_sequence_member;

  fastrtps_ser_interface->add_uint8_unbounded_sequence_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *))
    fastrtps__add_uint8_unbounded_sequence_member;

  fastrtps_ser_interface->add_int16_unbounded_sequence_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *))
    fastrtps__add_int16_unbounded_sequence_member;

  fastrtps_ser_interface->add_uint16_unbounded_sequence_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *))
    fastrtps__add_uint16_unbounded_sequence_member;

  fastrtps_ser_interface->add_int32_unbounded_sequence_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *))
    fastrtps__add_int32_unbounded_sequence_member;

  fastrtps_ser_interface->add_uint32_unbounded_sequence_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *))
    fastrtps__add_uint32_unbounded_sequence_member;

  fastrtps_ser_interface->add_int64_unbounded_sequence_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *))
    fastrtps__add_int64_unbounded_sequence_member;

  fastrtps_ser_interface->add_uint64_unbounded_sequence_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *))
    fastrtps__add_uint64_unbounded_sequence_member;

  fastrtps_ser_interface->add_string_unbounded_sequence_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *))
    fastrtps__add_string_unbounded_sequence_member;

  fastrtps_ser_interface->add_wstring_unbounded_sequence_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *))
    fastrtps__add_wstring_unbounded_sequence_member;

  fastrtps_ser_interface->add_bounded_string_unbounded_sequence_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, uint32_t))
    fastrtps__add_bounded_string_unbounded_sequence_member;

  fastrtps_ser_interface->add_bounded_wstring_unbounded_sequence_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, uint32_t))
    fastrtps__add_bounded_wstring_unbounded_sequence_member;


  // DYNAMIC TYPE BOUNDED SEQUENCE MEMBERS
  fastrtps_ser_interface->add_bool_bounded_sequence_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, uint32_t))
    fastrtps__add_bool_bounded_sequence_member;

  fastrtps_ser_interface->add_byte_bounded_sequence_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, uint32_t))
    fastrtps__add_byte_bounded_sequence_member;

  fastrtps_ser_interface->add_char_bounded_sequence_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, uint32_t))
    fastrtps__add_char_bounded_sequence_member;

  fastrtps_ser_interface->add_float32_bounded_sequence_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, uint32_t))
    fastrtps__add_float32_bounded_sequence_member;

  fastrtps_ser_interface->add_float64_bounded_sequence_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, uint32_t))
    fastrtps__add_float64_bounded_sequence_member;

  fastrtps_ser_interface->add_int8_bounded_sequence_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, uint32_t))
    fastrtps__add_int8_bounded_sequence_member;

  fastrtps_ser_interface->add_uint8_bounded_sequence_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, uint32_t))
    fastrtps__add_uint8_bounded_sequence_member;

  fastrtps_ser_interface->add_int16_bounded_sequence_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, uint32_t))
    fastrtps__add_int16_bounded_sequence_member;

  fastrtps_ser_interface->add_uint16_bounded_sequence_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, uint32_t))
    fastrtps__add_uint16_bounded_sequence_member;

  fastrtps_ser_interface->add_int32_bounded_sequence_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, uint32_t))
    fastrtps__add_int32_bounded_sequence_member;

  fastrtps_ser_interface->add_uint32_bounded_sequence_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, uint32_t))
    fastrtps__add_uint32_bounded_sequence_member;

  fastrtps_ser_interface->add_int64_bounded_sequence_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, uint32_t))
    fastrtps__add_int64_bounded_sequence_member;

  fastrtps_ser_interface->add_uint64_bounded_sequence_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, uint32_t))
    fastrtps__add_uint64_bounded_sequence_member;

  fastrtps_ser_interface->add_string_bounded_sequence_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, uint32_t))
    fastrtps__add_string_bounded_sequence_member;

  fastrtps_ser_interface->add_wstring_bounded_sequence_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, uint32_t))
    fastrtps__add_wstring_bounded_sequence_member;

  fastrtps_ser_interface->add_bounded_string_bounded_sequence_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, uint32_t, uint32_t))
    fastrtps__add_bounded_string_bounded_sequence_member;

  fastrtps_ser_interface->add_bounded_wstring_bounded_sequence_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, uint32_t, uint32_t))
    fastrtps__add_bounded_wstring_bounded_sequence_member;


  // DYNAMIC TYPE NESTED MEMBERS
  fastrtps_ser_interface->add_nested_struct_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, ser_dynamic_type_t *))
    fastrtps__add_nested_struct_member;

  fastrtps_ser_interface->add_nested_struct_static_array_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, ser_dynamic_type_t *, uint32_t))
    fastrtps__add_nested_struct_static_array_member;

  fastrtps_ser_interface->add_nested_struct_unbounded_sequence_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, ser_dynamic_type_t *))
    fastrtps__add_nested_struct_unbounded_sequence_member;

  fastrtps_ser_interface->add_nested_struct_bounded_sequence_member =
    (void (*)(ser_impl_t *, ser_type_builder_t *, MemberId, const char *, ser_dynamic_type_t *, uint32_t))
    fastrtps__add_nested_struct_bounded_sequence_member;

  // ===============================================================================================
  // DYNAMIC DATA METHODS
  // ===============================================================================================
  // DYNAMIC DATA UTILS
  fastrtps_ser_interface->clear_all_values =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *))
    fastrtps__clear_all_values;

  fastrtps_ser_interface->clear_nonkey_values =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *))
    fastrtps__clear_nonkey_values;

  fastrtps_ser_interface->clear_sequence_data =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *))
    fastrtps__clear_sequence_data;

  fastrtps_ser_interface->clear_value =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *, MemberId))
    fastrtps__clear_value;

  fastrtps_ser_interface->data_equals =
    (bool (*)(ser_impl_t *, const ser_dynamic_data_t *, const ser_dynamic_data_t *))
    fastrtps__data_equals;

  fastrtps_ser_interface->get_data_item_count =
    (uint32_t (*)(ser_impl_t *, const ser_dynamic_data_t *))
    fastrtps__get_data_item_count;

  fastrtps_ser_interface->get_data_member_id_by_name =
    (MemberId (*)(ser_impl_t *, const ser_dynamic_data_t *, const char *))
    fastrtps__get_data_member_id_by_name;

  fastrtps_ser_interface->get_data_member_id_at_index =
    (MemberId (*)(ser_impl_t *, const ser_dynamic_data_t *, uint32_t))
    fastrtps__get_data_member_id_at_index;

  fastrtps_ser_interface->get_array_index =
    (MemberId (*)(ser_impl_t *, ser_dynamic_data_t *, uint32_t))
    fastrtps__get_array_index;

  fastrtps_ser_interface->loan_value =
    (ser_dynamic_data_t * (*)(ser_impl_t *, ser_dynamic_data_t *, MemberId))
    fastrtps__loan_value;

  fastrtps_ser_interface->return_loaned_value =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *, const ser_dynamic_data_t *))
    fastrtps__return_loaned_value;

  fastrtps_ser_interface->print_dynamic_data =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *))
    fastrtps__print_dynamic_data;


  // DYNAMIC DATA CONSTRUCTION
  fastrtps_ser_interface->data_init_from_builder =
    (ser_dynamic_data_t * (*)(ser_impl_t *, ser_type_builder_t *))
    fastrtps__data_init_from_builder;

  fastrtps_ser_interface->data_init_from_type =
    (ser_dynamic_data_t * (*)(ser_impl_t *, ser_dynamic_type_t *))
    fastrtps__data_init_from_type;

  fastrtps_ser_interface->data_clone =
    (ser_dynamic_data_t * (*)(ser_impl_t *, const ser_dynamic_data_t *))
    fastrtps__data_clone;

  fastrtps_ser_interface->data_fini =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *))
    fastrtps__data_fini;


  // DYNAMIC DATA PRIMITIVE MEMBER GETTERS
  fastrtps_ser_interface->get_bool_value =
    (void (*)(ser_impl_t *, const ser_dynamic_data_t *, bool *, MemberId))
    fastrtps__get_bool_value;

  fastrtps_ser_interface->get_byte_value =
    (void (*)(ser_impl_t *, const ser_dynamic_data_t *, uint8_t *, MemberId))
    fastrtps__get_byte_value;

  fastrtps_ser_interface->get_char_value =
    (void (*)(ser_impl_t *, const ser_dynamic_data_t *, char *, MemberId))
    fastrtps__get_char_value;

  fastrtps_ser_interface->get_float32_value =
    (void (*)(ser_impl_t *, const ser_dynamic_data_t *, float *, MemberId))
    fastrtps__get_float32_value;

  fastrtps_ser_interface->get_float64_value =
    (void (*)(ser_impl_t *, const ser_dynamic_data_t *, double *, MemberId))
    fastrtps__get_float64_value;

  fastrtps_ser_interface->get_int8_value =
    (void (*)(ser_impl_t *, const ser_dynamic_data_t *, int8_t *, MemberId))
    fastrtps__get_int8_value;

  fastrtps_ser_interface->get_uint8_value =
    (void (*)(ser_impl_t *, const ser_dynamic_data_t *, uint8_t *, MemberId))
    fastrtps__get_uint8_value;

  fastrtps_ser_interface->get_int16_value =
    (void (*)(ser_impl_t *, const ser_dynamic_data_t *, int16_t *, MemberId))
    fastrtps__get_int16_value;

  fastrtps_ser_interface->get_uint16_value =
    (void (*)(ser_impl_t *, const ser_dynamic_data_t *, uint16_t *, MemberId))
    fastrtps__get_uint16_value;

  fastrtps_ser_interface->get_int32_value =
    (void (*)(ser_impl_t *, const ser_dynamic_data_t *, int32_t *, MemberId))
    fastrtps__get_int32_value;

  fastrtps_ser_interface->get_uint32_value =
    (void (*)(ser_impl_t *, const ser_dynamic_data_t *, uint32_t *, MemberId))
    fastrtps__get_uint32_value;

  fastrtps_ser_interface->get_int64_value =
    (void (*)(ser_impl_t *, const ser_dynamic_data_t *, int64_t *, MemberId))
    fastrtps__get_int64_value;

  fastrtps_ser_interface->get_uint64_value =
    (void (*)(ser_impl_t *, const ser_dynamic_data_t *, uint64_t *, MemberId))
    fastrtps__get_uint64_value;

  fastrtps_ser_interface->get_string_value =
    (void (*)(ser_impl_t *, const ser_dynamic_data_t *, const char **, MemberId))
    fastrtps__get_string_value;

  fastrtps_ser_interface->get_wstring_value =
    (void (*)(ser_impl_t *, const ser_dynamic_data_t *, const wchar_t **, MemberId))
    fastrtps__get_wstring_value;


  // DYNAMIC DATA PRIMITIVE MEMBER SETTERS
  fastrtps_ser_interface->set_bool_value =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *, bool, MemberId))
    fastrtps__set_bool_value;

  fastrtps_ser_interface->set_byte_value =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *, uint8_t, MemberId))
    fastrtps__set_byte_value;

  fastrtps_ser_interface->set_char_value =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *, char, MemberId))
    fastrtps__set_char_value;

  fastrtps_ser_interface->set_float32_value =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *, float, MemberId))
    fastrtps__set_float32_value;

  fastrtps_ser_interface->set_float64_value =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *, double, MemberId))
    fastrtps__set_float64_value;

  fastrtps_ser_interface->set_int8_value =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *, int8_t, MemberId))
    fastrtps__set_int8_value;

  fastrtps_ser_interface->set_uint8_value =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *, uint8_t, MemberId))
    fastrtps__set_uint8_value;

  fastrtps_ser_interface->set_int16_value =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *, int16_t, MemberId))
    fastrtps__set_int16_value;

  fastrtps_ser_interface->set_uint16_value =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *, uint16_t, MemberId))
    fastrtps__set_uint16_value;

  fastrtps_ser_interface->set_int32_value =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *, int32_t, MemberId))
    fastrtps__set_int32_value;

  fastrtps_ser_interface->set_uint32_value =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *, uint32_t, MemberId))
    fastrtps__set_uint32_value;

  fastrtps_ser_interface->set_int64_value =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *, int64_t, MemberId))
    fastrtps__set_int64_value;

  fastrtps_ser_interface->set_uint64_value =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *, uint64_t, MemberId))
    fastrtps__set_uint64_value;

  fastrtps_ser_interface->set_string_value =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *, const char *, MemberId))
    fastrtps__set_string_value;

  fastrtps_ser_interface->set_wstring_value =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *, const wchar_t *, MemberId))
    fastrtps__set_wstring_value;


  // DYNAMIC TYPE SEQUENCES
  fastrtps_ser_interface->clear_sequence_data =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *))
    fastrtps__clear_sequence_data;

  fastrtps_ser_interface->remove_sequence_data =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *, MemberId))
    fastrtps__remove_sequence_data;

  fastrtps_ser_interface->insert_sequence_data =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *, MemberId *))
    fastrtps__insert_sequence_data;

  fastrtps_ser_interface->insert_bool_value =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *, bool, MemberId *))
    fastrtps__insert_bool_value;

  fastrtps_ser_interface->insert_byte_value =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *, uint8_t, MemberId *))
    fastrtps__insert_byte_value;

  fastrtps_ser_interface->insert_char_value =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *, char, MemberId *))
    fastrtps__insert_char_value;

  fastrtps_ser_interface->insert_float32_value =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *, float, MemberId *))
    fastrtps__insert_float32_value;

  fastrtps_ser_interface->insert_float64_value =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *, double, MemberId *))
    fastrtps__insert_float64_value;

  fastrtps_ser_interface->insert_int16_value =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *, int16_t, MemberId *))
    fastrtps__insert_int16_value;

  fastrtps_ser_interface->insert_uint16_value =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *, uint16_t, MemberId *))
    fastrtps__insert_uint16_value;

  fastrtps_ser_interface->insert_int32_value =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *, int32_t, MemberId *))
    fastrtps__insert_int32_value;

  fastrtps_ser_interface->insert_uint32_value =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *, uint32_t, MemberId *))
    fastrtps__insert_uint32_value;

  fastrtps_ser_interface->insert_int64_value =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *, int64_t, MemberId *))
    fastrtps__insert_int64_value;

  fastrtps_ser_interface->insert_uint64_value =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *, uint64_t, MemberId *))
    fastrtps__insert_uint64_value;

  fastrtps_ser_interface->insert_string_value =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *, const char *, MemberId *))
    fastrtps__insert_string_value;

  fastrtps_ser_interface->insert_wstring_value =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *, const wchar_t *, MemberId *))
    fastrtps__insert_wstring_value;


  // DYNAMIC TYPE NESTED
  fastrtps_ser_interface->get_complex_value =
    (void (*)(ser_impl_t *, const ser_dynamic_data_t *, ser_dynamic_data_t **, MemberId))
    fastrtps__get_complex_value;

  fastrtps_ser_interface->set_complex_value =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *, ser_dynamic_data_t *, MemberId))
    fastrtps__set_complex_value;

  fastrtps_ser_interface->insert_const_complex_value =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *, const ser_dynamic_data_t *, MemberId *))
    fastrtps__insert_const_complex_value;

  fastrtps_ser_interface->insert_complex_value =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *, ser_dynamic_data_t *, MemberId *))
    fastrtps__insert_complex_value;

  fastrtps_ser_interface->insert_complex_value_ptr =
    (void (*)(ser_impl_t *, ser_dynamic_data_t *, ser_dynamic_data_t *, MemberId *))
    fastrtps__insert_complex_value_ptr;


  return fastrtps_ser_interface;
}


void
fastrtps__ser_impl_fini(ser_impl_t * ser_impl)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  fastrtps_ser_impl->type_factory_->delete_instance();
  fastrtps_ser_impl->data_factory_->delete_instance();
}

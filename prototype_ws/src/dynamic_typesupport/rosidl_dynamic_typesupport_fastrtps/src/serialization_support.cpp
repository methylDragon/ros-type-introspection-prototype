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

#include <rosidl_dynamic_typesupport/api/serialization_support_interface.h>
#include <rosidl_dynamic_typesupport/types.h>

#include "detail/dynamic_data.h"
#include "detail/dynamic_type.h"
#include "detail/serialization_support_impl_handle.h"

#include <rosidl_dynamic_typesupport_fastrtps/identifier.h>
#include <rosidl_dynamic_typesupport_fastrtps/serialization_support.h>

#include <fastrtps/types/DynamicDataFactory.h>
#include <fastrtps/types/DynamicTypeBuilderFactory.h>


// =================================================================================================
// SERIALIZATION SUPPORT IMPL
// =================================================================================================
rosidl_dynamic_typesupport_serialization_support_impl_t *
rosidl_dynamic_typesupport_fastrtps_create_serialization_support_impl()
{
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl =
    (rosidl_dynamic_typesupport_serialization_support_impl_t *) calloc(1, sizeof(rosidl_dynamic_typesupport_serialization_support_impl_t));
  serialization_support_impl->library_identifier =
    fastrtps_serialization_support_library_identifier;

  fastrtps__serialization_support_impl_handle_t * serialization_support_impl_handle =
    (fastrtps__serialization_support_impl_handle_t *) calloc(1, sizeof(fastrtps__serialization_support_impl_handle_t));

  // The actual business
  serialization_support_impl_handle->type_factory_ =
    eprosima::fastrtps::types::DynamicTypeBuilderFactory::get_instance();
  serialization_support_impl_handle->data_factory_ =
    eprosima::fastrtps::types::DynamicDataFactory::get_instance();

  serialization_support_impl->handle = serialization_support_impl_handle;

  return serialization_support_impl;
}


// =================================================================================================
// SERIALIZATION SUPPORT INTERFACE
// =================================================================================================
rosidl_dynamic_typesupport_serialization_support_interface_t *
rosidl_dynamic_typesupport_fastrtps_create_serialization_support_interface()
{
  rosidl_dynamic_typesupport_serialization_support_interface_t * serialization_support_interface =
    (rosidl_dynamic_typesupport_serialization_support_interface_t *) calloc(1, sizeof(rosidl_dynamic_typesupport_serialization_support_interface_t));


  // CORE ==========================================================================================
  serialization_support_interface->library_identifier = fastrtps_serialization_support_library_identifier;

  serialization_support_interface->serialization_support_impl_handle_fini =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *))
    fastrtps__serialization_support_impl_handle_fini;


  // ===============================================================================================
  // DYNAMIC TYPE METHODS
  // ===============================================================================================
  // DYNAMIC TYPE UTILS
  serialization_support_interface->dynamic_type_equals =
    (bool (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, const rosidl_dynamic_typesupport_dynamic_type_impl_t *, const rosidl_dynamic_typesupport_dynamic_type_impl_t *))
    fastrtps__dynamic_type_equals;

  serialization_support_interface->get_dynamic_type_member_count =
    (uint32_t (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, const rosidl_dynamic_typesupport_dynamic_type_impl_t *))
    fastrtps__get_type_member_count;


  // DYNAMIC TYPE CONSTRUCTION
  serialization_support_interface->struct_type_builder_init =
    (rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, const char *))
    fastrtps__struct_type_builder_init;

  serialization_support_interface->struct_type_builder_fini =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *))
    fastrtps__struct_type_builder_fini;

  serialization_support_interface->build_struct_type =
    (rosidl_dynamic_typesupport_dynamic_type_impl_t * (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *))
    fastrtps__build_struct_type;

  serialization_support_interface->dynamic_type_init_from_description =
    (rosidl_dynamic_typesupport_dynamic_type_impl_t * (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, type_description_t *))
    fastrtps__dynamic_type_init_from_description;

  serialization_support_interface->dynamic_type_fini =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_impl_t *))
    fastrtps__dynamic_type_fini;


  // DYNAMIC TYPE PRIMITIVE MEMBERS
  serialization_support_interface->add_bool_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *))
    fastrtps__add_bool_member;

  serialization_support_interface->add_byte_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *))
    fastrtps__add_byte_member;

  serialization_support_interface->add_char_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *))
    fastrtps__add_char_member;

  serialization_support_interface->add_float32_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *))
    fastrtps__add_float32_member;

  serialization_support_interface->add_float64_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *))
    fastrtps__add_float64_member;

  serialization_support_interface->add_int8_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *))
    fastrtps__add_int8_member;

  serialization_support_interface->add_uint8_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *))
    fastrtps__add_uint8_member;

  serialization_support_interface->add_int16_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *))
    fastrtps__add_int16_member;

  serialization_support_interface->add_uint16_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *))
    fastrtps__add_uint16_member;

  serialization_support_interface->add_int32_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *))
    fastrtps__add_int32_member;

  serialization_support_interface->add_uint32_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *))
    fastrtps__add_uint32_member;

  serialization_support_interface->add_int64_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *))
    fastrtps__add_int64_member;

  serialization_support_interface->add_uint64_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *))
    fastrtps__add_uint64_member;

  serialization_support_interface->add_string_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *))
    fastrtps__add_string_member;

  serialization_support_interface->add_wstring_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *))
    fastrtps__add_wstring_member;

  serialization_support_interface->add_bounded_string_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, uint32_t))
    fastrtps__add_bounded_string_member;

  serialization_support_interface->add_bounded_wstring_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, uint32_t))
    fastrtps__add_bounded_wstring_member;


  // DYNAMIC TYPE STATIC ARRAY MEMBERS
  serialization_support_interface->add_bool_array_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, uint32_t))
    fastrtps__add_bool_array_member;
  serialization_support_interface->add_byte_array_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, uint32_t))
    fastrtps__add_byte_array_member;
  serialization_support_interface->add_char_array_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, uint32_t))
    fastrtps__add_char_array_member;

  serialization_support_interface->add_float32_array_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, uint32_t))
    fastrtps__add_float32_array_member;

  serialization_support_interface->add_float64_array_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, uint32_t))
    fastrtps__add_float64_array_member;

  serialization_support_interface->add_int8_array_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, uint32_t))
    fastrtps__add_int8_array_member;

  serialization_support_interface->add_uint8_array_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, uint32_t))
    fastrtps__add_uint8_array_member;

  serialization_support_interface->add_int16_array_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, uint32_t))
    fastrtps__add_int16_array_member;

  serialization_support_interface->add_uint16_array_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, uint32_t))
    fastrtps__add_uint16_array_member;

  serialization_support_interface->add_int32_array_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, uint32_t))
    fastrtps__add_int32_array_member;

  serialization_support_interface->add_uint32_array_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, uint32_t))
    fastrtps__add_uint32_array_member;

  serialization_support_interface->add_int64_array_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, uint32_t))
    fastrtps__add_int64_array_member;

  serialization_support_interface->add_uint64_array_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, uint32_t))
    fastrtps__add_uint64_array_member;

  serialization_support_interface->add_string_array_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, uint32_t))
    fastrtps__add_string_array_member;

  serialization_support_interface->add_wstring_array_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, uint32_t))
    fastrtps__add_wstring_array_member;

  serialization_support_interface->add_bounded_string_array_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, uint32_t, uint32_t))
    fastrtps__add_bounded_string_array_member;

  serialization_support_interface->add_bounded_wstring_array_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, uint32_t, uint32_t))
    fastrtps__add_bounded_wstring_array_member;


  // DYNAMIC TYPE UNBOUNDED SEQUENCE MEMBERS
  serialization_support_interface->add_bool_unbounded_sequence_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *))
    fastrtps__add_bool_unbounded_sequence_member;

  serialization_support_interface->add_byte_unbounded_sequence_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *))
    fastrtps__add_byte_unbounded_sequence_member;

  serialization_support_interface->add_char_unbounded_sequence_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *))
    fastrtps__add_char_unbounded_sequence_member;

  serialization_support_interface->add_float32_unbounded_sequence_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *))
    fastrtps__add_float32_unbounded_sequence_member;

  serialization_support_interface->add_float64_unbounded_sequence_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *))
    fastrtps__add_float64_unbounded_sequence_member;

  serialization_support_interface->add_int8_unbounded_sequence_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *))
    fastrtps__add_int8_unbounded_sequence_member;

  serialization_support_interface->add_uint8_unbounded_sequence_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *))
    fastrtps__add_uint8_unbounded_sequence_member;

  serialization_support_interface->add_int16_unbounded_sequence_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *))
    fastrtps__add_int16_unbounded_sequence_member;

  serialization_support_interface->add_uint16_unbounded_sequence_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *))
    fastrtps__add_uint16_unbounded_sequence_member;

  serialization_support_interface->add_int32_unbounded_sequence_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *))
    fastrtps__add_int32_unbounded_sequence_member;

  serialization_support_interface->add_uint32_unbounded_sequence_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *))
    fastrtps__add_uint32_unbounded_sequence_member;

  serialization_support_interface->add_int64_unbounded_sequence_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *))
    fastrtps__add_int64_unbounded_sequence_member;

  serialization_support_interface->add_uint64_unbounded_sequence_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *))
    fastrtps__add_uint64_unbounded_sequence_member;

  serialization_support_interface->add_string_unbounded_sequence_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *))
    fastrtps__add_string_unbounded_sequence_member;

  serialization_support_interface->add_wstring_unbounded_sequence_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *))
    fastrtps__add_wstring_unbounded_sequence_member;

  serialization_support_interface->add_bounded_string_unbounded_sequence_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, uint32_t))
    fastrtps__add_bounded_string_unbounded_sequence_member;

  serialization_support_interface->add_bounded_wstring_unbounded_sequence_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, uint32_t))
    fastrtps__add_bounded_wstring_unbounded_sequence_member;


  // DYNAMIC TYPE BOUNDED SEQUENCE MEMBERS
  serialization_support_interface->add_bool_bounded_sequence_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, uint32_t))
    fastrtps__add_bool_bounded_sequence_member;

  serialization_support_interface->add_byte_bounded_sequence_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, uint32_t))
    fastrtps__add_byte_bounded_sequence_member;

  serialization_support_interface->add_char_bounded_sequence_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, uint32_t))
    fastrtps__add_char_bounded_sequence_member;

  serialization_support_interface->add_float32_bounded_sequence_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, uint32_t))
    fastrtps__add_float32_bounded_sequence_member;

  serialization_support_interface->add_float64_bounded_sequence_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, uint32_t))
    fastrtps__add_float64_bounded_sequence_member;

  serialization_support_interface->add_int8_bounded_sequence_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, uint32_t))
    fastrtps__add_int8_bounded_sequence_member;

  serialization_support_interface->add_uint8_bounded_sequence_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, uint32_t))
    fastrtps__add_uint8_bounded_sequence_member;

  serialization_support_interface->add_int16_bounded_sequence_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, uint32_t))
    fastrtps__add_int16_bounded_sequence_member;

  serialization_support_interface->add_uint16_bounded_sequence_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, uint32_t))
    fastrtps__add_uint16_bounded_sequence_member;

  serialization_support_interface->add_int32_bounded_sequence_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, uint32_t))
    fastrtps__add_int32_bounded_sequence_member;

  serialization_support_interface->add_uint32_bounded_sequence_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, uint32_t))
    fastrtps__add_uint32_bounded_sequence_member;

  serialization_support_interface->add_int64_bounded_sequence_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, uint32_t))
    fastrtps__add_int64_bounded_sequence_member;

  serialization_support_interface->add_uint64_bounded_sequence_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, uint32_t))
    fastrtps__add_uint64_bounded_sequence_member;

  serialization_support_interface->add_string_bounded_sequence_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, uint32_t))
    fastrtps__add_string_bounded_sequence_member;

  serialization_support_interface->add_wstring_bounded_sequence_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, uint32_t))
    fastrtps__add_wstring_bounded_sequence_member;

  serialization_support_interface->add_bounded_string_bounded_sequence_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, uint32_t, uint32_t))
    fastrtps__add_bounded_string_bounded_sequence_member;

  serialization_support_interface->add_bounded_wstring_bounded_sequence_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, uint32_t, uint32_t))
    fastrtps__add_bounded_wstring_bounded_sequence_member;


  // DYNAMIC TYPE NESTED MEMBERS
  serialization_support_interface->add_nested_struct_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, rosidl_dynamic_typesupport_dynamic_type_impl_t *))
    fastrtps__add_nested_struct_member;

  serialization_support_interface->add_nested_struct_array_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, rosidl_dynamic_typesupport_dynamic_type_impl_t *, uint32_t))
    fastrtps__add_nested_struct_array_member;

  serialization_support_interface->add_nested_struct_unbounded_sequence_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, rosidl_dynamic_typesupport_dynamic_type_impl_t *))
    fastrtps__add_nested_struct_unbounded_sequence_member;

  serialization_support_interface->add_nested_struct_bounded_sequence_member =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *, rosidl_dynamic_typesupport_member_id_t, const char *, rosidl_dynamic_typesupport_dynamic_type_impl_t *, uint32_t))
    fastrtps__add_nested_struct_bounded_sequence_member;

  // ===============================================================================================
  // DYNAMIC DATA METHODS
  // ===============================================================================================
  // DYNAMIC DATA UTILS
  serialization_support_interface->clear_all_values =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *))
    fastrtps__clear_all_values;

  serialization_support_interface->clear_nonkey_values =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *))
    fastrtps__clear_nonkey_values;

  serialization_support_interface->clear_sequence_data =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *))
    fastrtps__clear_sequence_data;

  serialization_support_interface->clear_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, rosidl_dynamic_typesupport_member_id_t))
    fastrtps__clear_value;

  serialization_support_interface->dynamic_data_equals =
    (bool (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, const rosidl_dynamic_typesupport_dynamic_data_impl_t *, const rosidl_dynamic_typesupport_dynamic_data_impl_t *))
    fastrtps__dynamic_data_equals;

  serialization_support_interface->get_dynamic_data_item_count =
    (uint32_t (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, const rosidl_dynamic_typesupport_dynamic_data_impl_t *))
    fastrtps__get_dynamic_data_item_count;

  serialization_support_interface->get_dynamic_data_member_id_by_name =
    (rosidl_dynamic_typesupport_member_id_t (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, const rosidl_dynamic_typesupport_dynamic_data_impl_t *, const char *))
    fastrtps__get_dynamic_data_member_id_by_name;

  serialization_support_interface->get_dynamic_data_member_id_at_index =
    (rosidl_dynamic_typesupport_member_id_t (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, const rosidl_dynamic_typesupport_dynamic_data_impl_t *, uint32_t))
    fastrtps__get_dynamic_data_member_id_at_index;

  serialization_support_interface->get_array_index =
    (rosidl_dynamic_typesupport_member_id_t (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, uint32_t))
    fastrtps__get_array_index;

  serialization_support_interface->loan_value =
    (rosidl_dynamic_typesupport_dynamic_data_impl_t * (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, rosidl_dynamic_typesupport_member_id_t))
    fastrtps__loan_value;

  serialization_support_interface->return_loaned_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, const rosidl_dynamic_typesupport_dynamic_data_impl_t *))
    fastrtps__return_loaned_value;

  serialization_support_interface->dynamic_data_print =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *))
    fastrtps__dynamic_data_print;


  // DYNAMIC DATA CONSTRUCTION
  serialization_support_interface->dynamic_data_init_from_builder =
    (rosidl_dynamic_typesupport_dynamic_data_impl_t * (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t *))
    fastrtps__dynamic_data_init_from_builder;

  serialization_support_interface->dynamic_data_init_from_dynamic_type =
    (rosidl_dynamic_typesupport_dynamic_data_impl_t * (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_type_impl_t *))
    fastrtps__dynamic_data_init_from_dynamic_type;

  serialization_support_interface->dynamic_data_clone =
    (rosidl_dynamic_typesupport_dynamic_data_impl_t * (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, const rosidl_dynamic_typesupport_dynamic_data_impl_t *))
    fastrtps__dynamic_data_clone;

  serialization_support_interface->dynamic_data_fini =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *))
    fastrtps__dynamic_data_fini;


  // DYNAMIC DATA PRIMITIVE MEMBER GETTERS
  serialization_support_interface->get_bool_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, const rosidl_dynamic_typesupport_dynamic_data_impl_t *, bool *, rosidl_dynamic_typesupport_member_id_t))
    fastrtps__get_bool_value;

  serialization_support_interface->get_byte_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, const rosidl_dynamic_typesupport_dynamic_data_impl_t *, uint8_t *, rosidl_dynamic_typesupport_member_id_t))
    fastrtps__get_byte_value;

  serialization_support_interface->get_char_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, const rosidl_dynamic_typesupport_dynamic_data_impl_t *, char *, rosidl_dynamic_typesupport_member_id_t))
    fastrtps__get_char_value;

  serialization_support_interface->get_float32_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, const rosidl_dynamic_typesupport_dynamic_data_impl_t *, float *, rosidl_dynamic_typesupport_member_id_t))
    fastrtps__get_float32_value;

  serialization_support_interface->get_float64_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, const rosidl_dynamic_typesupport_dynamic_data_impl_t *, double *, rosidl_dynamic_typesupport_member_id_t))
    fastrtps__get_float64_value;

  serialization_support_interface->get_int8_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, const rosidl_dynamic_typesupport_dynamic_data_impl_t *, int8_t *, rosidl_dynamic_typesupport_member_id_t))
    fastrtps__get_int8_value;

  serialization_support_interface->get_uint8_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, const rosidl_dynamic_typesupport_dynamic_data_impl_t *, uint8_t *, rosidl_dynamic_typesupport_member_id_t))
    fastrtps__get_uint8_value;

  serialization_support_interface->get_int16_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, const rosidl_dynamic_typesupport_dynamic_data_impl_t *, int16_t *, rosidl_dynamic_typesupport_member_id_t))
    fastrtps__get_int16_value;

  serialization_support_interface->get_uint16_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, const rosidl_dynamic_typesupport_dynamic_data_impl_t *, uint16_t *, rosidl_dynamic_typesupport_member_id_t))
    fastrtps__get_uint16_value;

  serialization_support_interface->get_int32_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, const rosidl_dynamic_typesupport_dynamic_data_impl_t *, int32_t *, rosidl_dynamic_typesupport_member_id_t))
    fastrtps__get_int32_value;

  serialization_support_interface->get_uint32_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, const rosidl_dynamic_typesupport_dynamic_data_impl_t *, uint32_t *, rosidl_dynamic_typesupport_member_id_t))
    fastrtps__get_uint32_value;

  serialization_support_interface->get_int64_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, const rosidl_dynamic_typesupport_dynamic_data_impl_t *, int64_t *, rosidl_dynamic_typesupport_member_id_t))
    fastrtps__get_int64_value;

  serialization_support_interface->get_uint64_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, const rosidl_dynamic_typesupport_dynamic_data_impl_t *, uint64_t *, rosidl_dynamic_typesupport_member_id_t))
    fastrtps__get_uint64_value;

  serialization_support_interface->get_string_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, const rosidl_dynamic_typesupport_dynamic_data_impl_t *, const char **, rosidl_dynamic_typesupport_member_id_t))
    fastrtps__get_string_value;

  serialization_support_interface->get_wstring_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, const rosidl_dynamic_typesupport_dynamic_data_impl_t *, const wchar_t **, rosidl_dynamic_typesupport_member_id_t))
    fastrtps__get_wstring_value;


  // DYNAMIC DATA PRIMITIVE MEMBER SETTERS
  serialization_support_interface->set_bool_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, bool, rosidl_dynamic_typesupport_member_id_t))
    fastrtps__set_bool_value;

  serialization_support_interface->set_byte_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, uint8_t, rosidl_dynamic_typesupport_member_id_t))
    fastrtps__set_byte_value;

  serialization_support_interface->set_char_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, char, rosidl_dynamic_typesupport_member_id_t))
    fastrtps__set_char_value;

  serialization_support_interface->set_float32_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, float, rosidl_dynamic_typesupport_member_id_t))
    fastrtps__set_float32_value;

  serialization_support_interface->set_float64_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, double, rosidl_dynamic_typesupport_member_id_t))
    fastrtps__set_float64_value;

  serialization_support_interface->set_int8_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, int8_t, rosidl_dynamic_typesupport_member_id_t))
    fastrtps__set_int8_value;

  serialization_support_interface->set_uint8_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, uint8_t, rosidl_dynamic_typesupport_member_id_t))
    fastrtps__set_uint8_value;

  serialization_support_interface->set_int16_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, int16_t, rosidl_dynamic_typesupport_member_id_t))
    fastrtps__set_int16_value;

  serialization_support_interface->set_uint16_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, uint16_t, rosidl_dynamic_typesupport_member_id_t))
    fastrtps__set_uint16_value;

  serialization_support_interface->set_int32_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, int32_t, rosidl_dynamic_typesupport_member_id_t))
    fastrtps__set_int32_value;

  serialization_support_interface->set_uint32_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, uint32_t, rosidl_dynamic_typesupport_member_id_t))
    fastrtps__set_uint32_value;

  serialization_support_interface->set_int64_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, int64_t, rosidl_dynamic_typesupport_member_id_t))
    fastrtps__set_int64_value;

  serialization_support_interface->set_uint64_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, uint64_t, rosidl_dynamic_typesupport_member_id_t))
    fastrtps__set_uint64_value;

  serialization_support_interface->set_string_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, const char *, rosidl_dynamic_typesupport_member_id_t))
    fastrtps__set_string_value;

  serialization_support_interface->set_wstring_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, const wchar_t *, rosidl_dynamic_typesupport_member_id_t))
    fastrtps__set_wstring_value;


  // DYNAMIC TYPE SEQUENCES
  serialization_support_interface->clear_sequence_data =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *))
    fastrtps__clear_sequence_data;

  serialization_support_interface->remove_sequence_data =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, rosidl_dynamic_typesupport_member_id_t))
    fastrtps__remove_sequence_data;

  serialization_support_interface->insert_sequence_data =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, rosidl_dynamic_typesupport_member_id_t *))
    fastrtps__insert_sequence_data;

  serialization_support_interface->insert_bool_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, bool, rosidl_dynamic_typesupport_member_id_t *))
    fastrtps__insert_bool_value;

  serialization_support_interface->insert_byte_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, uint8_t, rosidl_dynamic_typesupport_member_id_t *))
    fastrtps__insert_byte_value;

  serialization_support_interface->insert_char_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, char, rosidl_dynamic_typesupport_member_id_t *))
    fastrtps__insert_char_value;

  serialization_support_interface->insert_float32_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, float, rosidl_dynamic_typesupport_member_id_t *))
    fastrtps__insert_float32_value;

  serialization_support_interface->insert_float64_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, double, rosidl_dynamic_typesupport_member_id_t *))
    fastrtps__insert_float64_value;

  serialization_support_interface->insert_int16_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, int16_t, rosidl_dynamic_typesupport_member_id_t *))
    fastrtps__insert_int16_value;

  serialization_support_interface->insert_uint16_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, uint16_t, rosidl_dynamic_typesupport_member_id_t *))
    fastrtps__insert_uint16_value;

  serialization_support_interface->insert_int32_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, int32_t, rosidl_dynamic_typesupport_member_id_t *))
    fastrtps__insert_int32_value;

  serialization_support_interface->insert_uint32_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, uint32_t, rosidl_dynamic_typesupport_member_id_t *))
    fastrtps__insert_uint32_value;

  serialization_support_interface->insert_int64_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, int64_t, rosidl_dynamic_typesupport_member_id_t *))
    fastrtps__insert_int64_value;

  serialization_support_interface->insert_uint64_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, uint64_t, rosidl_dynamic_typesupport_member_id_t *))
    fastrtps__insert_uint64_value;

  serialization_support_interface->insert_string_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, const char *, rosidl_dynamic_typesupport_member_id_t *))
    fastrtps__insert_string_value;

  serialization_support_interface->insert_wstring_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, const wchar_t *, rosidl_dynamic_typesupport_member_id_t *))
    fastrtps__insert_wstring_value;


  // DYNAMIC TYPE NESTED
  serialization_support_interface->get_complex_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, const rosidl_dynamic_typesupport_dynamic_data_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t **, rosidl_dynamic_typesupport_member_id_t))
    fastrtps__get_complex_value;

  serialization_support_interface->set_complex_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, rosidl_dynamic_typesupport_member_id_t))
    fastrtps__set_complex_value;

  serialization_support_interface->insert_const_complex_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, const rosidl_dynamic_typesupport_dynamic_data_impl_t *, rosidl_dynamic_typesupport_member_id_t *))
    fastrtps__insert_const_complex_value;

  serialization_support_interface->insert_complex_value =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, rosidl_dynamic_typesupport_member_id_t *))
    fastrtps__insert_complex_value;

  serialization_support_interface->insert_complex_value_ptr =
    (void (*)(rosidl_dynamic_typesupport_serialization_support_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, rosidl_dynamic_typesupport_dynamic_data_impl_t *, rosidl_dynamic_typesupport_member_id_t *))
    fastrtps__insert_complex_value_ptr;


  return serialization_support_interface;
}

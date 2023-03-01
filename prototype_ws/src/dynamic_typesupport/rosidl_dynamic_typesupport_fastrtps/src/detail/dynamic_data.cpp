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

#include <fastrtps/types/DynamicDataHelper.hpp>
#include <fastrtps/types/DynamicTypeBuilderPtr.h>

#include "dynamic_data.h"
#include "serialization_support_impl_handle.h"

#include <rosidl_dynamic_typesupport/api/serialization_support_interface.h>


using eprosima::fastrtps::types::DynamicData;
using eprosima::fastrtps::types::DynamicData_ptr;
using eprosima::fastrtps::types::DynamicDataHelper;

using eprosima::fastrtps::types::DynamicTypeBuilder;
using eprosima::fastrtps::types::DynamicTypeBuilder_ptr;


// =================================================================================================
// DYNAMIC DATA
// =================================================================================================

// DYNAMIC DATA UTILS ==============================================================================
void
fastrtps__dynamic_data_clear_all_values(rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl)
{
  (void) serialization_support_impl;
  static_cast<DynamicData *>(data_impl->handle)->clear_all_values();
}


void
fastrtps__dynamic_data_clear_nonkey_values(rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl)
{
  (void) serialization_support_impl;
  static_cast<DynamicData *>(data_impl->handle)->clear_nonkey_values();
}


void
fastrtps__dynamic_data_clear_value(rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, rosidl_dynamic_typesupport_member_id_t id)
{
  (void) serialization_support_impl;
  static_cast<DynamicData *>(data_impl->handle)->clear_value(id);
}


bool
fastrtps__dynamic_data_equals(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * other_data_impl)
{
  (void) serialization_support_impl;
  return static_cast<const DynamicData *>(data_impl->handle)->equals(static_cast<const DynamicData *>(other_data_impl->handle));
}


uint32_t
fastrtps__dynamic_data_get_item_count(rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl)
{
  (void) serialization_support_impl;
  return static_cast<const DynamicData *>(data_impl->handle)->get_item_count();
}


rosidl_dynamic_typesupport_member_id_t
fastrtps__dynamic_data_get_member_id_by_name(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, const char * name)
{
  (void) serialization_support_impl;
  return static_cast<const DynamicData *>(data_impl->handle)->get_member_id_by_name(name);
}


rosidl_dynamic_typesupport_member_id_t
fastrtps__dynamic_data_get_member_id_at_index(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, uint32_t index)
{
  (void) serialization_support_impl;
  return static_cast<const DynamicData *>(data_impl->handle)->get_member_id_at_index(index);
}


rosidl_dynamic_typesupport_member_id_t
fastrtps__dynamic_data_get_array_index(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, uint32_t index)
{
  (void) serialization_support_impl;
  return static_cast<DynamicData *>(data_impl->handle)->get_array_index({index});
}


rosidl_dynamic_typesupport_dynamic_data_impl_t *
fastrtps__dynamic_data_loan_value(rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, rosidl_dynamic_typesupport_member_id_t id)
{
  (void) serialization_support_impl;
  return new rosidl_dynamic_typesupport_dynamic_data_impl_t{std::move(static_cast<DynamicData *>(data_impl->handle)->loan_value(id))};
}


void
fastrtps__dynamic_data_return_loaned_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * inner_data_impl)
{
  (void) serialization_support_impl;
  static_cast<DynamicData *>(data_impl->handle)
    ->return_loaned_value(static_cast<const DynamicData *>(inner_data_impl->handle));
}


void
fastrtps__dynamic_data_print(rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl)
{
  (void) serialization_support_impl;
  DynamicDataHelper::print(static_cast<DynamicData *>(data_impl->handle));
}


// DYNAMIC DATA CONSTRUCTION =======================================================================
rosidl_dynamic_typesupport_dynamic_data_impl_t *
fastrtps__dynamic_data_init_from_dynamic_type_builder(rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_type_builder_impl_t * type_builder_impl)
{
  return new rosidl_dynamic_typesupport_dynamic_data_impl_t{
    static_cast<fastrtps__serialization_support_impl_handle_t *>(serialization_support_impl->handle)
      ->data_factory_->create_data(static_cast<DynamicTypeBuilder *>(type_builder_impl->handle))
    };
}

rosidl_dynamic_typesupport_dynamic_data_impl_t *
fastrtps__dynamic_data_init_from_dynamic_type(rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_type_impl_t * type_impl)
{
  // NOTE(methylDragon): All this casting is unfortunately necessary...
  //
  //                     create_data only takes DynamicType_ptr (aka shared_ptr)
  //                     And passing a heap allocated shared_ptr is the only way to make sure the
  //                     lifetime of the dynamic type is preserved
  return new rosidl_dynamic_typesupport_dynamic_data_impl_t{
    static_cast<fastrtps__serialization_support_impl_handle_t *>(serialization_support_impl->handle)->data_factory_->create_data(
      eprosima::fastrtps::types::DynamicType_ptr(
        *static_cast<eprosima::fastrtps::types::DynamicType_ptr *>(type_impl->handle)
      )
    )
  };
}

rosidl_dynamic_typesupport_dynamic_data_impl_t *
fastrtps__dynamic_data_clone(rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl)
{
  return new rosidl_dynamic_typesupport_dynamic_data_impl_t{
    static_cast<fastrtps__serialization_support_impl_handle_t *>(serialization_support_impl->handle)->data_factory_->create_copy(
      static_cast<const DynamicData *>(data_impl->handle))
  };
}


void
fastrtps__dynamic_data_fini(rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl)
{
  static_cast<fastrtps__serialization_support_impl_handle_t *>(serialization_support_impl->handle)
    ->data_factory_->delete_data(static_cast<DynamicData *>(data_impl->handle));
}


// DYNAMIC DATA PRIMITIVE MEMBER GETTERS ===========================================================
void
fastrtps__dynamic_data_get_bool_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, bool * value, rosidl_dynamic_typesupport_member_id_t id)
{
  (void) serialization_support_impl;
  static_cast<const DynamicData *>(data_impl->handle)->get_bool_value(*value, id);
}


void
fastrtps__dynamic_data_get_byte_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, uint8_t * value, rosidl_dynamic_typesupport_member_id_t id)
{
  (void) serialization_support_impl;
  static_cast<const DynamicData *>(data_impl->handle)->get_byte_value(*value, id);
}


void
fastrtps__dynamic_data_get_char_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, char * value, rosidl_dynamic_typesupport_member_id_t id)
{
  (void) serialization_support_impl;
  static_cast<const DynamicData *>(data_impl->handle)->get_char8_value(*value, id);
}


void
fastrtps__dynamic_data_get_float32_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, float * value, rosidl_dynamic_typesupport_member_id_t id)
{
  (void) serialization_support_impl;
  static_cast<const DynamicData *>(data_impl->handle)->get_float32_value(*value, id);
}


void
fastrtps__dynamic_data_get_float64_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, double * value, rosidl_dynamic_typesupport_member_id_t id)
{
  (void) serialization_support_impl;
  static_cast<const DynamicData *>(data_impl->handle)->get_float64_value(*value, id);
}


void
fastrtps__dynamic_data_get_int8_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, int8_t * value, rosidl_dynamic_typesupport_member_id_t id)
{
  (void) serialization_support_impl;
  static_cast<const DynamicData *>(data_impl->handle)->get_int8_value(*value, id);
}


void
fastrtps__dynamic_data_get_uint8_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, uint8_t * value, rosidl_dynamic_typesupport_member_id_t id)
{
  (void) serialization_support_impl;
  static_cast<const DynamicData *>(data_impl->handle)->get_uint8_value(*value, id);
}


void
fastrtps__dynamic_data_get_int16_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, int16_t * value, rosidl_dynamic_typesupport_member_id_t id)
{
  (void) serialization_support_impl;
  static_cast<const DynamicData *>(data_impl->handle)->get_int16_value(*value, id);
}


void
fastrtps__dynamic_data_get_uint16_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, uint16_t * value, rosidl_dynamic_typesupport_member_id_t id)
{
  (void) serialization_support_impl;
  static_cast<const DynamicData *>(data_impl->handle)->get_uint16_value(*value, id);
}


void
fastrtps__dynamic_data_get_int32_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, int32_t * value, rosidl_dynamic_typesupport_member_id_t id)
{
  (void) serialization_support_impl;
  static_cast<const DynamicData *>(data_impl->handle)->get_int32_value(*value, id);
}


void
fastrtps__dynamic_data_get_uint32_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, uint32_t * value, rosidl_dynamic_typesupport_member_id_t id)
{
  (void) serialization_support_impl;
  static_cast<const DynamicData *>(data_impl->handle)->get_uint32_value(*value, id);
}


void
fastrtps__dynamic_data_get_int64_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, int64_t * value, rosidl_dynamic_typesupport_member_id_t id)
{
  (void) serialization_support_impl;
  static_cast<const DynamicData *>(data_impl->handle)->get_int64_value(*value, id);
}


void
fastrtps__dynamic_data_get_uint64_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, uint64_t * value, rosidl_dynamic_typesupport_member_id_t id)
{
  (void) serialization_support_impl;
  static_cast<const DynamicData *>(data_impl->handle)->get_uint64_value(*value, id);
}


void
fastrtps__dynamic_data_get_string_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, const char ** value, rosidl_dynamic_typesupport_member_id_t id)
{
  (void) serialization_support_impl;
  std::string tmp;
  static_cast<const DynamicData *>(data_impl->handle)->get_string_value(tmp, id);  // Lifetime is in the data_impl obj
  *value = tmp.c_str();
}


void
fastrtps__dynamic_data_get_wstring_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, const wchar_t ** value, rosidl_dynamic_typesupport_member_id_t id)
{
  (void) serialization_support_impl;
  std::wstring tmp;
  static_cast<const DynamicData *>(data_impl->handle)->get_wstring_value(tmp, id);
  *value = tmp.c_str();
}


// DYNAMIC DATA PRIMITIVE MEMBER SETTERS ===========================================================
void
fastrtps__dynamic_data_set_bool_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, bool value, rosidl_dynamic_typesupport_member_id_t id)
{
  (void) serialization_support_impl;
  static_cast<DynamicData *>(data_impl->handle)->set_bool_value(value, id);
}


void
fastrtps__dynamic_data_set_byte_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, uint8_t value, rosidl_dynamic_typesupport_member_id_t id)
{
  (void) serialization_support_impl;
  static_cast<DynamicData *>(data_impl->handle)->set_byte_value(value, id);
}


void
fastrtps__dynamic_data_set_char_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, char value, rosidl_dynamic_typesupport_member_id_t id)
{
  (void) serialization_support_impl;
  static_cast<DynamicData *>(data_impl->handle)->set_char8_value(value, id);
}


void
fastrtps__dynamic_data_set_float32_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, float value, rosidl_dynamic_typesupport_member_id_t id)
{
  (void) serialization_support_impl;
  static_cast<DynamicData *>(data_impl->handle)->set_float32_value(value, id);
}


void
fastrtps__dynamic_data_set_float64_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, double value, rosidl_dynamic_typesupport_member_id_t id)
{
  (void) serialization_support_impl;
  static_cast<DynamicData *>(data_impl->handle)->set_float64_value(value, id);
}


void
fastrtps__dynamic_data_set_int8_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, int8_t value, rosidl_dynamic_typesupport_member_id_t id)
{
  (void) serialization_support_impl;
  static_cast<DynamicData *>(data_impl->handle)->set_int8_value(value, id);
}


void
fastrtps__dynamic_data_set_uint8_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, uint8_t value, rosidl_dynamic_typesupport_member_id_t id)
{
  (void) serialization_support_impl;
  static_cast<DynamicData *>(data_impl->handle)->set_uint8_value(value, id);
}


void
fastrtps__dynamic_data_set_int16_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, int16_t value, rosidl_dynamic_typesupport_member_id_t id)
{
  (void) serialization_support_impl;
  static_cast<DynamicData *>(data_impl->handle)->set_int16_value(value, id);
}


void
fastrtps__dynamic_data_set_uint16_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, uint16_t value, rosidl_dynamic_typesupport_member_id_t id)
{
  (void) serialization_support_impl;
  static_cast<DynamicData *>(data_impl->handle)->set_uint16_value(value, id);
}


void
fastrtps__dynamic_data_set_int32_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, int32_t value, rosidl_dynamic_typesupport_member_id_t id)
{
  (void) serialization_support_impl;
  static_cast<DynamicData *>(data_impl->handle)->set_int32_value(value, id);
}


void
fastrtps__dynamic_data_set_uint32_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, uint32_t value, rosidl_dynamic_typesupport_member_id_t id)
{
  (void) serialization_support_impl;
  static_cast<DynamicData *>(data_impl->handle)->set_uint32_value(value, id);
}


void
fastrtps__dynamic_data_set_int64_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, int64_t value, rosidl_dynamic_typesupport_member_id_t id)
{
  (void) serialization_support_impl;
  static_cast<DynamicData *>(data_impl->handle)->set_int64_value(value, id);
}


void
fastrtps__dynamic_data_set_uint64_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, uint64_t value, rosidl_dynamic_typesupport_member_id_t id)
{
  (void) serialization_support_impl;
  static_cast<DynamicData *>(data_impl->handle)->set_uint64_value(value, id);
}


void
fastrtps__dynamic_data_set_string_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, const char * value, rosidl_dynamic_typesupport_member_id_t id)
{
  (void) serialization_support_impl;
  const std::string tmp(value);
  // TODO(methylDragon): Check for dealloc
  static_cast<DynamicData *>(data_impl->handle)->set_string_value(tmp, id);
}


void
fastrtps__dynamic_data_set_wstring_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl,
  rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, const wchar_t * value, rosidl_dynamic_typesupport_member_id_t id)
{
  (void) serialization_support_impl;
  const std::wstring tmp(value);
  // TODO(methylDragon): Check for dealloc
  static_cast<DynamicData *>(data_impl->handle)->set_wstring_value(tmp, id);
}


// DYNAMIC DATA SEQUENCES ==========================================================================
void
fastrtps__dynamic_data_clear_sequence_data(rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl)
{
  (void) serialization_support_impl;
  static_cast<DynamicData *>(data_impl->handle)->clear_data();
}


void
fastrtps__dynamic_data_remove_sequence_data(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, rosidl_dynamic_typesupport_member_id_t id)
{
  (void) serialization_support_impl;
  static_cast<DynamicData *>(data_impl->handle)->remove_sequence_data(id);
}


void
fastrtps__dynamic_data_insert_sequence_data(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (void) serialization_support_impl;
  static_cast<DynamicData *>(data_impl->handle)->insert_sequence_data(*out_id);
}


void
fastrtps__dynamic_data_insert_bool_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, bool value, rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (void) serialization_support_impl;
  static_cast<DynamicData *>(data_impl->handle)->insert_bool_value(value, *out_id);
}


void
fastrtps__dynamic_data_insert_byte_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, uint8_t value, rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (void) serialization_support_impl;
  static_cast<DynamicData *>(data_impl->handle)->insert_byte_value(value, *out_id);
}


void
fastrtps__dynamic_data_insert_char_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, char value, rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (void) serialization_support_impl;
  static_cast<DynamicData *>(data_impl->handle)->insert_char8_value(value, *out_id);
}


void
fastrtps__dynamic_data_insert_float32_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, float value, rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (void) serialization_support_impl;
  static_cast<DynamicData *>(data_impl->handle)->insert_float32_value(value, *out_id);
}


void
fastrtps__dynamic_data_insert_float64_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, double value, rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (void) serialization_support_impl;
  static_cast<DynamicData *>(data_impl->handle)->insert_float64_value(value, *out_id);
}


void
fastrtps__dynamic_data_insert_int16_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, int16_t value, rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (void) serialization_support_impl;
  static_cast<DynamicData *>(data_impl->handle)->insert_int16_value(value, *out_id);
}


void
fastrtps__dynamic_data_insert_uint16_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, uint16_t value, rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (void) serialization_support_impl;
  static_cast<DynamicData *>(data_impl->handle)->insert_uint16_value(value, *out_id);
}


void
fastrtps__dynamic_data_insert_int32_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, int32_t value, rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (void) serialization_support_impl;
  static_cast<DynamicData *>(data_impl->handle)->insert_int32_value(value, *out_id);
}


void
fastrtps__dynamic_data_insert_uint32_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, uint32_t value, rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (void) serialization_support_impl;
  static_cast<DynamicData *>(data_impl->handle)->insert_uint32_value(value, *out_id);
}


void
fastrtps__dynamic_data_insert_int64_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, int64_t value, rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (void) serialization_support_impl;
  static_cast<DynamicData *>(data_impl->handle)->insert_int64_value(value, *out_id);
}


void
fastrtps__dynamic_data_insert_uint64_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, uint64_t value, rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (void) serialization_support_impl;
  static_cast<DynamicData *>(data_impl->handle)->insert_uint64_value(value, *out_id);
}


void
fastrtps__dynamic_data_insert_string_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, const char * value, rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (void) serialization_support_impl;
  static_cast<DynamicData *>(data_impl->handle)->insert_string_value(value, *out_id);
}


void
fastrtps__dynamic_data_insert_wstring_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, const wchar_t * value,
  rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (void) serialization_support_impl;
  static_cast<DynamicData *>(data_impl->handle)->insert_wstring_value(value, *out_id);
}


// DYNAMIC DATA NESTED =============================================================================
void
fastrtps__dynamic_data_get_complex_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t ** value, rosidl_dynamic_typesupport_member_id_t id)
{
  (void) serialization_support_impl;
  auto tmp = static_cast<DynamicData *>((*value)->handle);
  // TODO(methylDragon): Check for dealloc
  static_cast<const DynamicData *>(data_impl->handle)->get_complex_value(&tmp, id);
}


void
fastrtps__dynamic_data_set_complex_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * value, rosidl_dynamic_typesupport_member_id_t id)
{
  (void) serialization_support_impl;
  static_cast<DynamicData *>(data_impl->handle)->set_complex_value(
    static_cast<DynamicData *>(value->handle), id);
}


void
fastrtps__dynamic_data_insert_const_complex_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, const rosidl_dynamic_typesupport_dynamic_data_impl_t * value, rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (void) serialization_support_impl;
  static_cast<DynamicData *>(data_impl->handle)->insert_complex_value(
    static_cast<const DynamicData *>(value->handle), *out_id);
}


void
fastrtps__dynamic_data_insert_complex_value(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * value, rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (void) serialization_support_impl;
  static_cast<DynamicData *>(data_impl->handle)->insert_complex_value(
    static_cast<DynamicData *>(value->handle), *out_id);
}


void
fastrtps__dynamic_data_insert_complex_value_ptr(
  rosidl_dynamic_typesupport_serialization_support_impl_t * serialization_support_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * data_impl, rosidl_dynamic_typesupport_dynamic_data_impl_t * value, rosidl_dynamic_typesupport_member_id_t * out_id)
{
  (void) serialization_support_impl;
  static_cast<DynamicData *>(data_impl->handle)->insert_complex_value(
    static_cast<DynamicData *>(value->handle), *out_id);
}

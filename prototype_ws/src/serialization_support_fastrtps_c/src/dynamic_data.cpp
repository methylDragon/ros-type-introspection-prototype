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

#include <serialization_support_fastrtps_c/dynamic_data.h>
#include <serialization_support_fastrtps_c/serialization_impl.h>

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
fastrtps__clear_all_values(ser_impl_t * ser_impl, ser_dynamic_data_t * data)
{
  (void) ser_impl;
  static_cast<DynamicData *>(data->impl)->clear_all_values();
}


void
fastrtps__clear_nonkey_values(ser_impl_t * ser_impl, ser_dynamic_data_t * data)
{
  (void) ser_impl;
  static_cast<DynamicData *>(data->impl)->clear_nonkey_values();
}


void
fastrtps__clear_value(ser_impl_t * ser_impl, ser_dynamic_data_t * data, MemberId id)
{
  (void) ser_impl;
  static_cast<DynamicData *>(data->impl)->clear_value(id);
}


bool
fastrtps__data_equals(
  ser_impl_t * ser_impl, const ser_dynamic_data_t * data, const ser_dynamic_data_t * other)
{
  (void) ser_impl;
  return static_cast<const DynamicData *>(data->impl)->equals(static_cast<const DynamicData *>(other->impl));
}


uint32_t
fastrtps__get_data_item_count(ser_impl_t * ser_impl, const ser_dynamic_data_t * data)
{
  (void) ser_impl;
  return static_cast<const DynamicData *>(data->impl)->get_item_count();
}


MemberId
fastrtps__get_data_member_id_by_name(
  ser_impl_t * ser_impl, const ser_dynamic_data_t * data, const char * name)
{
  (void) ser_impl;
  return static_cast<const DynamicData *>(data->impl)->get_member_id_by_name(name);
}


MemberId
fastrtps__get_data_member_id_at_index(
  ser_impl_t * ser_impl, const ser_dynamic_data_t * data, uint32_t index)
{
  (void) ser_impl;
  return static_cast<const DynamicData *>(data->impl)->get_member_id_at_index(index);
}


MemberId
fastrtps__get_array_index(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, uint32_t index)
{
  (void) ser_impl;
  return static_cast<DynamicData *>(data->impl)->get_array_index({index});
}


ser_dynamic_data_t *
fastrtps__loan_value(ser_impl_t * ser_impl, ser_dynamic_data_t * data, MemberId id)
{
  (void) ser_impl;
  return new ser_dynamic_data_t{std::move(static_cast<DynamicData *>(data->impl)->loan_value(id))};
}


void
fastrtps__return_loaned_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, const ser_dynamic_data_t * inner_data)
{
  (void) ser_impl;
  static_cast<DynamicData *>(data->impl)
    ->return_loaned_value(static_cast<const DynamicData *>(inner_data->impl));
}


void
fastrtps__print_dynamic_data(ser_impl_t * ser_impl, ser_dynamic_data_t * data)
{
  (void) ser_impl;
  DynamicDataHelper::print(static_cast<DynamicData *>(data->impl));
}


// DYNAMIC DATA CONSTRUCTION =======================================================================
ser_dynamic_data_t *
fastrtps__data_init_from_builder(ser_impl_t * ser_impl, ser_type_builder_t * builder)
{
  return new ser_dynamic_data_t{
    static_cast<fastrtps_ser_impl_t *>(ser_impl->impl)
      ->data_factory_->create_data(static_cast<DynamicTypeBuilder *>(builder->impl))
    };
}

ser_dynamic_data_t *
fastrtps__data_init_from_type(ser_impl_t * ser_impl, ser_dynamic_type_t * type)
{
  // NOTE(methylDragon): All this casting is unfortunately necessary...
  //
  //                     create_data only takes DynamicType_ptr (aka shared_ptr)
  //                     And passing a heap allocated shared_ptr is the only way to make sure the
  //                     lifetime of the dynamic type is preserved
  return new ser_dynamic_data_t{
    static_cast<fastrtps_ser_impl_t *>(ser_impl->impl)->data_factory_->create_data(
      eprosima::fastrtps::types::DynamicType_ptr(
        *static_cast<eprosima::fastrtps::types::DynamicType_ptr *>(type->impl)
      )
    )
  };
}

ser_dynamic_data_t *
fastrtps__data_clone(ser_impl_t * ser_impl, const ser_dynamic_data_t * data)
{
  return new ser_dynamic_data_t{
    static_cast<fastrtps_ser_impl_t *>(ser_impl->impl)->data_factory_->create_copy(
      static_cast<const DynamicData *>(data->impl))
  };
}


void
fastrtps__data_fini(ser_impl_t * ser_impl, ser_dynamic_data_t * data)
{
  static_cast<fastrtps_ser_impl_t *>(ser_impl->impl)
    ->data_factory_->delete_data(static_cast<DynamicData *>(data->impl));
}


// DYNAMIC DATA PRIMITIVE MEMBER GETTERS ===========================================================
void
fastrtps__get_bool_value(
  ser_impl_t * ser_impl, const ser_dynamic_data_t * data, bool * value, MemberId id)
{
  (void) ser_impl;
  static_cast<const DynamicData *>(data->impl)->get_bool_value(*value, id);
}


void
fastrtps__get_byte_value(
  ser_impl_t * ser_impl, const ser_dynamic_data_t * data, uint8_t * value, MemberId id)
{
  (void) ser_impl;
  static_cast<const DynamicData *>(data->impl)->get_byte_value(*value, id);
}


void
fastrtps__get_char_value(
  ser_impl_t * ser_impl, const ser_dynamic_data_t * data, char * value, MemberId id)
{
  (void) ser_impl;
  static_cast<const DynamicData *>(data->impl)->get_char8_value(*value, id);
}


void
fastrtps__get_float32_value(
  ser_impl_t * ser_impl, const ser_dynamic_data_t * data, float * value, MemberId id)
{
  (void) ser_impl;
  static_cast<const DynamicData *>(data->impl)->get_float32_value(*value, id);
}


void
fastrtps__get_float64_value(
  ser_impl_t * ser_impl, const ser_dynamic_data_t * data, double * value, MemberId id)
{
  (void) ser_impl;
  static_cast<const DynamicData *>(data->impl)->get_float64_value(*value, id);
}


void
fastrtps__get_int8_value(
  ser_impl_t * ser_impl, const ser_dynamic_data_t * data, int8_t * value, MemberId id)
{
  (void) ser_impl;
  static_cast<const DynamicData *>(data->impl)->get_int8_value(*value, id);
}


void
fastrtps__get_uint8_value(
  ser_impl_t * ser_impl, const ser_dynamic_data_t * data, uint8_t * value, MemberId id)
{
  (void) ser_impl;
  static_cast<const DynamicData *>(data->impl)->get_uint8_value(*value, id);
}


void
fastrtps__get_int16_value(
  ser_impl_t * ser_impl, const ser_dynamic_data_t * data, int16_t * value, MemberId id)
{
  (void) ser_impl;
  static_cast<const DynamicData *>(data->impl)->get_int16_value(*value, id);
}


void
fastrtps__get_uint16_value(
  ser_impl_t * ser_impl, const ser_dynamic_data_t * data, uint16_t * value, MemberId id)
{
  (void) ser_impl;
  static_cast<const DynamicData *>(data->impl)->get_uint16_value(*value, id);
}


void
fastrtps__get_int32_value(
  ser_impl_t * ser_impl, const ser_dynamic_data_t * data, int32_t * value, MemberId id)
{
  (void) ser_impl;
  static_cast<const DynamicData *>(data->impl)->get_int32_value(*value, id);
}


void
fastrtps__get_uint32_value(
  ser_impl_t * ser_impl, const ser_dynamic_data_t * data, uint32_t * value, MemberId id)
{
  (void) ser_impl;
  static_cast<const DynamicData *>(data->impl)->get_uint32_value(*value, id);
}


void
fastrtps__get_int64_value(
  ser_impl_t * ser_impl, const ser_dynamic_data_t * data, int64_t * value, MemberId id)
{
  (void) ser_impl;
  static_cast<const DynamicData *>(data->impl)->get_int64_value(*value, id);
}


void
fastrtps__get_uint64_value(
  ser_impl_t * ser_impl, const ser_dynamic_data_t * data, uint64_t * value, MemberId id)
{
  (void) ser_impl;
  static_cast<const DynamicData *>(data->impl)->get_uint64_value(*value, id);
}


void
fastrtps__get_string_value(
  ser_impl_t * ser_impl, const ser_dynamic_data_t * data, const char ** value, MemberId id)
{
  (void) ser_impl;
  std::string tmp;
  static_cast<const DynamicData *>(data->impl)->get_string_value(tmp, id);  // Lifetime is in the data obj
  *value = tmp.c_str();
}


void
fastrtps__get_wstring_value(
  ser_impl_t * ser_impl,
  const ser_dynamic_data_t * data, const wchar_t ** value, MemberId id)
{
  (void) ser_impl;
  std::wstring tmp;
  static_cast<const DynamicData *>(data->impl)->get_wstring_value(tmp, id);
  *value = tmp.c_str();
}


// DYNAMIC DATA PRIMITIVE MEMBER SETTERS ===========================================================
void
fastrtps__set_bool_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, bool value, MemberId id)
{
  (void) ser_impl;
  static_cast<DynamicData *>(data->impl)->set_bool_value(value, id);
}


void
fastrtps__set_byte_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, uint8_t value, MemberId id)
{
  (void) ser_impl;
  static_cast<DynamicData *>(data->impl)->set_byte_value(value, id);
}


void
fastrtps__set_char_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, char value, MemberId id)
{
  (void) ser_impl;
  static_cast<DynamicData *>(data->impl)->set_char8_value(value, id);
}


void
fastrtps__set_float32_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, float value, MemberId id)
{
  (void) ser_impl;
  static_cast<DynamicData *>(data->impl)->set_float32_value(value, id);
}


void
fastrtps__set_float64_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, double value, MemberId id)
{
  (void) ser_impl;
  static_cast<DynamicData *>(data->impl)->set_float64_value(value, id);
}


void
fastrtps__set_int8_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, int8_t value, MemberId id)
{
  (void) ser_impl;
  static_cast<DynamicData *>(data->impl)->set_int8_value(value, id);
}


void
fastrtps__set_uint8_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, uint8_t value, MemberId id)
{
  (void) ser_impl;
  static_cast<DynamicData *>(data->impl)->set_uint8_value(value, id);
}


void
fastrtps__set_int16_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, int16_t value, MemberId id)
{
  (void) ser_impl;
  static_cast<DynamicData *>(data->impl)->set_int16_value(value, id);
}


void
fastrtps__set_uint16_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, uint16_t value, MemberId id)
{
  (void) ser_impl;
  static_cast<DynamicData *>(data->impl)->set_uint16_value(value, id);
}


void
fastrtps__set_int32_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, int32_t value, MemberId id)
{
  (void) ser_impl;
  static_cast<DynamicData *>(data->impl)->set_int32_value(value, id);
}


void
fastrtps__set_uint32_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, uint32_t value, MemberId id)
{
  (void) ser_impl;
  static_cast<DynamicData *>(data->impl)->set_uint32_value(value, id);
}


void
fastrtps__set_int64_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, int64_t value, MemberId id)
{
  (void) ser_impl;
  static_cast<DynamicData *>(data->impl)->set_int64_value(value, id);
}


void
fastrtps__set_uint64_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, uint64_t value, MemberId id)
{
  (void) ser_impl;
  static_cast<DynamicData *>(data->impl)->set_uint64_value(value, id);
}


void
fastrtps__set_string_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, const char * value, MemberId id)
{
  (void) ser_impl;
  const std::string tmp(value);
  // TODO(methylDragon): Check for dealloc
  static_cast<DynamicData *>(data->impl)->set_string_value(tmp, id);
}


void
fastrtps__set_wstring_value(
  ser_impl_t * ser_impl,
  ser_dynamic_data_t * data, const wchar_t * value, MemberId id)
{
  (void) ser_impl;
  const std::wstring tmp(value);
  // TODO(methylDragon): Check for dealloc
  static_cast<DynamicData *>(data->impl)->set_wstring_value(tmp, id);
}


// DYNAMIC DATA SEQUENCES ==========================================================================
void
fastrtps__clear_sequence_data(ser_impl_t * ser_impl, ser_dynamic_data_t * data)
{
  (void) ser_impl;
  static_cast<DynamicData *>(data->impl)->clear_data();
}


void
fastrtps__remove_sequence_data(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, MemberId id)
{
  (void) ser_impl;
  static_cast<DynamicData *>(data->impl)->remove_sequence_data(id);
}


void
fastrtps__insert_sequence_data(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, MemberId * out_id)
{
  (void) ser_impl;
  static_cast<DynamicData *>(data->impl)->insert_sequence_data(*out_id);
}


void
fastrtps__insert_bool_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, bool value, MemberId * out_id)
{
  (void) ser_impl;
  static_cast<DynamicData *>(data->impl)->insert_bool_value(value, *out_id);
}


void
fastrtps__insert_byte_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, uint8_t value, MemberId * out_id)
{
  (void) ser_impl;
  static_cast<DynamicData *>(data->impl)->insert_byte_value(value, *out_id);
}


void
fastrtps__insert_char_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, char value, MemberId * out_id)
{
  (void) ser_impl;
  static_cast<DynamicData *>(data->impl)->insert_char8_value(value, *out_id);
}


void
fastrtps__insert_float32_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, float value, MemberId * out_id)
{
  (void) ser_impl;
  static_cast<DynamicData *>(data->impl)->insert_float32_value(value, *out_id);
}


void
fastrtps__insert_float64_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, double value, MemberId * out_id)
{
  (void) ser_impl;
  static_cast<DynamicData *>(data->impl)->insert_float64_value(value, *out_id);
}


void
fastrtps__insert_int16_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, int16_t value, MemberId * out_id)
{
  (void) ser_impl;
  static_cast<DynamicData *>(data->impl)->insert_int16_value(value, *out_id);
}


void
fastrtps__insert_uint16_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, uint16_t value, MemberId * out_id)
{
  (void) ser_impl;
  static_cast<DynamicData *>(data->impl)->insert_uint16_value(value, *out_id);
}


void
fastrtps__insert_int32_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, int32_t value, MemberId * out_id)
{
  (void) ser_impl;
  static_cast<DynamicData *>(data->impl)->insert_int32_value(value, *out_id);
}


void
fastrtps__insert_uint32_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, uint32_t value, MemberId * out_id)
{
  (void) ser_impl;
  static_cast<DynamicData *>(data->impl)->insert_uint32_value(value, *out_id);
}


void
fastrtps__insert_int64_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, int64_t value, MemberId * out_id)
{
  (void) ser_impl;
  static_cast<DynamicData *>(data->impl)->insert_int64_value(value, *out_id);
}


void
fastrtps__insert_uint64_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, uint64_t value, MemberId * out_id)
{
  (void) ser_impl;
  static_cast<DynamicData *>(data->impl)->insert_uint64_value(value, *out_id);
}


void
fastrtps__insert_string_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, const char * value, MemberId * out_id)
{
  (void) ser_impl;
  static_cast<DynamicData *>(data->impl)->insert_string_value(value, *out_id);
}


void
fastrtps__insert_wstring_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, const wchar_t * value,
  MemberId * out_id)
{
  (void) ser_impl;
  static_cast<DynamicData *>(data->impl)->insert_wstring_value(value, *out_id);
}


// DYNAMIC DATA NESTED =============================================================================
void
fastrtps__get_complex_value(
  ser_impl_t * ser_impl, const ser_dynamic_data_t * data, ser_dynamic_data_t ** value, MemberId id)
{
  (void) ser_impl;
  auto tmp = static_cast<DynamicData *>((*value)->impl);
  // TODO(methylDragon): Check for dealloc
  static_cast<const DynamicData *>(data->impl)->get_complex_value(&tmp, id);
}


void
fastrtps__set_complex_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, ser_dynamic_data_t * value, MemberId id)
{
  (void) ser_impl;
  static_cast<DynamicData *>(data->impl)->set_complex_value(
    static_cast<DynamicData *>(value->impl), id);
}


void
fastrtps__insert_const_complex_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, const ser_dynamic_data_t * value, MemberId * out_id)
{
  (void) ser_impl;
  static_cast<DynamicData *>(data->impl)->insert_complex_value(
    static_cast<const DynamicData *>(value->impl), *out_id);
}


void
fastrtps__insert_complex_value(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, ser_dynamic_data_t * value, MemberId * out_id)
{
  (void) ser_impl;
  static_cast<DynamicData *>(data->impl)->insert_complex_value(
    static_cast<DynamicData *>(value->impl), *out_id);
}


void
fastrtps__insert_complex_value_ptr(
  ser_impl_t * ser_impl, ser_dynamic_data_t * data, ser_dynamic_data_t * value, MemberId * out_id)
{
  (void) ser_impl;
  static_cast<DynamicData *>(data->impl)->insert_complex_value(
    static_cast<DynamicData *>(value->impl), *out_id);
}

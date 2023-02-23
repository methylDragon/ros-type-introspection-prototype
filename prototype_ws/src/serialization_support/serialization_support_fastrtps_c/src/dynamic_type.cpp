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

#include <fastrtps/types/DynamicType.h>
#include <fastrtps/types/DynamicTypePtr.h>
#include <fastrtps/types/DynamicTypeBuilderPtr.h>

#include <serialization_support_fastrtps_c/dynamic_type.h>
#include <serialization_support_fastrtps_c/serialization_impl.h>
#include <serialization_support_lib/description.h>

// using eprosima::fastrtps::types::DynamicType;  // Conflicts in this scope...
using eprosima::fastrtps::types::DynamicType_ptr;
using eprosima::fastrtps::types::DynamicTypeBuilder;
using eprosima::fastrtps::types::DynamicTypeBuilder_ptr;

#define BOUND_UNLIMITED 0


// =================================================================================================
// DYNAMIC TYPE
// =================================================================================================

// DYNAMIC TYPE UTILS =======================================================================
bool
fastrtps__type_equals(
  ser_impl_t * ser_impl, const ser_dynamic_type_t * type, const ser_dynamic_type_t * other)
{
  (void) ser_impl;
  return static_cast<const eprosima::fastrtps::types::DynamicType *>(type->impl)
    ->equals(static_cast<const eprosima::fastrtps::types::DynamicType *>(other->impl));
}


uint32_t
fastrtps__get_type_member_count(ser_impl_t * ser_impl, const ser_dynamic_type_t * type)
{
  (void) ser_impl;
  return static_cast<const eprosima::fastrtps::types::DynamicType *>(type->impl)
    ->get_members_count();
}


// DYNAMIC TYPE CONSTRUCTION =======================================================================
ser_type_builder_t *
fastrtps__struct_type_builder_init(ser_impl_t * ser_impl, const char * name)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  DynamicTypeBuilder * builder = fastrtps_ser_impl->type_factory_->create_struct_builder();
  builder->set_name(name);
  return new ser_type_builder_t{std::move(builder)};
}


void
fastrtps__struct_type_builder_fini(ser_impl_t * ser_impl, ser_type_builder_t * builder)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  fastrtps_ser_impl->type_factory_->delete_builder(
    static_cast<DynamicTypeBuilder *>(builder->impl));
}


ser_dynamic_type_t *
fastrtps__build_struct_type(ser_impl_t * ser_impl, ser_type_builder_t * builder)
{
  (void) ser_impl;

  // Disgusting, but unavoidable... (we can't easily transfer ownership)
  //
  // We're forcing the managed pointer to persist outside of function scope by moving ownership
  // to a new, heap-allocated DynamicType_ptr (which is a shared_ptr)
  return new ser_dynamic_type_t{
    static_cast<void *>(
      new DynamicType_ptr(
        std::move(static_cast<DynamicTypeBuilder *>(builder->impl)->build())
      )
    )
  };
}


ser_dynamic_type_t *
fastrtps__construct_type_from_description(
  ser_impl_t * ser_impl, type_description_t * description)
{
  individual_type_description_t * main_description = description->type_description;

  auto builder = new ser_type_builder_t{std::move(
    static_cast<DynamicTypeBuilder *>(
      fastrtps__struct_type_builder_init(ser_impl, main_description->type_name)->impl
    )
  )};

  for (size_t i = 0; i < main_description->field_count; i++) {
    type_description_field_t * field = main_description->fields[i];

    switch (field->field_type_id) {
      case UNSET_T_ID:
        std::cerr << "Field type not set for field ["
                  << field->field_name << "]" << std::endl;
        return nullptr;
        break;

      // PRIMITIVES
      case BOOL_T_ID:
        fastrtps__add_bool_member(ser_impl, builder, i, field->field_name);
        break;
      case BYTE_T_ID:
        fastrtps__add_byte_member(ser_impl, builder, i, field->field_name);
        break;
      case CHAR_T_ID:
        fastrtps__add_char_member(ser_impl, builder, i, field->field_name);
        break;
      case FLOAT_32_T_ID:
        fastrtps__add_float32_member(ser_impl, builder, i, field->field_name);
        break;
      case FLOAT_64_T_ID:
        fastrtps__add_float64_member(ser_impl, builder, i, field->field_name);
        break;
      case INT_8_T_ID:
        fastrtps__add_int8_member(ser_impl, builder, i, field->field_name);
        break;
      case UINT_8_T_ID:
        fastrtps__add_uint8_member(ser_impl, builder, i, field->field_name);
        break;
      case INT_16_T_ID:
        fastrtps__add_int16_member(ser_impl, builder, i, field->field_name);
        break;
      case UINT_16_T_ID:
        fastrtps__add_uint16_member(ser_impl, builder, i, field->field_name);
        break;
      case INT_32_T_ID:
        fastrtps__add_int32_member(ser_impl, builder, i, field->field_name);
        break;
      case UINT_32_T_ID:
        fastrtps__add_uint32_member(ser_impl, builder, i, field->field_name);
        break;
      case INT_64_T_ID:
        fastrtps__add_int64_member(ser_impl, builder, i, field->field_name);
        break;
      case UINT_64_T_ID:
        fastrtps__add_uint64_member(ser_impl, builder, i, field->field_name);
        break;
      case STRING_T_ID:
        fastrtps__add_string_member(ser_impl, builder, i, field->field_name);
        break;
      case WSTRING_T_ID:
        fastrtps__add_wstring_member(ser_impl, builder, i, field->field_name);
        break;
      case BOUNDED_STRING_T_ID:
        fastrtps__add_bounded_string_member(
          ser_impl, builder, i, field->field_name,
          field->field_length);
        break;
      case BOUNDED_WSTRING_T_ID:
        fastrtps__add_bounded_wstring_member(
          ser_impl, builder, i, field->field_name,
          field->field_length);
        break;

      // STATIC ARRAYS
      case BOOL_T_ID + STATIC_ARRAY_OFFSET:
        fastrtps__add_bool_static_array_member(
          ser_impl, builder, i, field->field_name, field->field_length);
        break;
      case BYTE_T_ID + STATIC_ARRAY_OFFSET:
        fastrtps__add_byte_static_array_member(
          ser_impl, builder, i, field->field_name, field->field_length);
        break;
      case CHAR_T_ID + STATIC_ARRAY_OFFSET:
        fastrtps__add_char_static_array_member(
          ser_impl, builder, i, field->field_name, field->field_length);
        break;
      case FLOAT_32_T_ID + STATIC_ARRAY_OFFSET:
        fastrtps__add_float32_static_array_member(
          ser_impl, builder, i, field->field_name, field->field_length);
        break;
      case FLOAT_64_T_ID + STATIC_ARRAY_OFFSET:
        fastrtps__add_float64_static_array_member(
          ser_impl, builder, i, field->field_name, field->field_length);
        break;
      case INT_8_T_ID + STATIC_ARRAY_OFFSET:
        fastrtps__add_int8_static_array_member(
          ser_impl, builder, i, field->field_name, field->field_length);
        break;
      case UINT_8_T_ID + STATIC_ARRAY_OFFSET:
        fastrtps__add_uint8_static_array_member(
          ser_impl, builder, i, field->field_name, field->field_length);
        break;
      case INT_16_T_ID + STATIC_ARRAY_OFFSET:
        fastrtps__add_int16_static_array_member(
          ser_impl, builder, i, field->field_name, field->field_length);
        break;
      case UINT_16_T_ID + STATIC_ARRAY_OFFSET:
        fastrtps__add_uint16_static_array_member(
          ser_impl, builder, i, field->field_name, field->field_length);
        break;
      case INT_32_T_ID + STATIC_ARRAY_OFFSET:
        fastrtps__add_int32_static_array_member(
          ser_impl, builder, i, field->field_name, field->field_length);
        break;
      case UINT_32_T_ID + STATIC_ARRAY_OFFSET:
        fastrtps__add_uint32_static_array_member(
          ser_impl, builder, i, field->field_name, field->field_length);
        break;
      case INT_64_T_ID + STATIC_ARRAY_OFFSET:
        fastrtps__add_int64_static_array_member(
          ser_impl, builder, i, field->field_name, field->field_length);
        break;
      case UINT_64_T_ID + STATIC_ARRAY_OFFSET:
        fastrtps__add_uint64_static_array_member(
          ser_impl, builder, i, field->field_name, field->field_length);
        break;
      case STRING_T_ID + STATIC_ARRAY_OFFSET:
        fastrtps__add_string_static_array_member(
          ser_impl, builder, i, field->field_name, field->field_length);
        break;
      case WSTRING_T_ID + STATIC_ARRAY_OFFSET:
        fastrtps__add_wstring_static_array_member(
          ser_impl, builder, i, field->field_name, field->field_length);
        break;
      case BOUNDED_STRING_T_ID + STATIC_ARRAY_OFFSET:
        fastrtps__add_bounded_string_static_array_member(
          ser_impl, builder, i, field->field_name,
          field->field_string_length, field->field_length);
        break;
      case BOUNDED_WSTRING_T_ID + STATIC_ARRAY_OFFSET:
        fastrtps__add_bounded_wstring_static_array_member(
          ser_impl, builder, i, field->field_name,
          field->field_string_length, field->field_length);
        break;

      // UNBOUNDED SEQUENCES
      case BOOL_T_ID + UNBOUNDED_SEQ_OFFSET:
        fastrtps__add_bool_unbounded_sequence_member(
          ser_impl, builder, i, field->field_name);
        break;
      case BYTE_T_ID + UNBOUNDED_SEQ_OFFSET:
        fastrtps__add_byte_unbounded_sequence_member(
          ser_impl, builder, i, field->field_name);
        break;
      case CHAR_T_ID + UNBOUNDED_SEQ_OFFSET:
        fastrtps__add_char_unbounded_sequence_member(
          ser_impl, builder, i, field->field_name);
        break;
      case FLOAT_32_T_ID + UNBOUNDED_SEQ_OFFSET:
        fastrtps__add_float32_unbounded_sequence_member(
          ser_impl, builder, i, field->field_name);
        break;
      case FLOAT_64_T_ID + UNBOUNDED_SEQ_OFFSET:
        fastrtps__add_float64_unbounded_sequence_member(
          ser_impl, builder, i, field->field_name);
        break;
      case INT_8_T_ID + UNBOUNDED_SEQ_OFFSET:
        fastrtps__add_int8_unbounded_sequence_member(
          ser_impl, builder, i, field->field_name);
        break;
      case UINT_8_T_ID + UNBOUNDED_SEQ_OFFSET:
        fastrtps__add_uint8_unbounded_sequence_member(
          ser_impl, builder, i, field->field_name);
        break;
      case INT_16_T_ID + UNBOUNDED_SEQ_OFFSET:
        fastrtps__add_int16_unbounded_sequence_member(
          ser_impl, builder, i, field->field_name);
        break;
      case UINT_16_T_ID + UNBOUNDED_SEQ_OFFSET:
        fastrtps__add_uint16_unbounded_sequence_member(
          ser_impl, builder, i, field->field_name);
        break;
      case INT_32_T_ID + UNBOUNDED_SEQ_OFFSET:
        fastrtps__add_int32_unbounded_sequence_member(
          ser_impl, builder, i, field->field_name);
        break;
      case UINT_32_T_ID + UNBOUNDED_SEQ_OFFSET:
        fastrtps__add_uint32_unbounded_sequence_member(
          ser_impl, builder, i, field->field_name);
        break;
      case INT_64_T_ID + UNBOUNDED_SEQ_OFFSET:
        fastrtps__add_int64_unbounded_sequence_member(
          ser_impl, builder, i, field->field_name);
        break;
      case UINT_64_T_ID + UNBOUNDED_SEQ_OFFSET:
        fastrtps__add_uint64_unbounded_sequence_member(
          ser_impl, builder, i, field->field_name);
        break;
      case STRING_T_ID + UNBOUNDED_SEQ_OFFSET:
        fastrtps__add_string_unbounded_sequence_member(
          ser_impl, builder, i, field->field_name);
        break;
      case WSTRING_T_ID + UNBOUNDED_SEQ_OFFSET:
        fastrtps__add_wstring_unbounded_sequence_member(
          ser_impl, builder, i, field->field_name);
        break;
      case BOUNDED_STRING_T_ID + UNBOUNDED_SEQ_OFFSET:
        fastrtps__add_bounded_string_unbounded_sequence_member(
          ser_impl, builder, i, field->field_name, field->field_string_length);
        break;
      case BOUNDED_WSTRING_T_ID + UNBOUNDED_SEQ_OFFSET:
        fastrtps__add_bounded_wstring_unbounded_sequence_member(
          ser_impl, builder, i, field->field_name, field->field_string_length);
        break;

      // BOUNDED SEQUENCES
      case BOOL_T_ID + BOUNDED_SEQ_OFFSET:
        fastrtps__add_bool_static_array_member(
          ser_impl, builder, i, field->field_name, field->field_length);
        break;
      case BYTE_T_ID + BOUNDED_SEQ_OFFSET:
        fastrtps__add_byte_static_array_member(
          ser_impl, builder, i, field->field_name, field->field_length);
        break;
      case CHAR_T_ID + BOUNDED_SEQ_OFFSET:
        fastrtps__add_char_static_array_member(
          ser_impl, builder, i, field->field_name, field->field_length);
        break;
      case FLOAT_32_T_ID + BOUNDED_SEQ_OFFSET:
        fastrtps__add_float32_static_array_member(
          ser_impl, builder, i, field->field_name, field->field_length);
        break;
      case FLOAT_64_T_ID + BOUNDED_SEQ_OFFSET:
        fastrtps__add_float64_static_array_member(
          ser_impl, builder, i, field->field_name, field->field_length);
        break;
      case INT_8_T_ID + BOUNDED_SEQ_OFFSET:
        fastrtps__add_int8_static_array_member(
          ser_impl, builder, i, field->field_name, field->field_length);
        break;
      case UINT_8_T_ID + BOUNDED_SEQ_OFFSET:
        fastrtps__add_uint8_static_array_member(
          ser_impl, builder, i, field->field_name, field->field_length);
        break;
      case INT_16_T_ID + BOUNDED_SEQ_OFFSET:
        fastrtps__add_int16_static_array_member(
          ser_impl, builder, i, field->field_name, field->field_length);
        break;
      case UINT_16_T_ID + BOUNDED_SEQ_OFFSET:
        fastrtps__add_uint16_static_array_member(
          ser_impl, builder, i, field->field_name, field->field_length);
        break;
      case INT_32_T_ID + BOUNDED_SEQ_OFFSET:
        fastrtps__add_int32_static_array_member(
          ser_impl, builder, i, field->field_name, field->field_length);
        break;
      case UINT_32_T_ID + BOUNDED_SEQ_OFFSET:
        fastrtps__add_uint32_static_array_member(
          ser_impl, builder, i, field->field_name, field->field_length);
        break;
      case INT_64_T_ID + BOUNDED_SEQ_OFFSET:
        fastrtps__add_int64_static_array_member(
          ser_impl, builder, i, field->field_name, field->field_length);
        break;
      case UINT_64_T_ID + BOUNDED_SEQ_OFFSET:
        fastrtps__add_uint64_static_array_member(
          ser_impl, builder, i, field->field_name, field->field_length);
        break;
      case STRING_T_ID + BOUNDED_SEQ_OFFSET:
        fastrtps__add_string_static_array_member(
          ser_impl, builder, i, field->field_name, field->field_length);
        break;
      case WSTRING_T_ID + BOUNDED_SEQ_OFFSET:
        fastrtps__add_wstring_static_array_member(
          ser_impl, builder, i, field->field_name, field->field_length);
        break;
      case BOUNDED_STRING_T_ID + BOUNDED_SEQ_OFFSET:
        fastrtps__add_bounded_string_static_array_member(
          ser_impl, builder, i, field->field_name,
          field->field_string_length, field->field_length);
        break;
      case BOUNDED_WSTRING_T_ID + BOUNDED_SEQ_OFFSET:
        fastrtps__add_bounded_wstring_static_array_member(
          ser_impl, builder, i, field->field_name,
          field->field_string_length, field->field_length);
        break;

      // NESTED
      case NESTED_T_ID:
      case NESTED_T_ID + STATIC_ARRAY_OFFSET:
      case NESTED_T_ID + UNBOUNDED_SEQ_OFFSET:
      case NESTED_T_ID + BOUNDED_SEQ_OFFSET:
        {
          if (field->nested_type_name == NULL) {
            std::cerr << "Nested type name is missing in description for field ["
                      << field->field_name << "]" << std::endl;
            return nullptr;
          }

          // Create a new type_description_t to pass to the next layer
          std::shared_ptr<type_description_t> recurse_description(
            std::move(get_ref_description_as_type_description(description, field->nested_type_name))
          );

          if (recurse_description.get() == NULL) {
            return nullptr;
          }

          // Recurse
          auto nested_struct = fastrtps__construct_type_from_description(
            ser_impl, recurse_description.get());

          if (nested_struct == NULL) {
            std::cerr << "Could not construct nested type for field ["
                      << field->field_name << "]" << std::endl;
            return nullptr;
          }

          switch (field->field_type_id) {
            case NESTED_T_ID:
              fastrtps__add_nested_struct_member(
                ser_impl, builder, i, field->field_name, nested_struct);
              break;

            case NESTED_T_ID + STATIC_ARRAY_OFFSET:
              fastrtps__add_nested_struct_static_array_member(
                ser_impl,
                builder, i, field->field_name, nested_struct, field->field_length);
              break;

            case NESTED_T_ID + UNBOUNDED_SEQ_OFFSET:
              fastrtps__add_nested_struct_unbounded_sequence_member(
                ser_impl, builder, i, field->field_name, nested_struct);
              break;

            case NESTED_T_ID + BOUNDED_SEQ_OFFSET:
              fastrtps__add_nested_struct_bounded_sequence_member(
                ser_impl,
                builder, i, field->field_name, nested_struct, field->field_length);
              break;
          }
        }
        break;

      default:
        printf("[ERROR] Invalid field type!");
        break;
    }
  }

  auto out = fastrtps__build_struct_type(ser_impl, builder);
  fastrtps__struct_type_builder_fini(ser_impl, builder);
  return out;
}


void
fastrtps__type_fini(ser_impl_t * ser_impl, ser_dynamic_type_t * type)
{
  // You typically don't need to call this because the DynamicType_ptr should manage the
  // destruction for you
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  fastrtps_ser_impl->type_factory_->delete_type(
    static_cast<eprosima::fastrtps::types::DynamicType_ptr *>(type->impl)->get());
}


// DYNAMIC TYPE PRIMITIVE MEMBERS ==================================================================
void
fastrtps__add_bool_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id, name, fastrtps_ser_impl->type_factory_->create_bool_type()
  );
}


void
fastrtps__add_byte_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id, name, fastrtps_ser_impl->type_factory_->create_byte_type()
  );
}


void
fastrtps__add_char_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id, name, fastrtps_ser_impl->type_factory_->create_char8_type()
  );
}


void
fastrtps__add_float32_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id, name, fastrtps_ser_impl->type_factory_->create_float32_type()
  );
}


void
fastrtps__add_float64_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id, name, fastrtps_ser_impl->type_factory_->create_float64_type()
  );
}


void
fastrtps__add_int8_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name)
{
  fastrtps__add_byte_member(ser_impl, builder, id, name);
}


void
fastrtps__add_uint8_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name)
{
  fastrtps__add_byte_member(ser_impl, builder, id, name);
}


void
fastrtps__add_int16_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id, name, fastrtps_ser_impl->type_factory_->create_int16_type()
  );
}


void
fastrtps__add_uint16_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id, name, fastrtps_ser_impl->type_factory_->create_uint16_type()
  );
}


void
fastrtps__add_int32_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id, name, fastrtps_ser_impl->type_factory_->create_int32_type()
  );
}


void
fastrtps__add_uint32_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id, name, fastrtps_ser_impl->type_factory_->create_uint32_type()
  );
}


void
fastrtps__add_int64_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id, name, fastrtps_ser_impl->type_factory_->create_int64_type()
  );
}


void
fastrtps__add_uint64_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id, name, fastrtps_ser_impl->type_factory_->create_uint64_type()
  );
}


void
fastrtps__add_string_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name)
{
  fastrtps__add_bounded_string_member(
    ser_impl, builder, id, name, BOUND_UNLIMITED);
}


void
fastrtps__add_wstring_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name)
{
  fastrtps__add_bounded_wstring_member(
    ser_impl, builder, id, name, BOUND_UNLIMITED);
}


void
fastrtps__add_bounded_string_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder,
  MemberId id, const char * name, uint32_t bound)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id, name, fastrtps_ser_impl->type_factory_->create_string_type(bound));
}


void
fastrtps__add_bounded_wstring_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder,
  MemberId id, const char * name, uint32_t bound)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id, name, fastrtps_ser_impl->type_factory_->create_wstring_type(bound));
}


// DYNAMIC TYPE STATIC ARRAY MEMBERS ===============================================================
void
fastrtps__add_bool_static_array_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id, name, fastrtps_ser_impl->type_factory_->create_array_builder(
      fastrtps_ser_impl->type_factory_->create_bool_type(), {bound})
  );
}


void
fastrtps__add_byte_static_array_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id, name, fastrtps_ser_impl->type_factory_->create_array_builder(
      fastrtps_ser_impl->type_factory_->create_byte_type(), {bound})
  );
}


void
fastrtps__add_char_static_array_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id, name, fastrtps_ser_impl->type_factory_->create_array_builder(
      fastrtps_ser_impl->type_factory_->create_char8_type(), {bound})
  );
}


void
fastrtps__add_float32_static_array_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id, name, fastrtps_ser_impl->type_factory_->create_array_builder(
      fastrtps_ser_impl->type_factory_->create_float32_type(), {bound})
  );
}


void
fastrtps__add_float64_static_array_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id, name, fastrtps_ser_impl->type_factory_->create_array_builder(
      fastrtps_ser_impl->type_factory_->create_float64_type(), {bound})
  );
}


void
fastrtps__add_int8_static_array_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound)
{
  fastrtps__add_byte_static_array_member(ser_impl, builder, id, name, bound);
}


void
fastrtps__add_uint8_static_array_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound)
{
  fastrtps__add_byte_static_array_member(ser_impl, builder, id, name, bound);
}


void
fastrtps__add_int16_static_array_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id,
    name,
    fastrtps_ser_impl->type_factory_->create_array_builder(
      fastrtps_ser_impl->type_factory_->create_int16_type(), {bound}
    )
  );
}


void
fastrtps__add_uint16_static_array_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id,
    name,
    fastrtps_ser_impl->type_factory_->create_array_builder(
      fastrtps_ser_impl->type_factory_->create_uint16_type(), {bound}
    )
  );
}


void
fastrtps__add_int32_static_array_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id,
    name,
    fastrtps_ser_impl->type_factory_->create_array_builder(
      fastrtps_ser_impl->type_factory_->create_int32_type(), {bound}
    )
  );
}


void
fastrtps__add_uint32_static_array_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id,
    name,
    fastrtps_ser_impl->type_factory_->create_array_builder(
      fastrtps_ser_impl->type_factory_->create_uint32_type(), {bound}
    )
  );
}


void
fastrtps__add_int64_static_array_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id,
    name,
    fastrtps_ser_impl->type_factory_->create_array_builder(
      fastrtps_ser_impl->type_factory_->create_int64_type(), {bound}
    )
  );
}


void
fastrtps__add_uint64_static_array_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id,
    name,
    fastrtps_ser_impl->type_factory_->create_array_builder(
      fastrtps_ser_impl->type_factory_->create_uint64_type(), {bound}
    )
  );
}


void
fastrtps__add_string_static_array_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound)
{
  fastrtps__add_bounded_string_static_array_member(
    ser_impl, builder, id, name, BOUND_UNLIMITED, bound);
}


void
fastrtps__add_wstring_static_array_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound)
{
  fastrtps__add_bounded_wstring_static_array_member(
    ser_impl, builder, id, name, BOUND_UNLIMITED, bound);
}


void
fastrtps__add_bounded_string_static_array_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t str_bound, uint32_t bound)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id,
    name,
    fastrtps_ser_impl->type_factory_->create_array_builder(
      fastrtps_ser_impl->type_factory_->create_string_type(str_bound), {bound}
    )
  );
}


void
fastrtps__add_bounded_wstring_static_array_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t str_bound, uint32_t bound)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id,
    name,
    fastrtps_ser_impl->type_factory_->create_array_builder(
      fastrtps_ser_impl->type_factory_->create_wstring_type(str_bound), {bound}
    )
  );
}


// DYNAMIC TYPE UNBOUNDED SEQUENCE MEMBERS =========================================================
void
fastrtps__add_bool_unbounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name)
{
  fastrtps__add_bool_bounded_sequence_member(ser_impl, builder, id, name, BOUND_UNLIMITED);
}


void
fastrtps__add_byte_unbounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name)
{
  fastrtps__add_byte_bounded_sequence_member(ser_impl, builder, id, name, BOUND_UNLIMITED);
}


void
fastrtps__add_char_unbounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name)
{
  fastrtps__add_char_bounded_sequence_member(ser_impl, builder, id, name, BOUND_UNLIMITED);
}


void
fastrtps__add_float32_unbounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name)
{
  fastrtps__add_float32_bounded_sequence_member(ser_impl, builder, id, name, BOUND_UNLIMITED);
}


void
fastrtps__add_float64_unbounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name)
{
  fastrtps__add_float64_bounded_sequence_member(ser_impl, builder, id, name, BOUND_UNLIMITED);
}


void
fastrtps__add_int8_unbounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name)
{
  fastrtps__add_int8_bounded_sequence_member(ser_impl, builder, id, name, BOUND_UNLIMITED);
}

void
fastrtps__add_uint8_unbounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name)
{
  fastrtps__add_uint8_bounded_sequence_member(ser_impl, builder, id, name, BOUND_UNLIMITED);
}


void
fastrtps__add_int16_unbounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name)
{
  fastrtps__add_int16_bounded_sequence_member(ser_impl, builder, id, name, BOUND_UNLIMITED);
}


void
fastrtps__add_uint16_unbounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name)
{
  fastrtps__add_uint16_bounded_sequence_member(ser_impl, builder, id, name, BOUND_UNLIMITED);
}


void
fastrtps__add_int32_unbounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name)
{
  fastrtps__add_int32_bounded_sequence_member(ser_impl, builder, id, name, BOUND_UNLIMITED);
}


void
fastrtps__add_uint32_unbounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name)
{
  fastrtps__add_uint32_bounded_sequence_member(ser_impl, builder, id, name, BOUND_UNLIMITED);
}


void
fastrtps__add_int64_unbounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name)
{
  fastrtps__add_int64_bounded_sequence_member(ser_impl, builder, id, name, BOUND_UNLIMITED);
}


void
fastrtps__add_uint64_unbounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name)
{
  fastrtps__add_uint64_bounded_sequence_member(ser_impl, builder, id, name, BOUND_UNLIMITED);
}


void
fastrtps__add_string_unbounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name)
{
  fastrtps__add_string_bounded_sequence_member(ser_impl, builder, id, name, BOUND_UNLIMITED);
}


void
fastrtps__add_wstring_unbounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name)
{
  fastrtps__add_wstring_bounded_sequence_member(ser_impl, builder, id, name, BOUND_UNLIMITED);
}


void
fastrtps__add_bounded_string_unbounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t str_bound)
{
  fastrtps__add_bounded_string_bounded_sequence_member(
    ser_impl, builder, id, name, str_bound, BOUND_UNLIMITED);
}


void
fastrtps__add_bounded_wstring_unbounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t str_bound)
{
  fastrtps__add_bounded_wstring_bounded_sequence_member(
    ser_impl, builder, id, name, str_bound, BOUND_UNLIMITED);
}


// DYNAMIC TYPE BOUNDED SEQUENCE MEMBERS ===========================================================
void
fastrtps__add_bool_bounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id,
    name,
    fastrtps_ser_impl->type_factory_->create_sequence_builder(
      fastrtps_ser_impl->type_factory_->create_bool_type(), {bound}
    )
  );
}


void
fastrtps__add_byte_bounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id,
    name,
    fastrtps_ser_impl->type_factory_->create_sequence_builder(
      fastrtps_ser_impl->type_factory_->create_byte_type(), {bound}
    )
  );
}


void
fastrtps__add_char_bounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id,
    name,
    fastrtps_ser_impl->type_factory_->create_sequence_builder(
      fastrtps_ser_impl->type_factory_->create_char8_type(), {bound}
    )
  );
}


void
fastrtps__add_float32_bounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id,
    name,
    fastrtps_ser_impl->type_factory_->create_sequence_builder(
      fastrtps_ser_impl->type_factory_->create_float32_type(), {bound}
    )
  );
}


void
fastrtps__add_float64_bounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id,
    name,
    fastrtps_ser_impl->type_factory_->create_sequence_builder(
      fastrtps_ser_impl->type_factory_->create_float64_type(), {bound}
    )
  );
}


void
fastrtps__add_int8_bounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound)
{
  fastrtps__add_byte_bounded_sequence_member(ser_impl, builder, id, name, bound);
}


void
fastrtps__add_uint8_bounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound)
{
  fastrtps__add_byte_bounded_sequence_member(ser_impl, builder, id, name, bound);
}


void
fastrtps__add_int16_bounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id,
    name,
    fastrtps_ser_impl->type_factory_->create_sequence_builder(
      fastrtps_ser_impl->type_factory_->create_int16_type(), {bound}
    )
  );
}


void
fastrtps__add_uint16_bounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id,
    name,
    fastrtps_ser_impl->type_factory_->create_sequence_builder(
      fastrtps_ser_impl->type_factory_->create_uint16_type(), {bound}
    )
  );
}


void
fastrtps__add_int32_bounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id,
    name,
    fastrtps_ser_impl->type_factory_->create_sequence_builder(
      fastrtps_ser_impl->type_factory_->create_int32_type(), {bound})
  );
}


void
fastrtps__add_uint32_bounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id,
    name,
    fastrtps_ser_impl->type_factory_->create_sequence_builder(
      fastrtps_ser_impl->type_factory_->create_uint32_type(), {bound}
    )
  );
}


void
fastrtps__add_int64_bounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id,
    name,
    fastrtps_ser_impl->type_factory_->create_sequence_builder(
      fastrtps_ser_impl->type_factory_->create_int64_type(), {bound}
    )
  );
}


void
fastrtps__add_uint64_bounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id,
    name,
    fastrtps_ser_impl->type_factory_->create_sequence_builder(
      fastrtps_ser_impl->type_factory_->create_uint64_type(), {bound}
    )
  );
}


void
fastrtps__add_string_bounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound)
{
  fastrtps__add_bounded_string_bounded_sequence_member(
    ser_impl, builder, id, name, BOUND_UNLIMITED, bound);
}


void
fastrtps__add_wstring_bounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t bound)
{
  fastrtps__add_bounded_wstring_bounded_sequence_member(
    ser_impl, builder, id, name, BOUND_UNLIMITED, bound);
}


void
fastrtps__add_bounded_string_bounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t str_bound, uint32_t bound)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id,
    name,
    fastrtps_ser_impl->type_factory_->create_sequence_builder(
      fastrtps_ser_impl->type_factory_->create_string_type(str_bound), {bound}
    )
  );
}


void
fastrtps__add_bounded_wstring_bounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  uint32_t str_bound, uint32_t bound)
{
  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id,
    name,
    fastrtps_ser_impl->type_factory_->create_sequence_builder(
      fastrtps_ser_impl->type_factory_->create_wstring_type(str_bound), {bound}
    )
  );
}


// DYNAMIC TYPE NESTED MEMBERS =====================================================================
void
fastrtps__add_nested_struct_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder,
  MemberId id, const char * name, ser_dynamic_type_t * nested_struct)
{
  (void) ser_impl;

  auto nested_struct_dynamictype_ptr = eprosima::fastrtps::types::DynamicType_ptr(
    *static_cast<eprosima::fastrtps::types::DynamicType_ptr *>(nested_struct->impl)
  );

  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id, name, nested_struct_dynamictype_ptr);
}


void
fastrtps__add_nested_struct_static_array_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  ser_dynamic_type_t * nested_struct, uint32_t bound)
{
  auto nested_struct_dynamictype_ptr = eprosima::fastrtps::types::DynamicType_ptr(
    *static_cast<eprosima::fastrtps::types::DynamicType_ptr *>(nested_struct->impl)
  );

  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id, name, fastrtps_ser_impl->type_factory_->create_array_builder(
      nested_struct_dynamictype_ptr, {bound}));
}


void
fastrtps__add_nested_struct_unbounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  ser_dynamic_type_t * nested_struct)
{
  fastrtps__add_nested_struct_bounded_sequence_member(
    ser_impl, builder, id, name, nested_struct, BOUND_UNLIMITED);
}


void
fastrtps__add_nested_struct_bounded_sequence_member(
  ser_impl_t * ser_impl, ser_type_builder_t * builder, MemberId id, const char * name,
  ser_dynamic_type_t * nested_struct, uint32_t bound)
{
  auto nested_struct_dynamictype_ptr = eprosima::fastrtps::types::DynamicType_ptr(
    *static_cast<eprosima::fastrtps::types::DynamicType_ptr *>(nested_struct->impl)
  );

  auto fastrtps_ser_impl = static_cast<fastrtps_ser_impl_t *>(ser_impl->impl);
  static_cast<DynamicTypeBuilder *>(builder->impl)->add_member(
    id,
    name,
    fastrtps_ser_impl->type_factory_->create_sequence_builder(
      nested_struct_dynamictype_ptr, {bound}
    )
  );
}

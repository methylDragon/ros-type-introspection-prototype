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

#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>

#include <fastrtps/types/DynamicDataFactory.h>
#include <fastrtps/types/DynamicDataHelper.hpp>

#include <fastrtps/types/DynamicTypeBuilderFactory.h>
#include <fastrtps/types/DynamicTypeBuilderPtr.h>

#include <fastrtps/types/DynamicTypePtr.h>

#include <evolving_serialization_lib/evolving_type_support.h>
#include "evolving_serialization_lib/types.h"

#include <evolving_fastrtps_c/type_support.h>

using namespace eprosima::fastrtps::types;


// CORE ============================================================================================
struct EvolvingFastRtpsTypeSupportImpl_s
{
  DynamicTypeBuilderFactory * factory_;
};


EvolvingFastRtpsTypeSupportImpl *
create_fastrtps_evolving_typesupport_impl()
{
  EvolvingFastRtpsTypeSupportImpl * ets_impl =
    (EvolvingFastRtpsTypeSupportImpl *) malloc(sizeof(EvolvingFastRtpsTypeSupportImpl));

  // The actual business
  ets_impl->factory_ = eprosima::fastrtps::types::DynamicTypeBuilderFactory::get_instance();

  return ets_impl;
}


EvolvingTypeSupportInterface *
create_fastrtps_evolving_typesupport_interface()
{
  EvolvingTypeSupportInterface * fastrtps_ets =
    (EvolvingTypeSupportInterface *) calloc(1, sizeof(EvolvingTypeSupportInterface));

  // DYNAMIC TYPE METHODS ==========================================================================
  // DYNAMIC TYPE CONSTRUCTION
  fastrtps_ets->create_struct_builder =
    (void * (*)(void *, const char *))fastrtps__create_struct_builder;

  fastrtps_ets->finalize_struct_builder =
    (void * (*)(void *, void *))fastrtps__finalize_struct_builder;

  fastrtps_ets->construct_type_from_description =
    (void * (*)(void *, type_description_t *))fastrtps__construct_type_from_description;

  // DYNAMIC TYPE PRIMITIVE MEMBERS
  fastrtps_ets->add_bool_member =
    (void (*)(void *, void *, uint32_t, const char *))fastrtps__add_bool_member;

  fastrtps_ets->add_byte_member =
    (void (*)(void *, void *, uint32_t, const char *))fastrtps__add_byte_member;

  fastrtps_ets->add_char_member =
    (void (*)(void *, void *, uint32_t, const char *))fastrtps__add_char_member;

  fastrtps_ets->add_float32_member =
    (void (*)(void *, void *, uint32_t, const char *))fastrtps__add_float32_member;

  fastrtps_ets->add_float64_member =
    (void (*)(void *, void *, uint32_t, const char *))fastrtps__add_float64_member;

  fastrtps_ets->add_int8_member =
    (void (*)(void *, void *, uint32_t, const char *))fastrtps__add_int8_member;

  fastrtps_ets->add_uint8_member =
    (void (*)(void *, void *, uint32_t, const char *))fastrtps__add_uint8_member;

  fastrtps_ets->add_int16_member =
    (void (*)(void *, void *, uint32_t, const char *))fastrtps__add_int16_member;

  fastrtps_ets->add_uint16_member =
    (void (*)(void *, void *, uint32_t, const char *))fastrtps__add_uint16_member;

  fastrtps_ets->add_int32_member =
    (void (*)(void *, void *, uint32_t, const char *))fastrtps__add_int32_member;

  fastrtps_ets->add_uint32_member =
    (void (*)(void *, void *, uint32_t, const char *))fastrtps__add_uint32_member;

  fastrtps_ets->add_int64_member =
    (void (*)(void *, void *, uint32_t, const char *))fastrtps__add_int64_member;

  fastrtps_ets->add_uint64_member =
    (void (*)(void *, void *, uint32_t, const char *))fastrtps__add_uint64_member;

  fastrtps_ets->add_string_member =
    (void (*)(void *, void *, uint32_t, const char *))fastrtps__add_string_member;

  fastrtps_ets->add_wstring_member =
    (void (*)(void *, void *, uint32_t, const char *))fastrtps__add_wstring_member;

  fastrtps_ets->add_bounded_string_member =
    (void (*)(void *, void *, uint32_t, const char *, uint32_t))
    fastrtps__add_bounded_string_member;

  fastrtps_ets->add_bounded_wstring_member =
    (void (*)(void *, void *, uint32_t, const char *, uint32_t))
    fastrtps__add_bounded_wstring_member;

  // DYNAMIC TYPE NESTED MEMBERS
  fastrtps_ets->add_nested_struct_member =
    (void (*)(void *, void *, uint32_t, const char *, void *))
    fastrtps__add_nested_struct_member;

  // DYNAMIC DATA UTILS
  fastrtps_ets->print_dynamic_data = (void (*)(void *, void *))fastrtps__print_dynamic_data;

  return fastrtps_ets;
}


// DYNAMIC TYPE CONSTRUCTION =======================================================================
void *
fastrtps__create_struct_builder(EvolvingFastRtpsTypeSupportImpl * ets_impl, const char * name)
{
  DynamicTypeBuilder * builder = ets_impl->factory_->create_struct_builder();
  builder->set_name(name);

  return builder;
}

void *
fastrtps__finalize_struct_builder(EvolvingFastRtpsTypeSupportImpl * ets_impl, void * builder)
{
  (void) ets_impl;

  // Disgusting, but unavoidable...
  //
  // We're forcing the managed pointer to persist outside of function scope by moving ownership
  // to a new, heap-allocated DynamicType_ptr (which is a shared_ptr)
  return reinterpret_cast<void *>(
    new DynamicType_ptr(
      std::move(static_cast<DynamicTypeBuilder *>(builder)->build())
    )
  );
}

void *
fastrtps__construct_type_from_description(
  EvolvingFastRtpsTypeSupportImpl * ets_impl, type_description_t * description)
{
  individual_type_description_t * main_description = description->type_description;

  auto builder = DynamicTypeBuilder_ptr(
    std::move(
      static_cast<DynamicTypeBuilder *>(
        fastrtps__create_struct_builder(ets_impl, main_description->type_name)
      )
    )
  );

  for (size_t i = 0; i < main_description->field_count; i++) {
    type_description_field_t * field = main_description->fields[i];

    switch (field->field_type) {
      case UNSET_T_IDX:
        printf("[ERROR] Field type not set!");
        break;

      // NESTED
      case NESTED_T_IDX:
        {
          if (field->nested_type_name == NULL) {
            std::cerr << "Nested type name is missing in description for field ["
                      << field->field_name << "]" << std::endl;
            return nullptr;
          }

          // Ensure referenced type exists
          auto ref_lookup =
            g_hash_table_lookup(description->referenced_type_descriptions, field->nested_type_name);

          if (ref_lookup == NULL) {
            std::cerr << "Referenced type description: [" << field->nested_type_name
                      << "] could not be found in description!" << std::endl;
            return nullptr;
          }

          // Create a new type_description_t to pass to the next layer
          std::shared_ptr<type_description_t> recurse_description(new type_description_t);

          recurse_description->type_description =
            static_cast<individual_type_description_t *>(ref_lookup);

          recurse_description->referenced_type_descriptions =
            description->referenced_type_descriptions;

          auto nested_struct = fastrtps__construct_type_from_description(
            ets_impl, recurse_description.get());

          if (nested_struct == NULL) {
            std::cerr << "Could not construct nested type for field ["
                      << field->field_name << "]" << std::endl;
            return nullptr;
          }

          fastrtps__add_nested_struct_member(
            ets_impl,
            builder.get(), i, field->field_name, nested_struct);
        }
        break;

      // PRIMITIVES
      case BOOL_T_IDX:
        fastrtps__add_bool_member(ets_impl, builder.get(), i, field->field_name);
        break;
      case BYTE_T_IDX:
        fastrtps__add_byte_member(ets_impl, builder.get(), i, field->field_name);
        break;
      case CHAR_T_IDX:
        fastrtps__add_char_member(ets_impl, builder.get(), i, field->field_name);
        break;
      case FLOAT_32_T_IDX:
        fastrtps__add_float32_member(ets_impl, builder.get(), i, field->field_name);
        break;
      case FLOAT_64_T_IDX:
        fastrtps__add_float64_member(ets_impl, builder.get(), i, field->field_name);
        break;
      case INT_8_T_IDX:
        fastrtps__add_int8_member(ets_impl, builder.get(), i, field->field_name);
        break;
      case UINT_8_T_IDX:
        fastrtps__add_uint8_member(ets_impl, builder.get(), i, field->field_name);
        break;
      case INT_16_T_IDX:
        fastrtps__add_int16_member(ets_impl, builder.get(), i, field->field_name);
        break;
      case UINT_16_T_IDX:
        fastrtps__add_uint16_member(ets_impl, builder.get(), i, field->field_name);
        break;
      case INT_32_T_IDX:
        fastrtps__add_int32_member(ets_impl, builder.get(), i, field->field_name);
        break;
      case UINT_32_T_IDX:
        fastrtps__add_uint32_member(ets_impl, builder.get(), i, field->field_name);
        break;
      case INT_64_T_IDX:
        fastrtps__add_int64_member(ets_impl, builder.get(), i, field->field_name);
        break;
      case UINT_64_T_IDX:
        fastrtps__add_uint64_member(ets_impl, builder.get(), i, field->field_name);
        break;
      case STRING_T_IDX:
        fastrtps__add_string_member(ets_impl, builder.get(), i, field->field_name);
        break;
      case WSTRING_T_IDX:
        fastrtps__add_wstring_member(ets_impl, builder.get(), i, field->field_name);
        break;
      case BOUNDED_STRING_T_IDX:
        fastrtps__add_bounded_string_member(
          ets_impl, builder.get(), i, field->field_name,
          field->field_array_size);
        break;
      case BOUNDED_WSTRING_T_IDX:
        fastrtps__add_bounded_wstring_member(
          ets_impl, builder.get(), i, field->field_name,
          field->field_array_size);
        break;
      default:
        printf("[ERROR] Invalid field type!");
        break;
    }
  }

  return fastrtps__finalize_struct_builder(ets_impl, builder.get());
}


// DYNAMIC TYPE PRIMITIVE MEMBERS ==================================================================
void
fastrtps__add_bool_member(
  EvolvingFastRtpsTypeSupportImpl * ets_impl, void * builder,
  uint32_t id, const char * name)
{
  static_cast<DynamicTypeBuilder *>(builder)->add_member(
    id, name,
    ets_impl->factory_->create_bool_type()
  );
}


void
fastrtps__add_byte_member(
  EvolvingFastRtpsTypeSupportImpl * ets_impl, void * builder,
  uint32_t id, const char * name)
{
  static_cast<DynamicTypeBuilder *>(builder)->add_member(
    id, name,
    ets_impl->factory_->create_byte_type()
  );
}


void
fastrtps__add_char_member(
  EvolvingFastRtpsTypeSupportImpl * ets_impl, void * builder,
  uint32_t id, const char * name)
{
  static_cast<DynamicTypeBuilder *>(builder)->add_member(
    id, name,
    ets_impl->factory_->create_char8_type()
  );
}


void
fastrtps__add_float32_member(
  EvolvingFastRtpsTypeSupportImpl * ets_impl, void * builder,
  uint32_t id, const char * name)
{
  static_cast<DynamicTypeBuilder *>(builder)->add_member(
    id, name,
    ets_impl->factory_->create_float32_type()
  );
}


void
fastrtps__add_float64_member(
  EvolvingFastRtpsTypeSupportImpl * ets_impl, void * builder,
  uint32_t id, const char * name)
{
  static_cast<DynamicTypeBuilder *>(builder)->add_member(
    id, name,
    ets_impl->factory_->create_float64_type()
  );
}


void
fastrtps__add_int8_member(
  EvolvingFastRtpsTypeSupportImpl * ets_impl, void * builder,
  uint32_t id, const char * name)
{
  fastrtps__add_byte_member(ets_impl, builder, id, name);
}


void
fastrtps__add_uint8_member(
  EvolvingFastRtpsTypeSupportImpl * ets_impl, void * builder,
  uint32_t id, const char * name)
{
  fastrtps__add_byte_member(ets_impl, builder, id, name);
}


void
fastrtps__add_int16_member(
  EvolvingFastRtpsTypeSupportImpl * ets_impl, void * builder,
  uint32_t id, const char * name)
{
  static_cast<DynamicTypeBuilder *>(builder)->add_member(
    id, name,
    ets_impl->factory_->create_int16_type()
  );
}


void
fastrtps__add_uint16_member(
  EvolvingFastRtpsTypeSupportImpl * ets_impl, void * builder,
  uint32_t id, const char * name)
{
  static_cast<DynamicTypeBuilder *>(builder)->add_member(
    id, name,
    ets_impl->factory_->create_uint16_type()
  );
}


void
fastrtps__add_int32_member(
  EvolvingFastRtpsTypeSupportImpl * ets_impl, void * builder,
  uint32_t id, const char * name)
{
  static_cast<DynamicTypeBuilder *>(builder)->add_member(
    id, name,
    ets_impl->factory_->create_int32_type()
  );
}


void
fastrtps__add_uint32_member(
  EvolvingFastRtpsTypeSupportImpl * ets_impl, void * builder,
  uint32_t id, const char * name)
{
  static_cast<DynamicTypeBuilder *>(builder)->add_member(
    id, name,
    ets_impl->factory_->create_uint32_type()
  );
}


void
fastrtps__add_int64_member(
  EvolvingFastRtpsTypeSupportImpl * ets_impl, void * builder,
  uint32_t id, const char * name)
{
  static_cast<DynamicTypeBuilder *>(builder)->add_member(
    id, name,
    ets_impl->factory_->create_int64_type()
  );
}


void
fastrtps__add_uint64_member(
  EvolvingFastRtpsTypeSupportImpl * ets_impl, void * builder,
  uint32_t id, const char * name)
{
  static_cast<DynamicTypeBuilder *>(builder)->add_member(
    id, name,
    ets_impl->factory_->create_uint64_type()
  );
}


void
fastrtps__add_string_member(
  EvolvingFastRtpsTypeSupportImpl * ets_impl, void * builder, uint32_t id, const char * name)
{
  fastrtps__add_bounded_string_member(
    ets_impl, builder, id, name, eprosima::fastrtps::types::MAX_STRING_LENGTH);
}


void
fastrtps__add_wstring_member(
  EvolvingFastRtpsTypeSupportImpl * ets_impl, void * builder, uint32_t id, const char * name)
{
  fastrtps__add_bounded_wstring_member(
    ets_impl, builder, id, name, eprosima::fastrtps::types::MAX_STRING_LENGTH);
}


void
fastrtps__add_bounded_string_member(
  EvolvingFastRtpsTypeSupportImpl * ets_impl, void * builder,
  uint32_t id, const char * name, uint32_t bound)
{
  static_cast<DynamicTypeBuilder *>(builder)->add_member(
    id, name, ets_impl->factory_->create_string_type(bound));
}


void
fastrtps__add_bounded_wstring_member(
  EvolvingFastRtpsTypeSupportImpl * ets_impl, void * builder,
  uint32_t id, const char * name, uint32_t bound)
{
  static_cast<DynamicTypeBuilder *>(builder)->add_member(
    id, name, ets_impl->factory_->create_wstring_type(bound));
}


// DYNAMIC TYPE NESTED MEMBERS =====================================================================
void
fastrtps__add_nested_struct_member(
  EvolvingFastRtpsTypeSupportImpl * ets_impl, void * builder,
  uint32_t id, const char * name, void * nested_struct)
{
  (void) ets_impl;

  auto nested_struct_dynamictype_ptr = eprosima::fastrtps::types::DynamicType_ptr(
    std::move(
      *reinterpret_cast<eprosima::fastrtps::types::DynamicType_ptr *>(nested_struct)
    )
  );

  static_cast<DynamicTypeBuilder *>(builder)->add_member(id, name, nested_struct_dynamictype_ptr);
}


// DYNAMIC DATA UTILS ==============================================================================
void
fastrtps__print_dynamic_data(EvolvingFastRtpsTypeSupportImpl * ets_impl, void * data)
{
  (void) ets_impl;
  DynamicDataHelper::print(static_cast<DynamicData *>(data));
}

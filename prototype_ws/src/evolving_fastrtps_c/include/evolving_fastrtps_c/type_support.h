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

#ifndef EVOLVING_FASTRTPS_C__TYPE_SUPPORT_IMPL_H_
#define EVOLVING_FASTRTPS_C__TYPE_SUPPORT_IMPL_H_

#include "evolving_serialization_lib/evolving_type_support.h"

#ifdef __cplusplus
extern "C" {
#endif


// CORE ============================================================================================
typedef struct EvolvingFastRtpsTypeSupportImpl_s EvolvingFastRtpsTypeSupportImpl;

EvolvingFastRtpsTypeSupportImpl *
create_fastrtps_evolving_typesupport_impl();

EvolvingTypeSupportInterface *
create_fastrtps_evolving_typesupport_interface();


// DYNAMIC TYPE CONSTRUCTION =======================================================================
void *
fastrtps__create_struct_builder(EvolvingFastRtpsTypeSupportImpl * ets_impl, const char * name);

void *
fastrtps__finalize_struct_builder(EvolvingFastRtpsTypeSupportImpl * ets_impl, void * builder);

void *
fastrtps__construct_type_from_description(
  EvolvingFastRtpsTypeSupportImpl * ets_impl, type_description_t * description);


// DYNAMIC TYPE PRIMITIVE MEMBERS ==================================================================
void
fastrtps__add_bool_member(
  EvolvingFastRtpsTypeSupportImpl * ets_impl, void * builder, uint32_t id, const char * name);

void
fastrtps__add_byte_member(
  EvolvingFastRtpsTypeSupportImpl * ets_impl, void * builder, uint32_t id, const char * name);

void
fastrtps__add_char_member(
  EvolvingFastRtpsTypeSupportImpl * ets_impl, void * builder, uint32_t id, const char * name);

void
fastrtps__add_float32_member(
  EvolvingFastRtpsTypeSupportImpl * ets_impl, void * builder, uint32_t id, const char * name);

void
fastrtps__add_float64_member(
  EvolvingFastRtpsTypeSupportImpl * ets_impl, void * builder, uint32_t id, const char * name);

void
fastrtps__add_int8_member(
  EvolvingFastRtpsTypeSupportImpl * ets_impl, void * builder, uint32_t id, const char * name);

void
fastrtps__add_uint8_member(
  EvolvingFastRtpsTypeSupportImpl * ets_impl, void * builder, uint32_t id, const char * name);

void
fastrtps__add_int16_member(
  EvolvingFastRtpsTypeSupportImpl * ets_impl, void * builder, uint32_t id, const char * name);

void
fastrtps__add_uint16_member(
  EvolvingFastRtpsTypeSupportImpl * ets_impl, void * builder, uint32_t id, const char * name);

void
fastrtps__add_int32_member(
  EvolvingFastRtpsTypeSupportImpl * ets_impl, void * builder, uint32_t id, const char * name);

void
fastrtps__add_uint32_member(
  EvolvingFastRtpsTypeSupportImpl * ets_impl, void * builder, uint32_t id, const char * name);

void
fastrtps__add_int64_member(
  EvolvingFastRtpsTypeSupportImpl * ets_impl, void * builder, uint32_t id, const char * name);

void
fastrtps__add_uint64_member(
  EvolvingFastRtpsTypeSupportImpl * ets_impl, void * builder, uint32_t id, const char * name);

void
fastrtps__add_string_member(
  EvolvingFastRtpsTypeSupportImpl * ets_impl, void * builder, uint32_t id, const char * name);

void
fastrtps__add_wstring_member(
  EvolvingFastRtpsTypeSupportImpl * ets_impl, void * builder, uint32_t id, const char * name);

void
fastrtps__add_bounded_string_member(
  EvolvingFastRtpsTypeSupportImpl * ets_impl, void * builder,
  uint32_t id, const char * name, uint32_t bound);

void
fastrtps__add_bounded_wstring_member(
  EvolvingFastRtpsTypeSupportImpl * ets_impl, void * builder,
  uint32_t id, const char * name, uint32_t bound);


// DYNAMIC TYPE NESTED MEMBERS =====================================================================
void
fastrtps__add_nested_struct_member(
  EvolvingFastRtpsTypeSupportImpl * instance, void * builder,
  uint32_t id, const char * name, void * nested_struct);

// DYNAMIC DATA UTILS ==============================================================================
void
fastrtps__print_dynamic_data(EvolvingFastRtpsTypeSupportImpl * ets_impl, void * data);


// UTILS ===========================================================================================


#ifdef __cplusplus
}
#endif

#endif  // EVOLVING_FASTRTPS_C__TYPE_SUPPORT_IMPL_H_

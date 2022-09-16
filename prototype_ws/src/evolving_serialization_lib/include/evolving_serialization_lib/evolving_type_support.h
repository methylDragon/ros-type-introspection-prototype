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

/// Polymorphic evolving type support interface
/// Downstream middlewares should populate this interface as appropriate

#ifndef EVOLVING_SERIALIZATION_LIB__EVOLVING_TYPE_SUPPORT_H_
#define EVOLVING_SERIALIZATION_LIB__EVOLVING_TYPE_SUPPORT_H_

#include <evolving_serialization_lib/types.h>
#include <evolving_serialization_lib/description.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct evolving_type_support_interface
{
  /// Interfaces mimicking the XTypes spec (Section 7.5: Language Binding)
  /// https://www.omg.org/spec/DDS-XTypes/1.1/PDF
  ///
  /// Luckily for us, FastRTPS mimics the spec quite well

  // double (*Area)(void *instance);

  // We need interfaces for...
  //
  // Building dynamic types
  //  - Create factory

  // DYNAMIC TYPE CONSTRUCTION
  void * (*create_struct_builder)(void * instance, const char * name);
  void * (*finalize_struct_builder)(void * instance, void * builder);
  void * (*construct_type_from_description)(void * instance, type_description_t * description);

  // DYNAMIC TYPE MEMBERS
  void (* add_bool_member)(void * instance, void * builder, uint32_t id, const char * name);
  void (* add_byte_member)(void * instance, void * builder, uint32_t id, const char * name);
  void (* add_char_member)(void * instance, void * builder, uint32_t id, const char * name);
  void (* add_float32_member)(void * instance, void * builder, uint32_t id, const char * name);
  void (* add_float64_member)(void * instance, void * builder, uint32_t id, const char * name);
  void (* add_int8_member)(void * instance, void * builder, uint32_t id, const char * name);
  void (* add_uint8_member)(void * instance, void * builder, uint32_t id, const char * name);
  void (* add_int16_member)(void * instance, void * builder, uint32_t id, const char * name);
  void (* add_uint16_member)(void * instance, void * builder, uint32_t id, const char * name);
  void (* add_int32_member)(void * instance, void * builder, uint32_t id, const char * name);
  void (* add_uint32_member)(void * instance, void * builder, uint32_t id, const char * name);
  void (* add_int64_member)(void * instance, void * builder, uint32_t id, const char * name);
  void (* add_uint64_member)(void * instance, void * builder, uint32_t id, const char * name);
  void (* add_string_member)(void * instance, void * builder, uint32_t id, const char * name);
  void (* add_wstring_member)(void * instance, void * builder, uint32_t id, const char * name);
  void (* add_bounded_string_member)(
    void * instance, void * builder, uint32_t id, const char * name, uint32_t bound);
  void (* add_bounded_wstring_member)(
    void * instance, void * builder, uint32_t id, const char * name, uint32_t bound);

  // DYNAMIC DATA UTILS
  void (* print_dynamic_data)(void * instance, void * data);


  // Building and populating dynamic data
  //  - Create dynamic data object
  //  - Set members (per type, per ID and per name (so TYPE * 2 members))
  //
  // Introspecting dynamic data
  //  - Getting members
  //  - Printing
  //
  // Then getting something that can be passed to pubsub
  // And maybe utils (eg. printing)


  // Reminder that function pointers are:
  // OUTPUT (* FN_NAME)(INPUTS)
  // double (* Area)(void * instance);
} EvolvingTypeSupportInterface;


// =================================================================================================
// FUNCTION REDIRECTION USING INTERFACE
// =================================================================================================

// CORE ============================================================================================
typedef struct
{
  void * instance;
  const EvolvingTypeSupportInterface * interface;
} EvolvingTypeSupport;

EvolvingTypeSupport *
create_evolving_typesupport(void * instance, EvolvingTypeSupportInterface * interface);


// DYNAMIC TYPE CONSTRUCTION =======================================================================
void *
ets_create_struct_builder(EvolvingTypeSupport * ets, const char * name);

void *
ets_finalize_struct_builder(EvolvingTypeSupport * ets, void * builder);

void *
ets_construct_type_from_description(EvolvingTypeSupport * ets, type_description_t * description);


// DYNAMIC TYPE MEMBERS ============================================================================
void
ets_add_bool_member(EvolvingTypeSupport * ets, void * builder, uint32_t id, const char * name);

void
ets_add_byte_member(EvolvingTypeSupport * ets, void * builder, uint32_t id, const char * name);

void
ets_add_char_member(EvolvingTypeSupport * ets, void * builder, uint32_t id, const char * name);

void
ets_add_float32_member(EvolvingTypeSupport * ets, void * builder, uint32_t id, const char * name);

void
ets_add_float64_member(EvolvingTypeSupport * ets, void * builder, uint32_t id, const char * name);

void
ets_add_int8_member(EvolvingTypeSupport * ets, void * builder, uint32_t id, const char * name);

void
ets_add_uint8_member(EvolvingTypeSupport * ets, void * builder, uint32_t id, const char * name);

void
ets_add_int16_member(EvolvingTypeSupport * ets, void * builder, uint32_t id, const char * name);

void
ets_add_uint16_member(EvolvingTypeSupport * ets, void * builder, uint32_t id, const char * name);

void
ets_add_int32_member(EvolvingTypeSupport * ets, void * builder, uint32_t id, const char * name);

void
ets_add_uint32_member(EvolvingTypeSupport * ets, void * builder, uint32_t id, const char * name);

void
ets_add_int64_member(EvolvingTypeSupport * ets, void * builder, uint32_t id, const char * name);

void
ets_add_uint64_member(EvolvingTypeSupport * ets, void * builder, uint32_t id, const char * name);

void
ets_add_string_member(EvolvingTypeSupport * ets, void * builder, uint32_t id, const char * name);

void
ets_add_wstring_member(EvolvingTypeSupport * ets, void * builder, uint32_t id, const char * name);

void
ets_add_bounded_string_member(
  EvolvingTypeSupport * ets, void * builder, uint32_t id, const char * name, uint32_t bound);

void
ets_add_bounded_wstring_member(
  EvolvingTypeSupport * ets, void * builder, uint32_t id, const char * name, uint32_t bound);


// DYNAMIC DATA UTILS ==============================================================================
void
ets_print_dynamic_data(EvolvingTypeSupport * ets, void * data);


#ifdef __cplusplus
}
#endif

#endif  // EVOLVING_SERIALIZATION_LIB__EVOLVING_TYPE_SUPPORT_H_

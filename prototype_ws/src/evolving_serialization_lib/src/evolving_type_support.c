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

#include <evolving_serialization_lib/evolving_type_support.h>

/// Base interface redirections

// =================================================================================================
// FUNCTION REDIRECTION USING INTERFACE
// =================================================================================================

// CORE ============================================================================================
EvolvingTypeSupport *
create_evolving_typesupport(void * instance, EvolvingTypeSupportInterface * interface)
{
  EvolvingTypeSupport * ts = (EvolvingTypeSupport *) malloc(sizeof(EvolvingTypeSupport));
  ts->instance = instance;
  ts->interface = interface;
  return ts;
}


// DYNAMIC TYPE CONSTRUCTION =======================================================================
void *
ets_create_struct_builder(EvolvingTypeSupport * ets, const char * name)
{
  return (ets->interface->create_struct_builder)(ets->instance, name);
}

void *
ets_finalize_struct_builder(EvolvingTypeSupport * ets, void * builder)
{
  return (ets->interface->finalize_struct_builder)(ets->instance, builder);
}

void *
ets_construct_type_from_description(EvolvingTypeSupport * ets, type_description_t * description)
{
  return (ets->interface->construct_type_from_description)(ets->instance, description);
}

// DYNAMIC TYPE PRIMITIVE MEMBERS ==================================================================
void
ets_add_bool_member(EvolvingTypeSupport * ets, void * builder, uint32_t id, const char * name)
{
  (ets->interface->add_bool_member)(ets->instance, builder, id, name);
}


void
ets_add_byte_member(EvolvingTypeSupport * ets, void * builder, uint32_t id, const char * name)
{
  (ets->interface->add_byte_member)(ets->instance, builder, id, name);
}


void
ets_add_char_member(EvolvingTypeSupport * ets, void * builder, uint32_t id, const char * name)
{
  (ets->interface->add_char_member)(ets->instance, builder, id, name);
}


void
ets_add_float32_member(EvolvingTypeSupport * ets, void * builder, uint32_t id, const char * name)
{
  (ets->interface->add_float32_member)(ets->instance, builder, id, name);
}


void
ets_add_float64_member(EvolvingTypeSupport * ets, void * builder, uint32_t id, const char * name)
{
  (ets->interface->add_float64_member)(ets->instance, builder, id, name);
}


void
ets_add_int8_member(EvolvingTypeSupport * ets, void * builder, uint32_t id, const char * name)
{
  (ets->interface->add_int8_member)(ets->instance, builder, id, name);
}


void
ets_add_uint8_member(EvolvingTypeSupport * ets, void * builder, uint32_t id, const char * name)
{
  (ets->interface->add_uint8_member)(ets->instance, builder, id, name);
}


void
ets_add_int16_member(EvolvingTypeSupport * ets, void * builder, uint32_t id, const char * name)
{
  (ets->interface->add_int16_member)(ets->instance, builder, id, name);
}


void
ets_add_uint16_member(EvolvingTypeSupport * ets, void * builder, uint32_t id, const char * name)
{
  (ets->interface->add_uint16_member)(ets->instance, builder, id, name);
}


void
ets_add_int32_member(EvolvingTypeSupport * ets, void * builder, uint32_t id, const char * name)
{
  (ets->interface->add_int32_member)(ets->instance, builder, id, name);
}


void
ets_add_uint32_member(EvolvingTypeSupport * ets, void * builder, uint32_t id, const char * name)
{
  (ets->interface->add_uint32_member)(ets->instance, builder, id, name);
}


void
ets_add_int64_member(EvolvingTypeSupport * ets, void * builder, uint32_t id, const char * name)
{
  (ets->interface->add_int64_member)(ets->instance, builder, id, name);
}


void
ets_add_uint64_member(EvolvingTypeSupport * ets, void * builder, uint32_t id, const char * name)
{
  (ets->interface->add_uint64_member)(ets->instance, builder, id, name);
}


void
ets_add_string_member(EvolvingTypeSupport * ets, void * builder, uint32_t id, const char * name)
{
  (ets->interface->add_string_member)(ets->instance, builder, id, name);
}


void
ets_add_wstring_member(
  EvolvingTypeSupport * ets, void * builder,
  uint32_t id, const char * name)
{
  (ets->interface->add_wstring_member)(ets->instance, builder, id, name);
}


void
ets_add_bounded_string_member(
  EvolvingTypeSupport * ets, void * builder,
  uint32_t id, const char * name, uint32_t bound)
{
  (ets->interface->add_bounded_string_member)(ets->instance, builder, id, name, bound);
}


void
ets_add_bounded_wstring_member(
  EvolvingTypeSupport * ets, void * builder,
  uint32_t id, const char * name, uint32_t bound)
{
  (ets->interface->add_bounded_wstring_member)(ets->instance, builder, id, name, bound);
}


// DYNAMIC TYPE NESTED MEMBERS =====================================================================
void
ets_add_nested_struct_member(
  EvolvingTypeSupport * ets, void * builder,
  uint32_t id, const char * name, void * nested_struct)
{
  (ets->interface->add_nested_struct_member)(ets->instance, builder, id, name, nested_struct);
}


// DYNAMIC DATA UTILS ==============================================================================
void
ets_print_dynamic_data(EvolvingTypeSupport * ets, void * data)
{
  (ets->interface->print_dynamic_data)(ets->instance, data);
}

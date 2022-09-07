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

#include <glib.h>
#include <evolving_serialization_lib/types.h>


// =================================================================================================
// Structs
// =================================================================================================
// TYPE DESCRIPTION FIELD ==========================================================================
typedef struct type_description_field_t
{
  char * field_name;
  uint8_t field_type;

  uint64_t field_array_size;
  char * nested_type_name;
} type_description_field_t;

type_description_field_t *
get_zero_initialized_type_description_field(void);

rcl_ret_t
type_description_field_fini(type_description_field_t * type_description_field);


// INDIVIDUAL TYPE DESCRIPTION =====================================================================
typedef struct individual_type_description_t
{
  char * type_name;
  char * type_version_hash;

  type_description_field_t ** fields;
  size_t field_count;
} individual_type_description_t;

individual_type_description_t *
get_zero_initialized_individual_type_description(void);

rcl_ret_t
individual_type_description_fini(individual_type_description_t * individual_type_description);


// TYPE DESCRIPTION ================================================================================
typedef struct type_description_t
{
  individual_type_description_t * type_description;

  // Hash-table of char * -> individual_type_description_t *
  // Keyed by referenced type_name
  //
  // We can also get the count of referenced_type_descriptions with
  // g_hash_table_size()
  GHashTable * referenced_type_descriptions;
} type_description_t;

type_description_t *
get_zero_initialized_type_description(void);

rcl_ret_t
type_description_fini(type_description_t * type_description);


// =================================================================================================
// Construction
// =================================================================================================
individual_type_description_t *
populate_individual_type_description(
  individual_type_description_t * individual_description_struct,
  GNode * description_node);


type_description_t *
populate_type_description(
  type_description_t * description_struct,
  GNode * full_description_node);


// =================================================================================================
// Printing
// =================================================================================================
void
print_type_description_field(type_description_field_t * input);

void
print_individual_type_description(individual_type_description_t * input);

void
print_type_description(type_description_t * input);

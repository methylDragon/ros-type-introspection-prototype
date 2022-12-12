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

#ifndef SERIALIZATION_SUPPORT_LIB__TYPES_H_
#define SERIALIZATION_SUPPORT_LIB__TYPES_H_

#include <stdbool.h>
#include <stdint.h>


#ifdef __cplusplus
extern "C" {
#endif


// DESCRIPTION STRUCTS =============================================================================
// NOTE(methylDragon): The description structs are declared here to avoid C linking issues against
//                     glib
typedef struct type_description_field_s type_description_field_t;
typedef struct individual_type_description_s  individual_type_description_t;
typedef struct type_description_s type_description_t;


// TYPE INDICES ====================================================================================
typedef uint32_t MemberId;

#define UNSET_T_ID 0
#define NESTED_T_ID 1


// PRIMITIVES ======================================================================================
#define BOOL_T_ID 2
#define BYTE_T_ID 3
#define CHAR_T_ID 4

#define FLOAT_32_T_ID 5
#define FLOAT_64_T_ID 6
#define INT_8_T_ID 7
#define UINT_8_T_ID 8
#define INT_16_T_ID 9
#define UINT_16_T_ID 10
#define INT_32_T_ID 11
#define UINT_32_T_ID 12
#define INT_64_T_ID 13
#define UINT_64_T_ID 14

#define STRING_T_ID 15
#define WSTRING_T_ID 16
#define BOUNDED_STRING_T_ID 17
#define BOUNDED_WSTRING_T_ID 18


// SEQUENCES =======================================================================================
#define SEQ_T_DELIMITER 32
#define STATIC_ARRAY_OFFSET 32
#define UNBOUNDED_SEQ_OFFSET 64
#define BOUNDED_SEQ_OFFSET 96

#ifdef __cplusplus
}
#endif

#endif  // SERIALIZATION_SUPPORT_LIB__TYPES_H_

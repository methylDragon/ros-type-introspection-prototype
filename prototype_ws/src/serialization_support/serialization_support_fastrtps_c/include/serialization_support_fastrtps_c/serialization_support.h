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

#ifndef SERIALIZATION_SUPPORT_FASTRTPS_C__SERIALIZATION_SUPPORT_H_
#define SERIALIZATION_SUPPORT_FASTRTPS_C__SERIALIZATION_SUPPORT_H_

#include <serialization_support_fastrtps_c/dynamic_data.h>
#include <serialization_support_fastrtps_c/dynamic_type.h>
#include <serialization_support_fastrtps_c/identifier.h>
#include <serialization_support_fastrtps_c/serialization_impl.h>


#ifdef __cplusplus
extern "C" {
#endif


/// This is the main file to include

// CORE ============================================================================================
serialization_support_interface_t *
create_fastrtps_ser_interface();

void
fastrtps__ser_impl_fini(ser_impl_t * ser_impl);


#ifdef __cplusplus
}
#endif

#endif  // SERIALIZATION_SUPPORT_FASTRTPS_C__SERIALIZATION_SUPPORT_H_

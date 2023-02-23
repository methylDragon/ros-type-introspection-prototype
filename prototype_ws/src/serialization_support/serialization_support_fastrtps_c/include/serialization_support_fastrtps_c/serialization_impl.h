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

#ifndef SERIALIZATION_SUPPORT_FASTRTPS_C__SERIALIZATION_IMPL_H_
#define SERIALIZATION_SUPPORT_FASTRTPS_C__SERIALIZATION_IMPL_H_

#include <fastrtps/types/DynamicDataFactory.h>
#include <fastrtps/types/DynamicTypeBuilderFactory.h>

#include "serialization_support_lib/api/serialization_support_interface.h"


#ifdef __cplusplus
extern "C" {
#endif


// CORE ============================================================================================
typedef struct fastrtps_ser_impl_s
{
  eprosima::fastrtps::types::DynamicTypeBuilderFactory * type_factory_;
  eprosima::fastrtps::types::DynamicDataFactory * data_factory_;
} fastrtps_ser_impl_t;


fastrtps_ser_impl_t *
create_fastrtps_ser_impl();


#ifdef __cplusplus
}
#endif

#endif  // SERIALIZATION_SUPPORT_FASTRTPS_C__SERIALIZATION_IMPL_H_

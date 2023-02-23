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

#include <fastrtps/types/DynamicDataFactory.h>
#include <fastrtps/types/DynamicTypeBuilderFactory.h>

#include <serialization_support_lib/api/serialization_support_interface.h>
#include <serialization_support_fastrtps_c/serialization_impl.h>

using eprosima::fastrtps::types::DynamicDataFactory;
using eprosima::fastrtps::types::DynamicTypeBuilderFactory;

// CORE ============================================================================================
fastrtps_ser_impl_t *
create_fastrtps_ser_impl()
{
  fastrtps_ser_impl_t * ser_impl =
    (fastrtps_ser_impl_t *) malloc(sizeof(fastrtps_ser_impl_t));

  // The actual business
  ser_impl->type_factory_ = DynamicTypeBuilderFactory::get_instance();
  ser_impl->data_factory_ = DynamicDataFactory::get_instance();

  return ser_impl;
}

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

#include <stdlib.h>

#include <serialization_support_lib/api/serialization_support.h>
#include <serialization_support_lib/api/serialization_support_interface.h>


// CORE ============================================================================================
const char *
ser_get_library_identifier(serialization_support_t * ser)
{
  return ser->interface->library_identifier;
}


serialization_support_t *
ser_support_init(void * impl, void * interface)
{
  serialization_support_t * ts =
    (serialization_support_t *) malloc(sizeof(serialization_support_t));

  ts->impl = (ser_impl_t *) malloc(sizeof(ser_impl_t));
  ts->impl->impl = impl;

  ts->interface = interface;
  return ts;
}


void
ser_support_fini(serialization_support_t * ser)
{
  (ser->interface->ser_impl_fini)(ser->impl);
  free(ser->impl);
  free(ser);
}

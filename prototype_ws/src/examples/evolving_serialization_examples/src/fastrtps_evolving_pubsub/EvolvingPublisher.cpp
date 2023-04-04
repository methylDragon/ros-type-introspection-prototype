// Copyright 2016 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#include "EvolvingPublisher.h"

#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/attributes/PublisherAttributes.h>

#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/qos/PublisherQos.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>

#include <fastrtps/types/DynamicDataHelper.hpp>
#include <fastrtps/types/DynamicDataFactory.h>
#include <fastrtps/types/DynamicTypeBuilderFactory.h>
#include <fastrtps/types/DynamicTypeBuilderPtr.h>

#include <thread>

#include "rosidl_dynamic_typesupport/yaml_parser.h"
#include "rosidl_dynamic_typesupport/description.h"
#include "rosidl_dynamic_typesupport/tree_traverse.h"
#include "rosidl_dynamic_typesupport/api/serialization_support.h"
#include "rosidl_dynamic_typesupport_fastrtps/serialization_support.h"

using namespace eprosima::fastdds::dds;
using eprosima::fastrtps::types::DynamicData;
// using eprosima::fastrtps::types::DynamicType;  // Conflicts
using eprosima::fastrtps::types::DynamicType_ptr;
using eprosima::fastrtps::types::DynamicTypeMember;

static rosidl_dynamic_typesupport_serialization_support_t * serialization_support = rosidl_dynamic_typesupport_serialization_support_init(
  rosidl_dynamic_typesupport_fastrtps_create_serialization_support_impl(),
  rosidl_dynamic_typesupport_fastrtps_create_serialization_support_interface());

EvolvingPublisher::EvolvingPublisher()
: mp_participant(nullptr), mp_publisher(nullptr)
{
}


bool EvolvingPublisher::init()
{
  using namespace eprosima::fastrtps::types;

  char * msg_path =
    g_strjoin("/", g_path_get_dirname(__FILE__), "..", "..", "msg", "example_pub_msg.yaml", NULL);
  type_description_t * full_description_struct = create_type_description_from_yaml_file(msg_path);

  rosidl_dynamic_typesupport_dynamic_type_t * example_msg_type =
    rosidl_dynamic_typesupport_dynamic_type_init_from_description(serialization_support, full_description_struct);

  // Create and Populate Data
  rosidl_dynamic_typesupport_dynamic_data_t * example_msg_data = rosidl_dynamic_typesupport_dynamic_data_init_from_dynamic_type(example_msg_type);

  rosidl_dynamic_typesupport_dynamic_data_set_string_value(example_msg_data, "A message!", 0);

  rosidl_dynamic_typesupport_dynamic_data_t * bool_array = rosidl_dynamic_typesupport_dynamic_data_loan_value(example_msg_data, 1);
  for (uint32_t i = 0; i < 5; ++i) {
    rosidl_dynamic_typesupport_member_id_t id = rosidl_dynamic_typesupport_dynamic_data_get_array_index(bool_array, i);
    rosidl_dynamic_typesupport_dynamic_data_set_bool_value(bool_array, i % 2 == 0, id);
  }
  rosidl_dynamic_typesupport_dynamic_data_return_loaned_value(example_msg_data, bool_array);

  this->msg_data_ = static_cast<DynamicData *>(example_msg_data->impl->handle);

  // Show
  std::map<std::string, DynamicTypeMember *> evolving_map;

  // NOTE(methylDragon): This is INCREDIBLY IMPORTANT to preserve lifetime!!!
  msg_type_ = DynamicType_ptr(*static_cast<DynamicType_ptr *>(example_msg_type->impl->handle));
  msg_type_->get_all_members_by_name(evolving_map);

  for (auto const & x : evolving_map) {
    std::cout << x.first << ':' << x.second << std::endl;
  }

  std::cout << "\n* * * = INITIAL MESSAGE CONSTRUCTED = * * *\n" << std::endl;
  rosidl_dynamic_typesupport_dynamic_data_print(example_msg_data);
  std::cout << "\n* * * * * * * * * * * * * * * * * * * * * *\n" << std::endl;

  TypeSupport example_msg_ts(msg_type_);

  // Setup pub
  DomainParticipantQos pqos;
  pqos.name("Participant_pub");
  mp_participant = DomainParticipantFactory::get_instance()->create_participant(0, pqos);

  if (mp_participant == nullptr) {
    return false;
  }

  example_msg_ts.get()->auto_fill_type_information(true);
  example_msg_ts.get()->auto_fill_type_object(true);
  example_msg_ts.register_type(mp_participant);

  mp_publisher = mp_participant->create_publisher(PUBLISHER_QOS_DEFAULT, nullptr);
  if (mp_publisher == nullptr) {
    return false;
  }

  topic_ = mp_participant->create_topic("EvolvingSubTopic", "ExampleMsg", TOPIC_QOS_DEFAULT);
  if (topic_ == nullptr) {
    return false;
  }

  // CREATE THE WRITER
  // Use QoS settings that mimic rmw_fastrtps
  DataWriterQos wqos;
  wqos.endpoint().history_memory_policy =
    eprosima::fastrtps::rtps::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;
  wqos.publish_mode().kind =
    eprosima::fastdds::dds::ASYNCHRONOUS_PUBLISH_MODE;

  writer_ = mp_publisher->create_datawriter(topic_, wqos, &m_listener);
  if (writer_ == nullptr) {
    return false;
  }

  return true;
}


EvolvingPublisher::~EvolvingPublisher()
{
  if (writer_ != nullptr) {
    mp_publisher->delete_datawriter(writer_);
  }
  if (mp_publisher != nullptr) {
    mp_participant->delete_publisher(mp_publisher);
  }
  if (topic_ != nullptr) {
    mp_participant->delete_topic(topic_);
  }
  DomainParticipantFactory::get_instance()->delete_participant(mp_participant);
}


void EvolvingPublisher::PubListener::on_publication_matched(
  eprosima::fastdds::dds::DataWriter *,
  const eprosima::fastdds::dds::PublicationMatchedStatus & info)
{
  if (info.current_count_change == 1) {
    n_matched = info.total_count;
    firstConnected = true;
    std::cout << "Publisher matched" << std::endl;
  } else if (info.current_count_change == -1) {
    n_matched = info.total_count;
    std::cout << "Publisher unmatched" << std::endl;
  } else {
    std::cout << info.current_count_change
              << " is not a valid value for PublicationMatchedStatus current count change"
              << std::endl;
  }
}


void EvolvingPublisher::runThread(uint32_t sleep)
{
  using namespace eprosima::fastrtps::types;

  while (!stop) {
    if (publish(true)) {
      std::cout << "\n== SENT ==" << std::endl;
      DynamicDataHelper::print(msg_data_);
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(sleep));
  }
}


void EvolvingPublisher::run(uint32_t sleep)
{
  stop = false;
  std::thread thread(&EvolvingPublisher::runThread, this, sleep);

  std::cout << "Publisher running. Please press enter to stop the Publisher at any time."
            << std::endl;
  std::cin.ignore();
  stop = true;

  thread.join();
}


bool EvolvingPublisher::publish(bool waitForListener)
{
  if (m_listener.firstConnected || !waitForListener || m_listener.n_matched > 0) {
    bool bool_;
    eprosima::fastrtps::types::DynamicData * bool_array_ = msg_data_->loan_value(1);
    bool_array_->get_bool_value(bool_, bool_array_->get_array_index({0}));

    for (uint32_t i = 0; i < 5; i++) {
      bool_array_->set_bool_value(!bool_, bool_array_->get_array_index({i}));
    }

    msg_data_->return_loaned_value(bool_array_);

    float float_;
    eprosima::fastrtps::types::DynamicData * inner_ = msg_data_->loan_value(2);
    eprosima::fastrtps::types::DynamicData * inner_inner_ = inner_->loan_value(0);

    inner_inner_->get_float32_value(float_, 0);
    inner_inner_->set_float32_value(float_ + 1, 0);

    inner_->return_loaned_value(inner_inner_);
    msg_data_->return_loaned_value(inner_);

    // The writer can write anything! Just pass it an appropriate DynamicData
    writer_->write(msg_data_.get());

    return true;
  }
  return false;
}


int main(int argc, char * argv[])
{
  (void)argc;
  (void)argv;

  EvolvingPublisher pub;
  if (pub.init()) {
    pub.run(1000);
  }

  rosidl_dynamic_typesupport_serialization_support_destroy(serialization_support);
  return 0;
}

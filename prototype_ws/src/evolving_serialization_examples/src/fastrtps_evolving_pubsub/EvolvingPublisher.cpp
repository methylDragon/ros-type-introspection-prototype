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

#include "evolving_serialization_lib/yaml_parser.h"
#include "evolving_serialization_lib/description.h"
#include "evolving_serialization_lib/tree_traverse.h"
#include "evolving_serialization_lib/evolving_type_support.h"
#include "evolving_fastrtps_c/type_support.h"

using namespace eprosima::fastdds::dds;

static EvolvingTypeSupport * ets = create_evolving_typesupport(
  create_fastrtps_evolving_typesupport_impl(),
  create_fastrtps_evolving_typesupport_interface());

EvolvingPublisher::EvolvingPublisher()
: mp_participant(nullptr), mp_publisher(nullptr)
{
}


bool EvolvingPublisher::init()
{
  using namespace eprosima::fastrtps::types;

  char * flat_yaml_path =
    g_strjoin("/", g_path_get_dirname(__FILE__), ".", "example_msg.yaml", NULL);

  type_description_t * full_description_struct = create_type_description_from_yaml(flat_yaml_path);

  auto example_msg_type = eprosima::fastrtps::types::DynamicType_ptr(
    std::move(
      *reinterpret_cast<eprosima::fastrtps::types::DynamicType_ptr *>(
        ets_construct_type_from_description(ets, full_description_struct)
      )
    )
  );

  // Create and Populate Data
  this->msg_data_ = DynamicDataFactory::get_instance()->create_data(example_msg_type);

  this->msg_data_->set_string_value("A message!", 0);
  auto bool_array = this->msg_data_->loan_value(1);
  for (uint32_t i = 0; i < 5; ++i) {
    bool_array->set_bool_value(false, bool_array->get_array_index({i}));
  }
  this->msg_data_->return_loaned_value(bool_array);

  std::map<std::string, eprosima::fastrtps::types::DynamicTypeMember *> evolving_map;
  example_msg_type->get_all_members_by_name(evolving_map);

  for (auto const & x : evolving_map) {
    std::cout << x.first << ':' << x.second << std::endl;
  }

  std::cout << "\n* * * = INITIAL MESSAGE CONSTRUCTED = * * *\n" << std::endl;
  DynamicDataHelper::print(this->msg_data_);
  std::cout << "\n* * * * * * * * * * * * * * * * * * * * * *\n" << std::endl;

  TypeSupport example_msg_ts(new eprosima::fastrtps::types::DynamicPubSubType(example_msg_type));

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

  return 0;
}

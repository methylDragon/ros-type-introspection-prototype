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

// Heavily modified from the FastDDS DynamicHelloWorld example
// https://github.com/eProsima/Fast-DDS/tree/master/examples/cpp/dds/DynamicHelloWorldExample

/*
 * Look out for this comment style for evolving serialization relevant code blocks!
 * Also, look out for ser_ prefixed functions!
 */

#include "EvolvingSubscription.h"
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/attributes/SubscriberAttributes.h>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>

#include <fastrtps/types/DynamicDataFactory.h>
#include <fastrtps/types/DynamicTypeBuilderFactory.h>
#include <fastrtps/types/DynamicTypeBuilderPtr.h>

#include "serialization_support_lib/yaml_parser.h"
#include "serialization_support_lib/description.h"
#include "serialization_support_lib/tree_traverse.h"
#include "serialization_support_lib/api/serialization_support.h"
#include "serialization_support_fastrtps_c/serialization_support.h"

using namespace eprosima::fastdds::dds;
using eprosima::fastrtps::types::DynamicData;
using eprosima::fastrtps::types::DynamicData_ptr;
using eprosima::fastrtps::types::DynamicType_ptr;
using eprosima::fastrtps::types::ReturnCode_t;


// This is the redirection struct!
static serialization_support_t * ser = ser_support_init(
  create_fastrtps_ser_impl(),
  create_fastrtps_ser_interface());


EvolvingSubscription::EvolvingSubscription()
: mp_participant(nullptr)
  , mp_subscriber(nullptr)
  , m_listener(this)
{
}


bool EvolvingSubscription::init()
{
  DomainParticipantFactoryQos factory_qos;
  factory_qos.entity_factory().autoenable_created_entities = false;
  DomainParticipantFactory::get_instance()->set_qos(factory_qos);

  DomainParticipantQos pqos;
  pqos.name("Participant_sub");
  StatusMask par_mask = StatusMask::subscription_matched() << StatusMask::data_available();
  mp_participant =
    DomainParticipantFactory::get_instance()->create_participant(0, pqos, &m_listener, par_mask);

  if (mp_participant == nullptr) {
    return false;
  }
  if (mp_participant->enable() != ReturnCode_t::RETCODE_OK) {
    DomainParticipantFactory::get_instance()->delete_participant(mp_participant);
    return false;
  }

  qos_ = DATAREADER_QOS_DEFAULT;
  qos_.reliability().kind = RELIABLE_RELIABILITY_QOS;

  // Use QoS settings that mimic rmw_fastrtps
  qos_.endpoint().history_memory_policy =
    eprosima::fastrtps::rtps::PREALLOCATED_WITH_REALLOC_MEMORY_MODE;

  return true;
}


EvolvingSubscription::~EvolvingSubscription()
{
  for (const auto & it : topics_) {
    mp_subscriber->delete_datareader(it.first);
    mp_participant->delete_topic(it.second);
  }
  if (mp_subscriber != nullptr) {
    mp_participant->delete_subscriber(mp_subscriber);
  }

  DomainParticipantFactory::get_instance()->delete_participant(mp_participant);
  topics_.clear();
  readers_.clear();
  datas_.clear();
}


void EvolvingSubscription::SubListener::on_subscription_matched(
  DataReader *, const SubscriptionMatchedStatus & info)
{
  if (info.current_count_change == 1) {
    std::cout << "Subscriber matched\n" << std::endl;
  } else if (info.current_count_change == -1) {
    std::cout << "Subscriber unmatched\n" << std::endl;
  } else {
    std::cout << info.current_count_change
              << " is not a valid value for SubscriptionMatchedStatus current count change"
              << std::endl;
  }
}


void EvolvingSubscription::SubListener::on_data_available(DataReader * reader)
{
  auto dit = subscription_->datas_.find(reader);

  if (dit != subscription_->datas_.end()) {
    auto builder = new ser_type_builder_t{subscription_->readers_[reader].get()};

    ser_dynamic_data_t * data = ser_data_init_from_builder(ser, builder);
    ser_struct_type_builder_fini(ser, builder);

    SampleInfo info;
    if (reader->take_next_sample(static_cast<DynamicData *>(data->impl), &info) == ReturnCode_t::RETCODE_OK) {
      if (info.instance_state == ALIVE_INSTANCE_STATE) {
        eprosima::fastrtps::types::DynamicType_ptr type = subscription_->readers_[reader];
        this->n_samples++;
        std::cout << "\nReceived data of type " << type->get_name() << std::endl;
        ser_print_dynamic_data(ser, data);
        ser_data_fini(ser, data);
      }
    }
  }
}


void EvolvingSubscription::SubListener::on_type_discovery(
  DomainParticipant *,
  const eprosima::fastrtps::rtps::SampleIdentity &,
  const eprosima::fastrtps::string_255 & topic_name,
  const eprosima::fastrtps::types::TypeIdentifier *,
  const eprosima::fastrtps::types::TypeObject *,
  eprosima::fastrtps::types::DynamicType_ptr dyn_type)
{
  // FastDDS receives DynamicType_ptr as an arg but we explicitly DON'T use it
  // We instead construct our own dynamically. Though we still check for equality!

  /* We can construct our own DynamicType_ptr dynamically using the evolving message
   * types libraries and compare!
   *
   * Equivalent base FastDDS DynamicType code (without dynamically traversing the type):
   *
   * auto type_factory = eprosima::fastrtps::types::DynamicTypeBuilderFactory::get_instance();
   * eprosima::fastrtps::types::DynamicTypeBuilder_ptr builder = type_factory->struct_type_builder_init();
   *
   * builder->add_member(0, "bool_field", type_factory->create_bool_type());
   * builder->add_member(1, "byte_field", type_factory->create_byte_type());
   * builder->add_member(2, "int32_field", type_factory->create_int32_type());
   * builder->add_member(
   *   3, "string_field",
   *   type_factory->create_string_type(eprosima::fastrtps::types::MAX_STRING_LENGTH));
   *
   * builder->set_name("ExampleMsg");
   * eprosima::fastrtps::types::DynamicType_ptr evolving_type_ptr = builder->build();
   *
   */

  // Loading from the yaml "simulates" type description distribution
  char * msg_path =
    g_strjoin("/", g_path_get_dirname(__FILE__), "..", "..", "msg", "example_pub_msg.yaml", NULL);
  type_description_t * full_description_struct = create_type_description_from_yaml_file(msg_path);
  print_type_description(full_description_struct);

  // NOTE(CH3): I hate that I have to do this...
  //            The joys of void pointer casting, and the pain of not being able to move ownership
  //            of shared_ptrs...
  ser_dynamic_type_t * evolving_type =
    ser_construct_type_from_description(ser, full_description_struct);

  auto evolving_type_ptr = DynamicType_ptr(
    *static_cast<DynamicType_ptr *>(evolving_type->impl)
  );

  /* Verify that types are equal! */
  if (dyn_type->equals(evolving_type_ptr.get())) {
    // NOTE(CH3): Type name doesn't matter for equality...
    //            To be honest, neither does correctly allocating the type struct...
    std::cout << "Dynamic type is equal!" << std::endl;
  } else {
    std::cout << "Dynamic type is NOT equal!" << std::endl;

    std::map<std::string, eprosima::fastrtps::types::DynamicTypeMember *> dyn_map;
    dyn_type->get_all_members_by_name(dyn_map);

    std::map<std::string, eprosima::fastrtps::types::DynamicTypeMember *> evolving_map;
    evolving_type_ptr->get_all_members_by_name(evolving_map);

    std::cout << "\nRECEIVED:" << std::endl;
    for (auto const & x : dyn_map) {
      std::cout << x.first << ':' << x.second << std::endl;
    }

    std::cout << "\nLOADED:" << std::endl;
    for (auto const & x : evolving_map) {
      std::cout << x.first << ':' << x.second << std::endl;
    }
    std::cout << std::endl;

    // dyn_type
    // std::cout << dyn_type->get_members_count() << " " << evolving_type_ptr->get_members_count() << std::endl;
  }

  TypeSupport m_type(evolving_type_ptr);
  m_type.register_type(subscription_->mp_participant);

  std::cout << "Discovered type: " << m_type->getName() << " from topic " << topic_name <<
    std::endl;

  if (subscription_->mp_subscriber == nullptr) {
    subscription_->mp_subscriber =
      subscription_->mp_participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT, nullptr);

    if (subscription_->mp_subscriber == nullptr) {
      return;
    }
  }

  Topic * topic = subscription_->mp_participant->create_topic(
    "EvolvingSubTopic", m_type->getName(), TOPIC_QOS_DEFAULT);

  if (topic == nullptr) {
    return;
  }

  StatusMask sub_mask = StatusMask::subscription_matched() << StatusMask::data_available();
  DataReader * reader = subscription_->mp_subscriber->create_datareader(
    topic, subscription_->qos_, &subscription_->m_listener, sub_mask);

  subscription_->topics_[reader] = topic;
  subscription_->readers_[reader] = evolving_type_ptr;  /* Evolving type, dynamically created! */
  eprosima::fastrtps::types::DynamicData_ptr data(
    eprosima::fastrtps::types::DynamicDataFactory::get_instance()->create_data(evolving_type_ptr));
  subscription_->datas_[reader] = data;
}


void EvolvingSubscription::run()
{
  std::cout << "Subscriber running. Please press enter to stop the Subscriber" << std::endl;
  std::cin.ignore();
}


int main(int argc, char * argv[])
{
  (void)argc;
  (void)argv;

  EvolvingSubscription sub;
  if (sub.init()) {
    sub.run();
  }

  ser_support_fini(ser);
  return 0;
}

// Copyright 2019 Proyectos y Sistemas de Mantenimiento SL (eProsima).
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

#ifndef EVOLVING_SUBSCRIBER_H_
#define EVOLVING_SUBSCRIBER_H_

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantListener.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastrtps/subscriber/SampleInfo.h>
#include <fastrtps/rtps/common/Types.h>

#include <fastrtps/types/TypeIdentifier.h>
#include <fastrtps/types/TypeObject.h>

#include <fastrtps/attributes/SubscriberAttributes.h>

#include <map>

class EvolvingSubscription
{
public:
  EvolvingSubscription();
  virtual ~EvolvingSubscription();

  bool init();
  void run();

private:
  eprosima::fastdds::dds::DomainParticipant * mp_participant;
  eprosima::fastdds::dds::Subscriber * mp_subscriber;

  std::map < eprosima::fastdds::dds::DataReader *, eprosima::fastdds::dds::Topic * > topics_;
  std::map < eprosima::fastdds::dds::DataReader *,
  eprosima::fastrtps::types::DynamicType_ptr > dyn_types_;
  std::map < eprosima::fastdds::dds::DataReader *,
  eprosima::fastrtps::types::DynamicData_ptr > dyn_datas_;

  eprosima::fastrtps::SubscriberAttributes att_;
  eprosima::fastdds::dds::DataReaderQos qos_;

public:
  class SubListener: public eprosima::fastdds::dds::DomainParticipantListener
  {
public:
    SubListener(EvolvingSubscription * sub)
      : n_matched(0), n_samples(0), subscription_(sub)
    {
    }

    ~SubListener() override
    {
    }

    void on_data_available(eprosima::fastdds::dds::DataReader * reader) override;

    void on_subscription_matched(
      eprosima::fastdds::dds::DataReader * reader,
      const eprosima::fastdds::dds::SubscriptionMatchedStatus & info) override;

    void on_type_discovery(
      eprosima::fastdds::dds::DomainParticipant * participant,
      const eprosima::fastrtps::rtps::SampleIdentity & request_sample_id,
      const eprosima::fastrtps::string_255 & topic,
      const eprosima::fastrtps::types::TypeIdentifier * identifier,
      const eprosima::fastrtps::types::TypeObject * object,
      eprosima::fastrtps::types::DynamicType_ptr dyn_type) override;

    int n_matched;
    uint32_t n_samples;
    EvolvingSubscription * subscription_;
  } m_listener;
};

#endif /* EVOLVING_SUBSCRIBER_H_ */

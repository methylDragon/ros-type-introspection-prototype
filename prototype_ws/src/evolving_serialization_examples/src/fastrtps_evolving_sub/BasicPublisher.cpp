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

#include "BasicPublisher.h"

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

using namespace eprosima::fastdds::dds;


BasicPublisher::BasicPublisher()
: mp_participant(nullptr), mp_publisher(nullptr)
{
}


bool BasicPublisher::init()
{
  using namespace eprosima::fastrtps::types;

  // Create type
  auto type_factory = eprosima::fastrtps::types::DynamicTypeBuilderFactory::get_instance();
  DynamicTypeBuilder_ptr builder = type_factory->create_struct_builder();

  /*
   * TODO(CH3): Eventually, we want to do this automatically with the evolving lib
   */

  builder->add_member(0, "bool_field", type_factory->create_bool_type());
  builder->add_member(1, "byte_field", type_factory->create_byte_type());
  builder->add_member(2, "int32_field", type_factory->create_int32_type());
  builder->add_member(
    3, "string_field",
    type_factory->create_string_type(eprosima::fastrtps::types::MAX_STRING_LENGTH));
  builder->set_name("ExampleMsg");

  DynamicType_ptr example_msg_type(builder->build());

  // Create and Populate Data
  this->msg_data_ = DynamicDataFactory::get_instance()->create_data(example_msg_type);

  msg_data_->set_bool_value(true, 0);
  msg_data_->set_byte_value(0, 1);
  msg_data_->set_int32_value(1, 2);
  msg_data_->set_string_value("Some dynamic string value", 3);

  DynamicDataHelper::print(msg_data_);

  TypeSupport example_msg_ts(new eprosima::fastrtps::types::DynamicPubSubType(example_msg_type));

  // Setup pub
  DomainParticipantQos pqos;
  pqos.name("Participant_pub");
  mp_participant = DomainParticipantFactory::get_instance()->create_participant(0, pqos);

  if (mp_participant == nullptr) {
    return false;
  }

  example_msg_ts.get()->auto_fill_type_information(false);
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
  writer_ = mp_publisher->create_datawriter(topic_, DATAWRITER_QOS_DEFAULT, &m_listener);
  if (writer_ == nullptr) {
    return false;
  }

  return true;
}


BasicPublisher::~BasicPublisher()
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


void BasicPublisher::PubListener::on_publication_matched(
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


void BasicPublisher::runThread(uint32_t sleep)
{
  using namespace eprosima::fastrtps::types;

  while (!stop) {
    if (publish(true)) {
      std::cout << "\n== SENT ==" << std::endl;
      DynamicDataHelper::print(msg_data_);

      // std::string message;
      // m_Hello->get_string_value(message, 0);
      // uint32_t index;
      // m_Hello->get_uint32_value(index, 1);
      // std::string aux_array = "[";
      // eprosima::fastrtps::types::DynamicData * array = m_Hello->loan_value(2);
      // for (uint32_t i = 0; i < 5; ++i) {
      //   aux_array += "[";
      //   for (uint32_t j = 0; j < 2; ++j) {
      //     uint32_t elem;
      //     array->get_uint32_value(elem, array->get_array_index({i, j}));
      //     aux_array += std::to_string(elem) + (j == 1 ? "]" : ", ");
      //   }
      //   aux_array += (i == 4 ? "]" : "], ");
      // }
      // m_Hello->return_loaned_value(array);
      // std::cout << "Message: " << message << " with index: " << index
      //           << " array: " << aux_array << " SENT" << std::endl;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(sleep));
  }
}


void BasicPublisher::run(uint32_t sleep)
{
  stop = false;
  std::thread thread(&BasicPublisher::runThread, this, sleep);

  std::cout << "Publisher running. Please press enter to stop the Publisher at any time."
            << std::endl;
  std::cin.ignore();
  stop = true;

  thread.join();
}


bool BasicPublisher::publish(bool waitForListener)
{
  if (m_listener.firstConnected || !waitForListener || m_listener.n_matched > 0) {
    bool bool_;
    msg_data_->get_bool_value(bool_, 0);
    msg_data_->set_bool_value(!bool_, 0);

    unsigned char byte_;
    msg_data_->get_byte_value(byte_, 1);
    msg_data_->set_byte_value(++byte_, 1);


    int32_t index_;
    msg_data_->get_int32_value(index_, 2);
    msg_data_->set_int32_value(index_ + 1, 2);

    // eprosima::fastrtps::types::DynamicData * array = m_Hello->loan_value(2);
    // array->set_uint32_value(10 + index, array->get_array_index({0, 0}));
    // array->set_uint32_value(20 + index, array->get_array_index({1, 0}));
    // array->set_uint32_value(30 + index, array->get_array_index({2, 0}));
    // array->set_uint32_value(40 + index, array->get_array_index({3, 0}));
    // array->set_uint32_value(50 + index, array->get_array_index({4, 0}));
    // array->set_uint32_value(60 + index, array->get_array_index({0, 1}));
    // array->set_uint32_value(70 + index, array->get_array_index({1, 1}));
    // array->set_uint32_value(80 + index, array->get_array_index({2, 1}));
    // array->set_uint32_value(90 + index, array->get_array_index({3, 1}));
    // array->set_uint32_value(100 + index, array->get_array_index({4, 1}));
    // m_Hello->return_loaned_value(array);

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

  BasicPublisher pub;
  if (pub.init()) {
    pub.run(1000);
  }

  return 0;
}

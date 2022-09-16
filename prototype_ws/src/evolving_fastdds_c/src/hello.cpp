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


/* Description:
 *
 * Demonstration snippet for parsing a nested type description yaml into a
 * traversible tree, and how to traverse the tree by string references, with
 * indexing!
 *
 */

#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/attributes/PublisherAttributes.h>

#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>

#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/qos/PublisherQos.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/publisher/qos/DataWriterQos.hpp>

#include <fastrtps/types/DynamicDataFactory.h>
#include <fastrtps/types/DynamicDataHelper.hpp>

#include <fastrtps/types/DynamicTypeBuilderFactory.h>
#include <fastrtps/types/DynamicTypeBuilderPtr.h>

#include <fastrtps/types/DynamicTypePtr.h>


class PubListener : public eprosima::fastdds::dds::DataWriterListener
{
public:
  PubListener()
  : n_matched(0)
    , firstConnected(false)
  {
  }

  ~PubListener() override
  {
  }

  void on_publication_matched(
    eprosima::fastdds::dds::DataWriter * writer,
    const eprosima::fastdds::dds::PublicationMatchedStatus & info) override;
  int n_matched;

  bool firstConnected;
}
m_listener;


void PubListener::on_publication_matched(
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


using namespace eprosima;


int main()
{
  // TYPE CONSTRUCTION =============================================================================
  using namespace eprosima::fastrtps::types;

  // Create type
  DynamicTypeBuilder_ptr builder =
    DynamicTypeBuilderFactory::get_instance()->create_struct_builder();

  builder->add_member(
    0, "color",
    DynamicTypeBuilderFactory::get_instance()->create_string_type(100));
  builder->add_member(1, "x", DynamicTypeBuilderFactory::get_instance()->create_int32_type());
  builder->add_member(2, "y", DynamicTypeBuilderFactory::get_instance()->create_int32_type());
  builder->add_member(
    3, "shapesize",
    DynamicTypeBuilderFactory::get_instance()->create_int32_type());
  builder->set_name("ShapeType");

  DynamicType_ptr shape_type(builder->build());

  // Create and Populate Data
  DynamicData_ptr data(DynamicDataFactory::get_instance()->create_data(shape_type));

  data->set_string_value("Some dynamic string value", 0);
  data->set_int32_value(5, 1);  // Second argument is the index
  data->set_int32_value(10, data->get_member_id_by_name("y"));  // Or by name!

  // std::cout << "DATA: " << data->get_int32_value(data->get_member_id_by_name("x")) << std::endl;  // 5

  DynamicDataHelper::print(data);


  // PUBSUB ========================================================================================
  using namespace eprosima::fastdds::dds;

  TypeSupport shape_ts(new eprosima::fastrtps::types::DynamicPubSubType(shape_type));

  DomainParticipantQos pqos;
  pqos.name("ShapeType_pub");

  DomainParticipant * shape_participant =
    DomainParticipantFactory::get_instance()->create_participant(0, pqos);

  if (shape_participant == nullptr) {
    return false;
  }

  // REGISTER THE TYPE
  shape_ts.get()->auto_fill_type_information(false);
  shape_ts.get()->auto_fill_type_object(true);

  shape_ts.register_type(shape_participant);

  // CREATE THE PUBLISHER
  Publisher * shape_publisher = shape_participant->create_publisher(
    PUBLISHER_QOS_DEFAULT,
    nullptr);

  if (shape_publisher == nullptr) {
    return false;
  }

  Topic * topic_ = shape_participant->create_topic(
    "EvolvingSubTopic", "ShapeType",
    TOPIC_QOS_DEFAULT);

  if (topic_ == nullptr) {
    return false;
  }

  // CREATE THE WRITER
  DataWriter * writer_ = shape_publisher->create_datawriter(
    topic_, DATAWRITER_QOS_DEFAULT,
    &m_listener);

  if (writer_ == nullptr) {
    return false;
  }

  printf("WOW");
  // You can clear data with
  // data->clear_data();

  return 0;
}

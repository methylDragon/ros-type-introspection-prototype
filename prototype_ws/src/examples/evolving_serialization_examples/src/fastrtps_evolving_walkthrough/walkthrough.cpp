#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/attributes/SubscriberAttributes.h>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/subscriber/SampleInfo.hpp>
#include <fastdds/dds/subscriber/qos/DataReaderQos.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>

#include <fastrtps/types/DynamicDataFactory.h>
#include <fastrtps/types/DynamicTypeBuilderFactory.h>
#include <fastrtps/types/DynamicTypeBuilderPtr.h>

#include "rosidl_dynamic_typesupport/yaml_parser.h"
#include "rosidl_dynamic_typesupport/description.h"
#include "rosidl_dynamic_typesupport/tree_traverse.h"
#include "rosidl_dynamic_typesupport/api/serialization_support.h"
#include "rosidl_dynamic_typesupport_fastrtps/identifier.h"
#include "rosidl_dynamic_typesupport_fastrtps/serialization_support.h"

using namespace eprosima::fastrtps::types;

static rosidl_dynamic_typesupport_serialization_support_t * serialization_support = rosidl_dynamic_typesupport_serialization_support_init(
  rosidl_dynamic_typesupport_fastrtps_create_serialization_support_impl(),
  rosidl_dynamic_typesupport_fastrtps_create_serialization_support_interface());


int main(int argc, char * argv[])
{
  (void)argc;
  (void)argv;

  assert(rosidl_dynamic_typesupport_serialization_support_get_library_identifier(serialization_support) == fastrtps_serialization_support_library_identifier);

  // NOTE(methylDragon): The use of `auto` was deliberately avoided for explanatory purposes

  // FLAT EXAMPLE
  rosidl_dynamic_typesupport_dynamic_type_builder_t * flat_builder = rosidl_dynamic_typesupport_dynamic_type_builder_init(serialization_support, "flat");
  rosidl_dynamic_typesupport_dynamic_type_builder_add_bool_member(flat_builder, 0, "bool_field");
  rosidl_dynamic_typesupport_dynamic_type_builder_add_int32_member(flat_builder, 1, "int32_field");
  rosidl_dynamic_typesupport_dynamic_type_builder_add_string_member(flat_builder, 2, "string_field");

  rosidl_dynamic_typesupport_dynamic_data_t * flat_data = rosidl_dynamic_typesupport_dynamic_data_init_from_dynamic_type_builder(flat_builder);
  rosidl_dynamic_typesupport_dynamic_type_builder_destroy(flat_builder);

  printf("\n== FLAT DATA EXAMPLE ==\n");
  rosidl_dynamic_typesupport_dynamic_data_print(flat_data);
  rosidl_dynamic_typesupport_dynamic_data_destroy(flat_data);


  // SEQUENCE/ARRAY EXAMPLE
  int bound = 5;
  rosidl_dynamic_typesupport_dynamic_type_builder_t * seq_builder = rosidl_dynamic_typesupport_dynamic_type_builder_init(serialization_support, "seq");

  rosidl_dynamic_typesupport_dynamic_type_builder_add_bool_array_member(seq_builder, 0, "bool_array_field", bound);
  rosidl_dynamic_typesupport_dynamic_type_builder_add_int16_unbounded_sequence_member(seq_builder, 1, "int16_array_field");
  rosidl_dynamic_typesupport_dynamic_type_builder_add_int32_bounded_sequence_member(seq_builder, 2, "int32_seq_field", bound);
  rosidl_dynamic_typesupport_dynamic_type_builder_add_string_member(seq_builder, 3, "string_field");
  rosidl_dynamic_typesupport_dynamic_type_builder_add_string_array_member(seq_builder, 4, "string_array_field", bound);

  rosidl_dynamic_typesupport_dynamic_data_t * seq_data = rosidl_dynamic_typesupport_dynamic_data_init_from_dynamic_type_builder(seq_builder);
  rosidl_dynamic_typesupport_dynamic_type_builder_destroy(seq_builder);

  printf("\n== SEQ DATA EXAMPLE ==\n");
  rosidl_dynamic_typesupport_dynamic_data_t * int16_seq = rosidl_dynamic_typesupport_dynamic_data_loan_value(seq_data, 1);
  rosidl_dynamic_typesupport_dynamic_data_t * int32_seq = rosidl_dynamic_typesupport_dynamic_data_loan_value(seq_data, 2);

  rosidl_dynamic_typesupport_member_id_t id;

  for (int i = 0; i < 50; i++) {
    rosidl_dynamic_typesupport_dynamic_data_insert_int16_value(int16_seq, i + 1, &id);
  }

  for (int i = 0; i < 6; i++) {
    if (i == 5) {
      std::cout << "We're purposely attempting to inserting the 6th element into the 5-bounded "
                << "int32_seq_field! Expect an error!" << std::endl;
    }
    rosidl_dynamic_typesupport_dynamic_data_insert_int32_value(int32_seq, i + 1, &id);
  }

  sleep(1);
  std::cout << std::endl;

  rosidl_dynamic_typesupport_dynamic_data_return_loaned_value(seq_data, int16_seq);
  rosidl_dynamic_typesupport_dynamic_data_return_loaned_value(seq_data, int32_seq);

  rosidl_dynamic_typesupport_dynamic_data_print(seq_data);
  rosidl_dynamic_typesupport_dynamic_data_destroy(seq_data);


  // NESTED EXAMPLE
  rosidl_dynamic_typesupport_dynamic_type_builder_t * inner_builder = rosidl_dynamic_typesupport_dynamic_type_builder_init(serialization_support, "inner");
  rosidl_dynamic_typesupport_dynamic_type_builder_add_bool_member(inner_builder, 0, "inner_bool_field");
  rosidl_dynamic_typesupport_dynamic_type_t * inner_type = rosidl_dynamic_typesupport_dynamic_type_init_from_dynamic_type_builder(inner_builder);

  rosidl_dynamic_typesupport_dynamic_type_builder_t * outer_builder = rosidl_dynamic_typesupport_dynamic_type_builder_init(serialization_support, "outer");
  rosidl_dynamic_typesupport_dynamic_type_builder_add_bool_member(outer_builder, 0, "outer_bool_field");
  rosidl_dynamic_typesupport_dynamic_type_builder_add_complex_member(outer_builder, 1, "outer_nested_field", inner_type);

  rosidl_dynamic_typesupport_dynamic_data_t * nested_data = rosidl_dynamic_typesupport_dynamic_data_init_from_dynamic_type_builder(outer_builder);
  rosidl_dynamic_typesupport_dynamic_type_builder_destroy(inner_builder);
  rosidl_dynamic_typesupport_dynamic_type_builder_destroy(outer_builder);
  rosidl_dynamic_typesupport_dynamic_type_destroy(inner_type);

  printf("\n== NESTED DATA EXAMPLE ==\n");
  rosidl_dynamic_typesupport_dynamic_data_print(nested_data);
  rosidl_dynamic_typesupport_dynamic_data_destroy(nested_data);


  // FROM YAML
  // This tests almost everything! Sequences, nesting, etc.!
  char * nested_yaml_path =
    g_strjoin("/", g_path_get_dirname(__FILE__), "..", "..", "msg", "nested.yaml", NULL);

  type_description_t * yaml_description =
    create_type_description_from_yaml_file(nested_yaml_path);
  // print_type_description(yaml_description);

  rosidl_dynamic_typesupport_dynamic_type_t * yaml_type = rosidl_dynamic_typesupport_dynamic_type_init_from_description(serialization_support, yaml_description);
  rosidl_dynamic_typesupport_dynamic_data_t * yaml_data = rosidl_dynamic_typesupport_dynamic_data_init_from_dynamic_type(yaml_type);

  printf("\n== NESTED DATA FROM YAML EXAMPLE ==\n");
  rosidl_dynamic_typesupport_dynamic_data_print(yaml_data);
  rosidl_dynamic_typesupport_dynamic_data_destroy(yaml_data);
  rosidl_dynamic_typesupport_dynamic_type_destroy(yaml_type);


  return 0;
}

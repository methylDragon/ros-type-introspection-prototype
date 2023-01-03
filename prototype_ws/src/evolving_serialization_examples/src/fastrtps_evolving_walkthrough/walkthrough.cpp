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

#include "serialization_support_lib/yaml_parser.h"
#include "serialization_support_lib/description.h"
#include "serialization_support_lib/tree_traverse.h"
#include "serialization_support_lib/api/serialization_support.h"
#include "serialization_support_fastrtps_c/identifier.h"
#include "serialization_support_fastrtps_c/serialization_support.h"

using namespace eprosima::fastrtps::types;

static serialization_support_t * ser = ser_support_init(
  create_fastrtps_ser_impl(),
  create_fastrtps_ser_interface());


int main(int argc, char * argv[])
{
  (void)argc;
  (void)argv;

  assert(ser_get_library_identifier(ser) == fastrtps_serialization_support_library_identifier);

  // NOTE(methylDragon): The use of `auto` was deliberately avoided for explanatory purposes

  // FLAT EXAMPLE
  ser_type_builder_t * flat_builder = ser_struct_type_builder_init(ser, "flat");
  ser_add_bool_member(ser, flat_builder, 0, "bool_field");
  ser_add_int32_member(ser, flat_builder, 1, "int32_field");
  ser_add_string_member(ser, flat_builder, 2, "string_field");

  ser_dynamic_data_t * flat_data = ser_data_init_from_builder(ser, flat_builder);
  ser_struct_type_builder_fini(ser, flat_builder);

  printf("\n== FLAT DATA EXAMPLE ==\n");
  ser_print_dynamic_data(ser, flat_data);
  ser_data_fini(ser, flat_data);


  // SEQUENCE/ARRAY EXAMPLE
  int bound = 5;
  ser_type_builder_t * seq_builder = ser_struct_type_builder_init(ser, "flat");

  ser_add_bool_static_array_member(ser, seq_builder, 0, "bool_array_field", bound);
  ser_add_int16_unbounded_sequence_member(ser, seq_builder, 1, "int16_array_field");
  ser_add_int32_bounded_sequence_member(ser, seq_builder, 2, "int32_seq_field", bound);
  ser_add_string_member(ser, seq_builder, 3, "string_field");
  ser_add_string_static_array_member(ser, seq_builder, 4, "string_array_field", bound);

  ser_dynamic_data_t * seq_data = ser_data_init_from_builder(ser, seq_builder);
  ser_struct_type_builder_fini(ser, seq_builder);

  printf("\n== SEQ DATA EXAMPLE ==\n");
  ser_dynamic_data_t * int16_seq = ser_loan_value(ser, seq_data, 1);
  ser_dynamic_data_t * int32_seq = ser_loan_value(ser, seq_data, 2);

  MemberId id;

  for (int i = 0; i < 50; i++) {
    ser_insert_int16_value(ser, int16_seq, i + 1, &id);
  }

  for (int i = 0; i < 6; i++) {
    if (i == 5) {
      std::cout << "We're purposely attempting to inserting the 6th element into the 5-bounded "
                << "int32_seq_field! Expect an error!" << std::endl;
    }
    ser_insert_int32_value(ser, int32_seq, i + 1, &id);
  }

  sleep(1);
  std::cout << std::endl;

  ser_return_loaned_value(ser, seq_data, int16_seq);
  ser_return_loaned_value(ser, seq_data, int32_seq);

  ser_print_dynamic_data(ser, seq_data);
  ser_data_fini(ser, seq_data);


  // NESTED EXAMPLE
  ser_type_builder_t * inner_builder = ser_struct_type_builder_init(ser, "inner");
  ser_add_bool_member(ser, inner_builder, 0, "inner_bool_field");
  ser_dynamic_type_t * inner_type = ser_build_struct_type(ser, inner_builder);

  ser_type_builder_t * outer_builder = ser_struct_type_builder_init(ser, "outer");
  ser_add_bool_member(ser, outer_builder, 0, "outer_bool_field");
  ser_add_nested_struct_member(ser, outer_builder, 1, "outer_nested_field", inner_type);

  ser_dynamic_data_t * nested_data = ser_data_init_from_builder(ser, outer_builder);
  ser_struct_type_builder_fini(ser, inner_builder);
  ser_struct_type_builder_fini(ser, outer_builder);
  ser_type_fini(ser, inner_type);

  printf("\n== NESTED DATA EXAMPLE ==\n");
  ser_print_dynamic_data(ser, nested_data);
  ser_data_fini(ser, nested_data);


  // FROM YAML
  // This tests almost everything! Sequences, nesting, etc.!
  char * nested_yaml_path =
    g_strjoin("/", g_path_get_dirname(__FILE__), "..", "..", "msg", "nested.yaml", NULL);

  type_description_t * yaml_description =
    create_type_description_from_yaml_file(nested_yaml_path);
  // print_type_description(yaml_description);

  ser_dynamic_type_t * yaml_type = ser_construct_type_from_description(ser, yaml_description);
  ser_dynamic_data_t * yaml_data = ser_data_init_from_type(ser, yaml_type);

  printf("\n== NESTED DATA FROM YAML EXAMPLE ==\n");
  ser_print_dynamic_data(ser, yaml_data);
  ser_data_fini(ser, yaml_data);
  ser_type_fini(ser, yaml_type);


  return 0;
}

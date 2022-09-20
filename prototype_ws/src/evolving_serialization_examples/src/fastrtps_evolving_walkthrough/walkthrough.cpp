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

#include "evolving_serialization_lib/yaml_parser.h"
#include "evolving_serialization_lib/description.h"
#include "evolving_serialization_lib/tree_traverse.h"
#include "evolving_serialization_lib/evolving_type_support.h"
#include "evolving_fastrtps_c/type_support.h"

using namespace eprosima::fastrtps::types;

static EvolvingTypeSupport * ets = create_evolving_typesupport(
  create_fastrtps_evolving_typesupport_impl(),
  create_fastrtps_evolving_typesupport_interface());


int main(int argc, char * argv[])
{
  (void)argc;
  (void)argv;


  // FLAT EXAMPLE
  auto flat_builder = ets_create_struct_builder(ets, "flat");
  ets_add_bool_member(ets, flat_builder, 0, "bool_field");
  ets_add_int32_member(ets, flat_builder, 1, "int32_field");
  ets_add_string_member(ets, flat_builder, 2, "string_field");

  DynamicData_ptr flat_data(DynamicDataFactory::get_instance()->create_data(
      static_cast<DynamicTypeBuilder *>(flat_builder))
  );

  printf("\n== FLAT DATA EXAMPLE ==\n");
  ets_print_dynamic_data(ets, flat_data.get());


  // SEQUENCE/ARRAY EXAMPLE
  int bound = 5;
  auto seq_builder = ets_create_struct_builder(ets, "flat");

  ets_add_bool_static_array_member(ets, seq_builder, 0, "bool_array_field", bound);
  ets_add_int16_unbounded_sequence_member(ets, seq_builder, 1, "int16_array_field");
  ets_add_int32_bounded_sequence_member(ets, seq_builder, 2, "int32_seq_field", bound);
  ets_add_string_member(ets, seq_builder, 3, "string_field");
  ets_add_string_static_array_member(ets, seq_builder, 4, "string_array_field", bound);

  DynamicData_ptr seq_data(DynamicDataFactory::get_instance()->create_data(
      static_cast<DynamicTypeBuilder *>(seq_builder))
  );

  printf("\n== SEQ DATA EXAMPLE ==\n");
  DynamicData * int16_seq = seq_data->loan_value(1);
  DynamicData * int32_seq = seq_data->loan_value(2);

  MemberId id;

  for (int i = 0; i < 50; i++) {
    int16_seq->insert_int16_value(i + 1, id);
  }

  for (int i = 0; i < 6; i++) {
    if (i == 5) {
      std::cout << "We're purposely attempting to inserting the 6th element into the 5-bounded "
                << "int32_seq_field! Expect an error!" << std::endl;
    }
    int32_seq->insert_int32_value(i + 1, id);
  }

  sleep(1);
  std::cout << std::endl;

  seq_data->return_loaned_value(int16_seq);
  seq_data->return_loaned_value(int32_seq);

  ets_print_dynamic_data(ets, seq_data.get());


  // NESTED EXAMPLE
  auto inner_builder = ets_create_struct_builder(ets, "inner");
  ets_add_bool_member(ets, inner_builder, 0, "inner_bool_field");

  auto outer_builder = ets_create_struct_builder(ets, "outer");
  ets_add_bool_member(ets, outer_builder, 0, "outer_bool_field");
  ets_add_nested_struct_member(
    ets, outer_builder, 1, "outer_nested_field",
    ets_finalize_struct_builder(ets, inner_builder)
  );

  DynamicData_ptr nested_data(DynamicDataFactory::get_instance()->create_data(
      static_cast<DynamicTypeBuilder *>(outer_builder))
  );

  printf("\n== NESTED DATA EXAMPLE ==\n");
  ets_print_dynamic_data(ets, nested_data.get());


  // FROM YAML
  // This tests almost everything! Sequences, nesting, etc.!
  char * nested_yaml_path =
    g_strjoin("/", g_path_get_dirname(__FILE__), "..", "..", "msg", "nested.yaml", NULL);

  type_description_t * yaml_description =
    create_type_description_from_yaml(nested_yaml_path);
  // print_type_description(yaml_description);

  // Yes.. I know it's disgusting
  DynamicData_ptr yaml_data(
    DynamicDataFactory::get_instance()->create_data(
      eprosima::fastrtps::types::DynamicType_ptr(
        std::move(
          *reinterpret_cast<eprosima::fastrtps::types::DynamicType_ptr *>(
            ets_construct_type_from_description(ets, yaml_description)
          )
        )
      )
    )
  );

  printf("\n== NESTED DATA FROM YAML EXAMPLE ==\n");
  ets_print_dynamic_data(ets, yaml_data.get());


  return 0;
}

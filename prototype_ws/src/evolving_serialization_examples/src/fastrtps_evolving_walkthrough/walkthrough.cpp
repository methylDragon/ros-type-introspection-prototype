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


  // NESTED EXAMPLE
  auto inner_builder = ets_create_struct_builder(ets, "inner");
  ets_add_bool_member(ets, inner_builder, 0, "inner_bool_field");

  auto outer_builder = ets_create_struct_builder(ets, "outer");
  ets_add_bool_member(ets, outer_builder, 0, "outer_bool_field");
  ets_add_nested_struct_member(
    ets, outer_builder, 1, "outer_nested_field",
    ets_finalize_struct_builder(ets, inner_builder));

  DynamicData_ptr nested_data(DynamicDataFactory::get_instance()->create_data(
      static_cast<DynamicTypeBuilder *>(outer_builder))
  );

  printf("\n== NESTED DATA EXAMPLE ==\n");
  ets_print_dynamic_data(ets, nested_data.get());


  // FROM YAML
  char * nested_yaml_path =
    g_strjoin("/", g_path_get_dirname(__FILE__), "..", "..", "msg", "nested.yaml", NULL);

  type_description_t * yaml_description =
    create_type_description_from_yaml(nested_yaml_path);
  print_type_description(yaml_description);

  DynamicData_ptr yaml_data(DynamicDataFactory::get_instance()->create_data(
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

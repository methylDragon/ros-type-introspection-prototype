# YAML Parsing =====================================================================================
add_executable(flat_msg_yaml_parsing "flat_msg_yaml_parsing.c")
ament_target_dependencies(flat_msg_yaml_parsing serialization_support_lib)

add_executable(nested_msg_yaml_parsing "nested_msg_yaml_parsing.c")
ament_target_dependencies(nested_msg_yaml_parsing serialization_support_lib)

install(TARGETS flat_msg_yaml_parsing RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS nested_msg_yaml_parsing RUNTIME DESTINATION lib/${PROJECT_NAME})

# YAML Parsing =====================================================================================
add_executable(flat_msg_yaml_parsing "flat_msg_yaml_parsing.c")
ament_target_dependencies(flat_msg_yaml_parsing rosidl_dynamic_typesupport)

add_executable(nested_msg_yaml_parsing "nested_msg_yaml_parsing.c")
ament_target_dependencies(nested_msg_yaml_parsing rosidl_dynamic_typesupport)

install(TARGETS flat_msg_yaml_parsing RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS nested_msg_yaml_parsing RUNTIME DESTINATION lib/${PROJECT_NAME})

find_package(evolving_protobuf_c)

# Protobuf Evolving Pub Sub ========================================================================
include_directories(${evolving_protobuf_c_INCLUDE_DIRS})

add_executable(protogen_demo "protogen_demo.cpp")
target_link_libraries(protogen_demo
  ${evolving_serialization_lib_LIBRARIES}
  ${evolving_protobuf_c_LIBRARIES}
  protobuf
)

# add_executable(proto_from_description "create_proto_from_description.cpp")
# target_link_libraries(proto_from_description
#   ${evolving_serialization_lib_LIBRARIES}
#   ${evolving_protobuf_c_LIBRARIES}
#   protobuf
# )
#
# add_executable(proto_from_yaml "create_proto_from_yaml.cpp")
# target_link_libraries(proto_from_yaml
#   ${evolving_serialization_lib_LIBRARIES}
#   ${evolving_protobuf_c_LIBRARIES}
#   protobuf
# )

install(TARGETS protogen_demo RUNTIME DESTINATION lib/${PROJECT_NAME})
# install(TARGETS proto_from_description RUNTIME DESTINATION lib/${PROJECT_NAME})
# install(TARGETS proto_from_yaml RUNTIME DESTINATION lib/${PROJECT_NAME})

find_package(evolving_protobuf_c)

# Protobuf Evolving Pub Sub ========================================================================
add_executable(protogen_demo "protogen_demo.cpp")
target_link_libraries(protogen_demo
  evolving_protobuf_c::protogen
)

add_executable(protobuf_evolving_ser "protobuf_evolving_ser.cpp")
target_link_libraries(protobuf_evolving_ser
  evolving_protobuf_c::protogen
)

install(TARGETS protogen_demo RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS protobuf_evolving_ser RUNTIME DESTINATION lib/${PROJECT_NAME})

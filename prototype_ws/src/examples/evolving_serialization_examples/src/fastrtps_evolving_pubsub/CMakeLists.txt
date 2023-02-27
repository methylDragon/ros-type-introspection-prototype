find_package(rosidl_dynamic_typesupport_fastrtps REQUIRED)

# FastRTPS Evolving Pub Sub ========================================================================
add_executable(fastrtps_evolving_pub "EvolvingPublisher.cpp")
target_link_libraries(fastrtps_evolving_pub
  rosidl_dynamic_typesupport_fastrtps::rosidl_dynamic_typesupport_fastrtps
)

add_executable(fastrtps_evolving_sub "EvolvingSubscription.cpp")
target_link_libraries(fastrtps_evolving_sub
  rosidl_dynamic_typesupport_fastrtps::rosidl_dynamic_typesupport_fastrtps
)

install(TARGETS fastrtps_evolving_pub RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS fastrtps_evolving_sub RUNTIME DESTINATION lib/${PROJECT_NAME})

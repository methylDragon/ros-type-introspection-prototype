find_package(rosidl_dynamic_typesupport_fastrtps REQUIRED)

# FastRTPS Evolving Walkthrough ====================================================================
add_executable(fastrtps_evolving_walkthrough "walkthrough.cpp")
target_link_libraries(fastrtps_evolving_walkthrough
  rosidl_dynamic_typesupport_fastrtps::rosidl_dynamic_typesupport_fastrtps
)

install(TARGETS fastrtps_evolving_walkthrough RUNTIME DESTINATION lib/${PROJECT_NAME})

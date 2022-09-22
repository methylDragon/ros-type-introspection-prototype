# FastRTPS Evolving Walkthrough ====================================================================
add_executable(fastrtps_evolving_walkthrough "walkthrough.cpp")
target_link_libraries(fastrtps_evolving_walkthrough
  ${evolving_serialization_lib_LIBRARIES}
  ${evolving_fastrtps_c_LIBRARIES}
  fastrtps
  fastcdr
)

install(TARGETS fastrtps_evolving_walkthrough RUNTIME DESTINATION lib/${PROJECT_NAME})

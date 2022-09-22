# FastRTPS Evolving Pub Sub ========================================================================
add_executable(fastrtps_evolving_pub
  "EvolvingPublisher.cpp")
target_link_libraries(fastrtps_evolving_pub
  ${evolving_serialization_lib_LIBRARIES}
  ${evolving_fastrtps_c_LIBRARIES}
  fastrtps
  fastcdr
)

add_executable(fastrtps_evolving_sub
  "EvolvingSubscription.cpp")
target_link_libraries(fastrtps_evolving_sub
  ${evolving_serialization_lib_LIBRARIES}
  ${evolving_fastrtps_c_LIBRARIES}
  fastrtps
  fastcdr
)

install(TARGETS fastrtps_evolving_pub RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS fastrtps_evolving_sub RUNTIME DESTINATION lib/${PROJECT_NAME})

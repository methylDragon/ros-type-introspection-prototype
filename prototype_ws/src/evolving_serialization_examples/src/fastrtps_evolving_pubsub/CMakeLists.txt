find_package(serialization_support_fastrtps_c REQUIRED)

# FastRTPS Evolving Pub Sub ========================================================================
add_executable(fastrtps_evolving_pub "EvolvingPublisher.cpp")
target_link_libraries(fastrtps_evolving_pub
  serialization_support_fastrtps_c::serialization_support_fastrtps_c
)

add_executable(fastrtps_evolving_sub "EvolvingSubscription.cpp")
target_link_libraries(fastrtps_evolving_sub
  serialization_support_fastrtps_c::serialization_support_fastrtps_c
)

install(TARGETS fastrtps_evolving_pub RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS fastrtps_evolving_sub RUNTIME DESTINATION lib/${PROJECT_NAME})

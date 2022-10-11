find_package(evolving_fastrtps_c REQUIRED)

# FastRTPS Evolving Walkthrough ====================================================================
add_executable(fastrtps_evolving_walkthrough "walkthrough.cpp")
target_link_libraries(fastrtps_evolving_walkthrough
  evolving_fastrtps_c::evolving_fastrtps_c
)

install(TARGETS fastrtps_evolving_walkthrough RUNTIME DESTINATION lib/${PROJECT_NAME})

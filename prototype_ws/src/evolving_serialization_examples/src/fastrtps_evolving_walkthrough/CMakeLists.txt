find_package(evolving_fastrtps_c REQUIRED)
find_package(fastrtps "2.8.0" REQUIRED)

# FastRTPS Evolving Walkthrough ====================================================================
include_directories(${evolving_fastrtps_c_INCLUDE_DIRS})

add_executable(fastrtps_evolving_walkthrough "walkthrough.cpp")
target_link_libraries(fastrtps_evolving_walkthrough
  ${evolving_serialization_lib_LIBRARIES}
  ${evolving_fastrtps_c_LIBRARIES}
  fastrtps
  fastcdr
)

install(TARGETS fastrtps_evolving_walkthrough RUNTIME DESTINATION lib/${PROJECT_NAME})

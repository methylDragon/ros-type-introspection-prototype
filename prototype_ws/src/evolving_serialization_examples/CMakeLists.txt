cmake_minimum_required(VERSION 3.5)
project(evolving_serialization_examples)
set(CMAKE_VERBOSE_MAKEFILE ON)

#################################################
# compiler settings
#################################################

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_VERBOSE_MAKEFILE ON)

#################################################
# package dependencies
#################################################
find_package(ament_cmake_ros REQUIRED)
find_package(evolving_serialization_lib REQUIRED)
find_package(evolving_fastrtps_c REQUIRED)
find_package(fastrtps "2.8.0" REQUIRED)

include_directories(
  include
  ${evolving_serialization_lib_INCLUDE_DIRS}
  ${evolving_fastrtps_c_INCLUDE_DIRS}
)


# YAML Parsing
add_executable(flat_msg_yaml_parsing "src/yaml_parsing/flat_msg_yaml_parsing.c")
ament_target_dependencies(flat_msg_yaml_parsing evolving_serialization_lib)

add_executable(nested_msg_yaml_parsing "src/yaml_parsing/nested_msg_yaml_parsing.c")
ament_target_dependencies(nested_msg_yaml_parsing evolving_serialization_lib)

install(TARGETS flat_msg_yaml_parsing RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS nested_msg_yaml_parsing RUNTIME DESTINATION lib/${PROJECT_NAME})


# Evolving Sub
add_executable(fastrtps_evolving_sub_publisher
  "src/fastrtps_evolving_sub/BasicPublisher.cpp")
target_link_libraries(fastrtps_evolving_sub_publisher
  ${evolving_serialization_lib_LIBRARIES}
  ${evolving_fastrtps_c_LIBRARIES}
  fastrtps
  fastcdr
)

add_executable(fastrtps_evolving_sub_evolving_subscription
  "src/fastrtps_evolving_sub/EvolvingSubscription.cpp")
target_link_libraries(fastrtps_evolving_sub_evolving_subscription
  ${evolving_serialization_lib_LIBRARIES}
  ${evolving_fastrtps_c_LIBRARIES}
  fastrtps
  fastcdr
)

install(TARGETS fastrtps_evolving_sub_publisher RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS fastrtps_evolving_sub_evolving_subscription RUNTIME DESTINATION lib/${PROJECT_NAME})


# FastRTPS Evolving Walkthrough
add_executable(fastrtps_evolving_walkthrough "src/fastrtps_evolving_walkthrough/walkthrough.cpp")
target_link_libraries(fastrtps_evolving_walkthrough
  ${evolving_serialization_lib_LIBRARIES}
  ${evolving_fastrtps_c_LIBRARIES}
  fastrtps
  fastcdr
)

install(TARGETS fastrtps_evolving_walkthrough RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()

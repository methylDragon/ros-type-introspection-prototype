cmake_minimum_required(VERSION 3.5)
project(evolving_serialization_examples)
set(CMAKE_VERBOSE_MAKEFILE ON)


# COMPILER SETTINGS ================================================================================
# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_VERBOSE_MAKEFILE ON)


# DEPS =============================================================================================
find_package(ament_cmake_ros REQUIRED)
find_package(rosidl_dynamic_typesupport REQUIRED)


# TARGETS ==========================================================================================
add_subdirectory(src/fastrtps_evolving_pubsub)
add_subdirectory(src/fastrtps_evolving_walkthrough)
add_subdirectory(src/protobuf_evolving_ser)
add_subdirectory(src/yaml_parsing)

ament_package()

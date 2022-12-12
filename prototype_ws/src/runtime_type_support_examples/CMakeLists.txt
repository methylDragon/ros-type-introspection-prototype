cmake_minimum_required(VERSION 3.5)
project(runtime_type_support_examples)
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
find_package(serialization_support_lib REQUIRED)
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)


# TARGETS ==========================================================================================
add_executable(main "src/main.cpp")
target_link_libraries(main
  serialization_support_lib::serialization_support_lib
  rcl::rcl
  rclcpp::rclcpp
)
# ament_target_dependencies(main
#   "rcl"
# )

install(TARGETS main RUNTIME DESTINATION lib/${PROJECT_NAME})
ament_package()

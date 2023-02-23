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
find_package(runtime_type_support_examples_msgs REQUIRED)


# TARGETS ==========================================================================================
add_executable(runtime_sub_serialized "src/runtime_sub_serialized.cpp")
target_link_libraries(runtime_sub_serialized
  serialization_support_lib::serialization_support_lib
  rcl::rcl
  rclcpp::rclcpp
)

add_executable(runtime_sub_direct "src/runtime_sub_direct.cpp")
target_link_libraries(runtime_sub_direct
  serialization_support_lib::serialization_support_lib
  rcl::rcl
  rclcpp::rclcpp
)

add_executable(static_sub "src/static_sub.cpp")
target_link_libraries(static_sub
  ${runtime_type_support_examples_msgs_TARGETS}
  rclcpp::rclcpp
)

add_executable(static_pub "src/static_pub.cpp")
target_link_libraries(static_pub
  ${runtime_type_support_examples_msgs_TARGETS}
  rclcpp::rclcpp
)


# INSTALL ==========================================================================================
install(TARGETS runtime_sub_serialized RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS runtime_sub_direct RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS static_sub RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS static_pub RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()

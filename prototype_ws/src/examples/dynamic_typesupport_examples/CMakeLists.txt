cmake_minimum_required(VERSION 3.5)
project(dynamic_typesupport_examples)
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
find_package(rcl REQUIRED)
find_package(rclcpp REQUIRED)
find_package(dynamic_typesupport_examples_msgs REQUIRED)

find_package(fastrtps_cmake_module REQUIRED)
find_package(fastcdr REQUIRED CONFIG)
find_package(fastrtps 2.3 REQUIRED CONFIG)
find_package(FastRTPS 2.3 REQUIRED MODULE)

# TARGETS ==========================================================================================
include_directories(src)

add_executable(dynamic_sub_serialized
  "src/dynamic_sub_serialized.cpp"
  "src/detail/create_description.cpp")
target_link_libraries(dynamic_sub_serialized
  rosidl_dynamic_typesupport::rosidl_dynamic_typesupport
  rcl::rcl
  rclcpp::rclcpp
  fastcdr
  fastrtps
)

add_executable(dynamic_sub_direct
  "src/dynamic_sub_direct.cpp"
  "src/detail/create_description.cpp")
target_link_libraries(dynamic_sub_direct
  rosidl_dynamic_typesupport::rosidl_dynamic_typesupport
  rcl::rcl
  rclcpp::rclcpp
  fastcdr
  fastrtps
)

add_executable(static_sub "src/static_sub.cpp")
target_link_libraries(static_sub
  ${dynamic_typesupport_examples_msgs_TARGETS}
  rclcpp::rclcpp
)

add_executable(static_pub "src/static_pub.cpp")
target_link_libraries(static_pub
  ${dynamic_typesupport_examples_msgs_TARGETS}
  rclcpp::rclcpp
)


# INSTALL ==========================================================================================
install(TARGETS dynamic_sub_serialized RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS dynamic_sub_direct RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS static_sub RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS static_pub RUNTIME DESTINATION lib/${PROJECT_NAME})

ament_package()

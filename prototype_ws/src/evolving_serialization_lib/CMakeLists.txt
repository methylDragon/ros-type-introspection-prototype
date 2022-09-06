cmake_minimum_required(VERSION 3.5)
project(evolving_serialization_lib)
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
find_package(rcl REQUIRED)
find_package(yaml REQUIRED)

# Glib
find_package(PkgConfig REQUIRED)
pkg_search_module(GLIB REQUIRED glib-2.0)
include_directories(${GLIB_INCLUDE_DIRS})
# link_directories(${GLIB_LIBRARY_DIRS})
# add_definitions(${GLIB_CFLAGS_OTHER})

# find_package(rcl_action REQUIRED)
# find_package(rcutils REQUIRED)
# find_package(rosidl_generator_c REQUIRED)
# find_package(std_msgs REQUIRED)

#set(CMAKE_MODULE_PATH
#    ${CMAKE_MODULE_PATH}
#    "${CMAKE_CURRENT_SOURCE_DIR}/../../../resources/cmake"
#)

set(DEPENDENCIES
  rcl
  PkgConfig
  yaml
  GLIB
)

add_library(impl_utils
  "src/impl/yaml_parser.c"
  "src/impl/tree_traverse.c"
)
target_include_directories(impl_utils PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>")
ament_target_dependencies(impl_utils ${DEPENDENCIES})

add_executable(flat_msg_yaml_parsing "src/flat_msg_yaml_parsing.c")
target_link_libraries(flat_msg_yaml_parsing impl_utils)
ament_target_dependencies(flat_msg_yaml_parsing ${DEPENDENCIES})

add_executable(nested_msg_yaml_parsing "src/nested_msg_yaml_parsing.c")
target_link_libraries(nested_msg_yaml_parsing impl_utils)
ament_target_dependencies(nested_msg_yaml_parsing ${DEPENDENCIES})

install(TARGETS flat_msg_yaml_parsing RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS nested_msg_yaml_parsing RUNTIME DESTINATION lib/${PROJECT_NAME})

install(TARGETS impl_utils
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(ament_cmake)
ament_export_dependencies(rcl)
# ament_export_dependencies(rcutils)
# ament_export_dependencies(rosidl_generator_c)
ament_package()

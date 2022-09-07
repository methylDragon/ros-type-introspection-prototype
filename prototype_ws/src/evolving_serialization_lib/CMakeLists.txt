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

set(DEPENDENCIES
  rcl
  PkgConfig
  yaml
  GLIB
)

include_directories(
  ${GLIB_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_CURRENT_SOURCE_DIR}/src
)

add_library(${PROJECT_NAME}
  "src/impl/tree_traverse_impl.c"

  "src/yaml_parser.c"
  "src/tree_traverse.c"
  "src/description.c"
)
ament_target_dependencies(${PROJECT_NAME} ${DEPENDENCIES})

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})
ament_export_targets(${PROJECT_NAME})

ament_export_dependencies(ament_cmake)
ament_export_dependencies(rcl)

ament_package()

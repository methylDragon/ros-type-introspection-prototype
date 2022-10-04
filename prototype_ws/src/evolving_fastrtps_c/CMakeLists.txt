cmake_minimum_required(VERSION 3.5)
project(evolving_fastrtps_c)
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
find_package(fastcdr REQUIRED CONFIG)
find_package(fastrtps "2.8.0" REQUIRED CONFIG)

include_directories(
  include
  ${evolving_serialization_lib_INCLUDE_DIRS}
  ${fastcdr_INCLUDE_DIRS}
  ${fastrtps_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  "src/type_support.cpp")
target_link_libraries(${PROJECT_NAME}
  ${evolving_serialization_lib_LIBRARIES}
  fastcdr
  fastrtps
)

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

ament_export_dependencies(fastrtps)
ament_export_dependencies(fastcdr)
ament_package()

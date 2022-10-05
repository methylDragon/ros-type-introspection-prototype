cmake_minimum_required(VERSION 3.5)
project(evolving_protobuf_c)
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

# include(FindProtobuf)
find_package(Protobuf REQUIRED)

include_directories(
  include
  ${evolving_serialization_lib_INCLUDE_DIRS}
  ${PROTOBUF_INCLUDE_DIR}
)

add_library(protogen "src/protogen.c")  # Utils for generating .proto files
target_link_libraries(protogen
  ${evolving_serialization_lib_LIBRARIES}
  ${PROTOBUF_LIBRARIES}
)

# add_library(${PROJECT_NAME}
#   "src/type_support.cpp")
# target_link_libraries(${PROJECT_NAME}
#   ${evolving_serialization_lib_LIBRARIES}
#   ${PROTOBUF_LIBRARIES}
# )

install(TARGETS protogen EXPORT protogen
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(protogen)
ament_export_targets(protogen)

ament_export_dependencies(Protobuf)
ament_package()

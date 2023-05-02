cmake_minimum_required(VERSION 3.5)
project(evolving_protobuf_c)
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
find_package(Protobuf REQUIRED)


# TARGETS ==========================================================================================
add_library(protogen "src/protogen.c")  # Utils for generating .proto files
target_include_directories(protogen PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(protogen
  rosidl_dynamic_typesupport::rosidl_dynamic_typesupport
  ${PROTOBUF_LIBRARIES}
)


# INSTALL AND EXPORT ===============================================================================
install(TARGETS protogen EXPORT protogen-export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_targets(protogen-export HAS_LIBRARY_TARGET)
ament_export_dependencies(rosidl_dynamic_typesupport)
ament_export_dependencies(Protobuf)

ament_package()

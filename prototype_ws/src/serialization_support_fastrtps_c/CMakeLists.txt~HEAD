cmake_minimum_required(VERSION 3.5)
project(serialization_support_fastrtps_c)
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
find_package(fastcdr REQUIRED)
find_package(fastrtps "2.8.0" REQUIRED)


# TARGETS ==========================================================================================
add_library(${PROJECT_NAME}
  "src/dynamic_data.cpp"
  "src/dynamic_type.cpp"

  "src/serialization_support.cpp"
  "src/serialization_impl.cpp"
)
target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${PROJECT_NAME}
  serialization_support_lib::serialization_support_lib
  fastcdr
  fastrtps
)


# INSTALL AND EXPORT ===============================================================================
install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}-export
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_targets(${PROJECT_NAME}-export HAS_LIBRARY_TARGET)
ament_export_dependencies(serialization_support_lib)
ament_export_dependencies(fastcdr)
ament_export_dependencies(fastrtps)

ament_package()

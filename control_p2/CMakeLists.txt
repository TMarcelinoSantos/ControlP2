cmake_minimum_required(VERSION 3.8)
project(control_p2)

set(library_name control_p2)
set(lib_srcs src/control_node.cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(
  include
  ${CMAKE_SOURCE_DIR}/../../lart_common
)

add_library(${library_name} ${lib_srcs})

add_executable(control_node src/control_node.cpp)

ament_target_dependencies(${library_name} rclcpp)

target_link_libraries(control_node ${library_name})

install(TARGETS 
control_node
DESTINATION lib/${PROJECT_NAME}
)

ament_package()

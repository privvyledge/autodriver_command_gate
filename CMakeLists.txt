cmake_minimum_required(VERSION 3.8)
project(command_gate)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

set(DEPS rclcpp rclcpp_components std_msgs std_srvs geometry_msgs ackermann_msgs)

add_library(command_gate_component SHARED src/command_gate_node.cpp)
target_include_directories(command_gate_component PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(command_gate_component ${DEPS})

# Registers plugin + auto-generates standalone executable named command_gate_node
rclcpp_components_register_node(command_gate_component
  PLUGIN "command_gate::CommandGateNode"
  EXECUTABLE command_gate_node)

install(TARGETS command_gate_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

# Python
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS command_gate/command_gate_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME command_gate_node_py)

# Shared resources
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES resource/${PROJECT_NAME}
  DESTINATION share/ament_index/resource_index/packages)

ament_export_include_directories(include)
ament_export_libraries(command_gate_component)
ament_export_dependencies(${DEPS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

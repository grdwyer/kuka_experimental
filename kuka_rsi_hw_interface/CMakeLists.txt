cmake_minimum_required(VERSION 3.5)
project(kuka_rsi_hw_interface)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(angles REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
find_package(tinyxml_vendor REQUIRED)
find_package(TinyXML)

## COMPILE
add_library(
  ${PROJECT_NAME}
  SHARED
  src/kuka_hardware_interface.cpp
)
target_include_directories(
  ${PROJECT_NAME}
  PRIVATE
  include
  ${Boost_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${TinyXML_LIBRARIES}
)

ament_target_dependencies(
  ${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  angles
  tinyxml_vendor
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "KUKA_RSI_HW_INTERFACE_BUILDING_DLL")

# Export hardware pligins
pluginlib_export_plugin_description_file(hardware_interface kuka_rsi_hw_interface.xml)

# INSTALL
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
endif()

## EXPORTS
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)
ament_package()

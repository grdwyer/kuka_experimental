cmake_minimum_required(VERSION 3.5)
project(kuka_rsi_hw_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic -fPIC)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(urdf REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(controller_manager REQUIRED)
find_package(angles REQUIRED)

find_package(Boost REQUIRED COMPONENTS system)
find_package(TinyXML2 REQUIRED)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${TinyXML_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/kuka_hardware_interface.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${TinyXML_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        hardware_interface
        angles
        pluginlib
        rclcpp
        std_srvs
        urdf
        sensor_msgs
        )

pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugin.xml)

# Installation
install(
        TARGETS ${PROJECT_NAME}
        DESTINATION lib
)

install(
        DIRECTORY include/
        DESTINATION include
)

ament_export_include_directories(
        include
)
ament_export_libraries(
        ${PROJECT_NAME}
)

install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}
        )

ament_export_dependencies(
        hardware_interface
        pluginlib
        rclcpp
)
ament_package()

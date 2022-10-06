cmake_minimum_required(VERSION 3.5)

project(kuka_kr60_support)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  meshes
  test
  urdf
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

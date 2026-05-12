cmake_minimum_required(VERSION 3.8)

project(wh044)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes urdf rviz scripts
  DESTINATION share/${PROJECT_NAME}
)

install(FILES package.xml DESTINATION share/${PROJECT_NAME})

ament_package()

cmake_minimum_required(VERSION 3.8)
project(eoat_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/generate_roller_collision_parts.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

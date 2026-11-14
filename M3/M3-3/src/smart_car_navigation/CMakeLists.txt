cmake_minimum_required(VERSION 3.8)
project(smart_car_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config behavior_trees
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

# Install the exact M3-2 map used for this navigation exercise.
install(FILES
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../M3-2/maps/map.yaml
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../M3-2/maps/map.pgm
  DESTINATION share/${PROJECT_NAME}/maps
)

ament_package()

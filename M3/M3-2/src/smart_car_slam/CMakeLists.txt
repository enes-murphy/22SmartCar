cmake_minimum_required(VERSION 3.8)
project(smart_car_slam)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(DIRECTORY ../../worlds ../../config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

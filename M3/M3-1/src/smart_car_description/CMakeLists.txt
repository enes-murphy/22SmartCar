cmake_minimum_required(VERSION 3.8)
project(smart_car_description)

find_package(ament_cmake REQUIRED)

# Install the files ROS 2 and Gazebo will look up by package name.
install(DIRECTORY urdf config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

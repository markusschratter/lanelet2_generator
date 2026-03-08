cmake_minimum_required(VERSION 3.8)
project(lanelet2_generator)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Install Python package (lanelet2_generator)
ament_python_install_package(${PROJECT_NAME})

# Install node executable (only ROS component)
install(PROGRAMS lanelet2_generator_node/route_to_lanelet_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME route_to_lanelet_node
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

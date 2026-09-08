cmake_minimum_required(VERSION 3.8)
project(my_robot)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Python 노드 설치
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  my_robot/sensor_processor.py
  DESTINATION lib/${PROJECT_NAME}
)

# urdf, launch, worlds, config 디렉토리 설치
install(DIRECTORY
  urdf
  launch
  worlds
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

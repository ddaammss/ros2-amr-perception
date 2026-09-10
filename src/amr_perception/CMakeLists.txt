cmake_minimum_required(VERSION 3.8)
project(amr_perception)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Python 노드 설치
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  amr_perception/sensor_processor.py
  amr_perception/lidar_pointcloud_filter.py
  DESTINATION lib/${PROJECT_NAME}
)

# urdf, launch, worlds, rviz 디렉토리 설치
install(DIRECTORY
  urdf
  launch
  worlds
  rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

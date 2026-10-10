cmake_minimum_required(VERSION 3.8)
project(vision)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  srv/DetectionResult.srv
  DEPENDENCIES std_msgs
)

install(PROGRAMS
  scripts/vision_server.py
  scripts/vision_client.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY scripts/ultralytics
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

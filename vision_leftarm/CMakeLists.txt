cmake_minimum_required(VERSION 3.8)
project(vision_leftarm)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  srv/DetectionResult.srv
  DEPENDENCIES std_msgs
)

install(PROGRAMS
  scripts/vision_leftarm_server.py
  scripts/vision_leftarm_client.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/best.pt
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY scripts/ultralytics
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

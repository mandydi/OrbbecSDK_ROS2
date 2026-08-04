cmake_minimum_required(VERSION 3.8)
project(vision_rightarm)

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
  scripts/vision_rightarm_server.py
  scripts/vision_rightarm_client.py
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

install(FILES
  launch/vision_rightarm_launch.py
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_package()

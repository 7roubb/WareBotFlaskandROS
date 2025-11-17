cmake_minimum_required(VERSION 3.5)
project(warebot_robot)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY warebot_robot
  DESTINATION lib/python3.12/site-packages
)

install(
  PROGRAMS
    warebot_robot/mqtt_task_executor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

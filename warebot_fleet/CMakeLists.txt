cmake_minimum_required(VERSION 3.5)
project(warebot_fleet)

find_package(ament_cmake REQUIRED)

# Install Python package (the folder)
install(
  DIRECTORY warebot_fleet/
  DESTINATION lib/python3.12/site-packages/warebot_fleet
)

# Install entrypoint scripts
install(
  PROGRAMS
    warebot_fleet/robot_state_mqtt_bridge.py
    warebot_fleet/map_to_mqtt.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

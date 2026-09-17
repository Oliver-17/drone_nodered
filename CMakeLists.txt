cmake_minimum_required(VERSION 3.8)
project(drone_nodered)

find_package(ament_cmake REQUIRED)

# 無人機專用的部分：drone_api_node 和之後的 Node-RED 流程。
# 通用的 Node-RED ↔ ROS 2 連接放在 nodered_ros2，不放這裡 ——
# 那邊不能依賴 px4_msgs，別人拿去接地面車時才不用裝。
install(DIRECTORY launch gz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/drone_api_node.py
  scripts/start_empty_sitl.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

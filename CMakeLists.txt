cmake_minimum_required(VERSION 3.8)
project(drone_nodered)

find_package(ament_cmake REQUIRED)

# 無人機專用的部分：drone_api_node 和之後的 Node-RED 流程。
# 通用的 Node-RED ↔ ROS 2 連接放在 nodered_ros2，不放這裡 ——
# 那邊不能依賴 px4_msgs，別人拿去接地面車時才不用裝。
# maps：gen_warehouse.py 產生的 Nav2 靜態地圖，nav2.launch.py 的 map:= 要指到這裡
# docs：場景預覽圖，開發時目視確認用
# config：倉庫場景的 Nav2 參數（nav2.launch.py 的 params_file:= 要指到這裡）
install(DIRECTORY launch gz nodered systemd desktop maps docs config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/drone_api_node.py
  scripts/start_empty_sitl.sh
  scripts/move_target.py
  scripts/gen_warehouse.py
  scripts/target_world_node.py
  scripts/install_services.sh
  scripts/uninstall_services.sh
  scripts/console_start.sh
  scripts/console_stop.sh
  scripts/install_flows.sh
  scripts/install_dashboard.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

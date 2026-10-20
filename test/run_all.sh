#!/usr/bin/env bash
# =============================================================================
#  run_all.sh —— 把所有「不需要模擬」的測試跑一遍
#
#  用法：
#      bash ~/ros2_ws/src/drone_nodered/test/run_all.sh
#
#  這裡面沒有一支需要 ROS 在跑、需要 Gazebo、或需要飛 —— 全部是純邏輯，
#  幾秒就跑完。**改完 drone_api_node.py 或 flow 檔之後先跑這個。**
#
#  需要真的飛才驗得到的東西在 test/tools/（run_track_test.sh 之類），
#  那些要先 source test/tools/guard.sh 的 guard_or_die。
# =============================================================================
HERE="$(cd "$(dirname "$0")" && pwd)"
fails=()

run () {
    printf '\n\033[1m=== %s ===\033[0m\n' "$2"
    if "${@:3}"; then :; else fails+=("$2"); fi
}

run x "禁飛方框（t_no_fly.py）"            python3 "$HERE/t_no_fly.py"
run x "站位方向與低通（t_standoff_dir.py）" python3 "$HERE/t_standoff_dir.py"
run x "座標轉換（t_coords.py）"             python3 "$HERE/t_coords.py"
run x "搜尋航點的保險（t_search_waypoint.py）" python3 "$HERE/t_search_waypoint.py"
run x "流程合併保留使用者分頁（t_merge_keep.py）" python3 "$HERE/t_merge_keep.py"
run x "儀表板地圖 widget（t_map_widget.js）" node "$HERE/t_map_widget.js"
run x "launch 參數求值順序（t_launch_order.sh）" bash "$HERE/t_launch_order.sh"

printf '\n'
if [ ${#fails[@]} -gt 0 ]; then
    printf '\033[31m✗ %d 支失敗：\033[0m\n' "${#fails[@]}"
    for f in "${fails[@]}"; do printf '    - %s\n' "$f"; done
    exit 1
fi
printf '\033[32m✓ 全部通過\033[0m\n'

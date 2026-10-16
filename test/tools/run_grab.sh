#!/usr/bin/env bash
# =============================================================================
#  run_grab.sh —— 開模擬、起飛、繞環線、一路存機載影像、收乾淨
#
#  用法：
#      bash test/tools/run_grab.sh [輸出目錄] [機體] [張數]
#      bash test/tools/run_grab.sh ~/yolo_data/frames_car x500_cam_down 160
#
#  為什麼用 api/track 而不是 api/patrol 來繞：
#      track 帶 search_loop 時會自己巡航找目標，而那條環線的其中一段正好
#      沿 x=0 走道通過目標車的位置 —— 所以不管偵測得到與否，車都會進畫面。
#      patrol 的座標是「離起飛點 NED」要自己換算，track 的 search_x/y 直接吃 map 座標。
#
#  ⚠️ 一定要先 guard_or_die：這支會開一整套 PX4+Gazebo+Nav2，
#     如果已經有人在跑，兩套會搶 /MAV1/api/* 和 CPU，量到的東西全部不可信。
# =============================================================================
set -e
OUT="${1:-$HOME/yolo_data/frames}"
MODEL="${2:-x500_cam_down}"
LIMIT="${3:-160}"
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$OUT"
LOG="$OUT/launch.log"      # log 和影像放一起，事後要查哪一幀發生什麼事比較好對

source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
source "$HERE/guard.sh"
guard_or_die || exit 1

cleanup() {
  echo "--- 收尾 ---"
  if [ -n "$PGID" ]; then
    kill -INT -"$PGID" 2>/dev/null || true
    for _ in $(seq 1 25); do kill -0 -"$PGID" 2>/dev/null || break; sleep 1; done
    kill -TERM -"$PGID" 2>/dev/null || true; sleep 2
    kill -KILL -"$PGID" 2>/dev/null || true
  fi
  # MicroXRCEAgent 不在同一個行程群組裡，要另外收
  pkill -f "MicroXRCEAgent udp4 -p 8888" 2>/dev/null || true
  echo "已關閉模擬"
}
trap cleanup EXIT INT TERM

echo "=== 啟動模擬（機體 $MODEL、車靜止、開視覺、關 RViz）==="
setsid ros2 launch drone_nodered warehouse_all.launch.py \
  vision:=true model:="$MODEL" target:=false rviz:=false > "$LOG" 2>&1 &
PGID=$(ps -o pgid= $! | tr -d ' ')
echo "log=$LOG  PGID=$PGID"

python3 "$HERE/wait_ready.py" 200 || { tail -25 "$LOG"; exit 1; }

echo "=== 起飛 3 m ==="
ros2 service call /MAV1/api/takeoff drone_nodered_interfaces/srv/Takeoff \
  "{altitude: 3.0}" | tail -2
sleep 15

echo "=== 繞搜尋環線（map 座標，第三段沿 x=0 走道經過目標車 (0,-6)）==="
ros2 service call /MAV1/api/track drone_nodered_interfaces/srv/Track \
  "{distance: 0.0, altitude: 3.0, duration: 180.0,
    search_x: [5.6, 5.6, 0.0, 0.0], search_y: [-9.0, 9.0, 9.0, -9.0]}" | tail -2

echo "=== 抓圖（每 8 幀一張，上限 $LIMIT）==="
python3 "$HERE/grab_frames.py" "$OUT" 8 "$LIMIT"
echo "=== 抓圖結束 ==="

#!/usr/bin/env bash
# =============================================================================
#  run_target_accuracy.sh —— 量視覺算出來的目標位置準不準
#
#  用法：
#      bash test/tools/run_target_accuracy.sh [記錄秒數]
#
#  車**靜止**（target:=false），飛機飛到看得到它的位置懸停，
#  然後比對 api/target 和車的真實座標。
#
#  為什麼車要靜止：這支量的是**幾何參數對不對**（相機俯角、安裝位姿、內參），
#  車一動就會混進「追蹤反應延遲」那個完全不同的問題。
#
#  ⚠️ 怎麼讓飛機「對著車」—— 2026-10-16 第一版在這裡失敗過：
#     本來想用「先飛到南邊、再往北飛過來」把機頭帶出來（run_hover_grab.sh 的做法），
#     但那一段只有 1 公尺，Nav2 兩秒就判定到達，機頭根本沒轉。
#     而 GoTo 的 yaw 欄位也救不了 —— nav2 的 yaw_goal_tolerance 是 3.14，
#     終點朝向本來就不檢查（GoTo.srv 的註解寫了）。
#
#     改用 api/track 自己的「原地掃描」：看不到目標時飛機會慢慢轉頭找，
#     找到就鎖定並保持距離懸停。那是它本來就有的行為，不用額外做什麼，
#     而且鎖定之後飛機對著車、姿態平穩，正是量幾何精度最好的狀態。
# =============================================================================
set -e
SECS="${1:-30}"
HERE="$(cd "$(dirname "$0")" && pwd)"
OUT="${2:-$HOME/yolo_data/target_accuracy}"
mkdir -p "$OUT"
LOG="$OUT/launch.log"

# 觀測點：車在 (0,-6)，相機俯角 67°、高 3 m 時視野涵蓋前方 0.2~3.8 m，
# 所以停在車南邊 2~3 m 處，轉頭就找得到。朝向交給 api/track 的掃描。
HOVER_Y=-8.5

source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
source "$HERE/guard.sh"
guard_or_die || exit 1

cleanup() {
  echo "--- 收尾 ---"
  pkill -f "move_target.py" 2>/dev/null || true
  if [ -n "$PGID" ]; then
    kill -INT -"$PGID" 2>/dev/null || true
    for _ in $(seq 1 25); do kill -0 -"$PGID" 2>/dev/null || break; sleep 1; done
    kill -TERM -"$PGID" 2>/dev/null || true; sleep 2
    kill -KILL -"$PGID" 2>/dev/null || true
  fi
  pkill -f "MicroXRCEAgent udp4 -p 8888" 2>/dev/null || true
  echo "已關閉模擬"
}
trap cleanup EXIT INT TERM

echo "=== 啟動模擬（車靜止、偵測器依世界檔自動選）==="
setsid ros2 launch drone_nodered warehouse_all.launch.py \
  vision:=true model:=x500_cam_down target:=false rviz:=false headless:=true \
  > "$LOG" 2>&1 &
PGID=$(ps -o pgid= $! | tr -d ' ')
echo "log=$LOG  PGID=$PGID"

python3 "$HERE/wait_ready.py" 200 || { tail -25 "$LOG"; exit 1; }

echo "=== 確認相機參數真的傳下去了 ==="
ros2 param get /target_world camera_z_m || true
ros2 param get /target_world camera_pitch_deg || true
ros2 param get /target_world target_height_m || true

echo "=== 起飛 3 m ==="
ros2 service call /MAV1/api/takeoff drone_nodered_interfaces/srv/Takeoff \
  "{altitude: 3.0}" | tail -2
python3 "$HERE/wait_task.py" takeoff 60 || exit 1

echo "=== 飛到觀測點 (0, $HOVER_Y) ==="
ros2 service call /MAV1/api/goto drone_nodered_interfaces/srv/GoTo \
  "{map_x: 0.0, map_y: $HOVER_Y, yaw: 999.0, timeout: 0.0}" | tail -2
python3 "$HERE/wait_task.py" goto 120 || exit 1
sleep 3

echo "=== 開始追蹤（不帶搜尋環線 → 原地轉頭掃描找車）==="
ros2 service call /MAV1/api/track drone_nodered_interfaces/srv/Track \
  "{distance: 0.0, altitude: 3.0, duration: $((SECS + 90)).0}" | tail -2

echo "=== 等它鎖定並穩住 ==="
for i in $(seq 1 40); do
  ph=$(ros2 topic echo /MAV1/api/status --once --full-length 2>/dev/null \
       | grep -o '"phase": *"[a-z]*"' | head -1 | grep -o '[a-z]*"$' | tr -d '"')
  [ "$ph" = "following" ] && { echo "  ✓ 已鎖定（$i 秒）"; break; }
  sleep 1
done
sleep 8          # 鎖定後還在調整位置，等它停穩

echo "=== 量 $SECS 秒 ==="
python3 "$HERE/measure_target_error.py" "$SECS"

echo "=== 偵測器的狀態 ==="
grep -E "最近 5 秒" "$LOG" | tail -4

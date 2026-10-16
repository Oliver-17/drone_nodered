#!/usr/bin/env bash
# =============================================================================
#  run_hover_grab.sh —— 飛機懸停、車自己經過，一路抓圖
#
#  用法：
#      bash test/tools/run_hover_grab.sh [輸出目錄] [機體] [張數]
#      bash test/tools/run_hover_grab.sh ~/yolo_data/frames_hover x500_cam_down 160
#
#  和 run_grab.sh 的差別，以及為什麼需要兩支：
#      run_grab.sh   飛機繞環線、車靜止 → 整趟只有 6 張看得到車（掠過 1.8 秒）。
#                    那支是用來重現「搜尋時偶然掃到」的情境。
#      這一支        飛機停著、車來回穿過畫面 → 一百多張都看得到車，
#                    而且涵蓋各種距離和車身朝向。要統計偵測率得用這支。
#
#  ---------------------------------------------------------------------------
#  幾何（改任何一個數字之前先看這段）
#  ---------------------------------------------------------------------------
#  相機俯角 67°、飛機高 3 m → 相機離地 2.82 m（掛載在機體下方 0.18 m）。
#  光軸打在正前方 2.82/tan(67°) = 1.20 m，垂直 FOV 54.5° → 視野涵蓋
#  飛機前方 y 約 +0.2 ~ +3.8 m（再遠就超出畫面上緣）。
#
#  所以：飛機停在 (0, -4.5) 朝北   → 看得到 y ∈ [-4.3, -0.7] 這段走道
#        車在 x=0 走道 y -6 ~ -1 來回（aisle 模式從車的生成位置 -6 往北走 span）
#        → 車每趟會完整穿過視野，3.6 m / 0.3 m/s = 12 秒，抓圖 1.9 Hz → 約 23 張
#
#  ⚠️ 朝向是靠「最後一段往北飛」帶出來的，不是指定的 ——
#     nav2 的 yaw_goal_tolerance 是 3.14（GoTo.srv 的註解寫了：終點朝向根本
#     不檢查）。所以先飛到走道南端 (0,-7)，再往北飛到 (0,-4.5)，
#     到達時機頭自然朝北。直接從生成點飛過去的話朝向是隨機的。
#
#  ⚠️ 一定要先 guard_or_die：這支會開一整套 PX4+Gazebo+Nav2。
# =============================================================================
set -e
OUT="${1:-$HOME/yolo_data/frames_hover}"
MODEL="${2:-x500_cam_down}"
LIMIT="${3:-160}"
HERE="$(cd "$(dirname "$0")" && pwd)"
mkdir -p "$OUT"
LOG="$OUT/launch.log"

# 觀測點和車的範圍（見上面的幾何說明）
HOVER_X=0.0
HOVER_Y=-4.5
APPROACH_Y=-7.0      # 先到走道南端，再往北飛過去，機頭才會朝北
CAR_SPEED=0.3
CAR_Y_MIN=-6.0
CAR_Y_MAX=-1.0       # 只用來算 span（= max - min = 5 m），起點是車的生成位置

source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
source "$HERE/guard.sh"
guard_or_die || exit 1

CARPID=""
cleanup() {
  echo "--- 收尾 ---"
  # 車要先收：它是 setsid 起的，不在模擬的行程群組裡，
  # 漏掉的話會留一個程序一直在設 pose（下次測試的車會在奇怪的地方）
  if [ -n "$CARPID" ]; then
    kill -TERM "$CARPID" 2>/dev/null || true
    sleep 1
  fi
  pkill -f "move_target.py" 2>/dev/null || true
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

echo "=== 啟動模擬（機體 $MODEL、車先不動、開視覺、關 RViz）==="
# target:=false —— 車由這支腳本自己控制範圍和速度，不要讓 launch 用預設的 loop 模式去推
setsid ros2 launch drone_nodered warehouse_all.launch.py \
  vision:=true model:="$MODEL" target:=false rviz:=false > "$LOG" 2>&1 &
PGID=$(ps -o pgid= $! | tr -d ' ')
echo "log=$LOG  PGID=$PGID"

python3 "$HERE/wait_ready.py" 200 || { tail -25 "$LOG"; exit 1; }

echo "=== 起飛 3 m ==="
ros2 service call /MAV1/api/takeoff drone_nodered_interfaces/srv/Takeoff \
  "{altitude: 3.0}" | tail -2
python3 "$HERE/wait_task.py" takeoff 60 || exit 1

echo "=== 飛到走道南端 (0, $APPROACH_Y) ==="
ros2 service call /MAV1/api/goto drone_nodered_interfaces/srv/GoTo \
  "{map_x: $HOVER_X, map_y: $APPROACH_Y, yaw: 999.0, timeout: 0.0}" | tail -2
python3 "$HERE/wait_task.py" goto 120 || exit 1

echo "=== 往北飛到觀測點 (0, $HOVER_Y)，機頭會朝北 ==="
ros2 service call /MAV1/api/goto drone_nodered_interfaces/srv/GoTo \
  "{map_x: $HOVER_X, map_y: $HOVER_Y, yaw: 999.0, timeout: 0.0}" | tail -2
python3 "$HERE/wait_task.py" goto 90 || exit 1
sleep 3            # 讓它穩下來，剛到位時還在修正

echo "=== 讓車在走道來回（速度 $CAR_SPEED m/s）==="
setsid python3 "$HOME/ros2_ws/src/drone_nodered/scripts/move_target.py" \
  --mode aisle --model hatchback_target --world warehouse \
  --speed "$CAR_SPEED" --y-min "$CAR_Y_MIN" --y-max "$CAR_Y_MAX" \
  > "$OUT/car.log" 2>&1 &
CARPID=$!
sleep 3
if ! kill -0 "$CARPID" 2>/dev/null; then
  echo "✗ 車沒有跑起來："; cat "$OUT/car.log"; exit 1
fi
head -3 "$OUT/car.log"

echo "=== 抓圖（每 8 幀一張，上限 $LIMIT）==="
python3 "$HERE/grab_frames.py" "$OUT" 8 "$LIMIT"
echo "=== 抓圖結束 ==="

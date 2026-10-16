#!/usr/bin/env bash
# =============================================================================
#  run_track_test.sh —— 端到端：真車版世界，車在跑、飛機用 YOLO 追
#
#  用法：
#      bash test/tools/run_track_test.sh [記錄秒數]
#
#  這支驗的是「偵測得到」之外的那一半 —— 追不追得動。
#  偵測本身已經分開驗過了（test/tools/check_yolo_node.py，不用飛）。
#
#  ⚠️ 世界必須是真車版。切換方式：
#        cd scripts && python3 gen_warehouse.py --target hatchback   # 真車
#        cd scripts && python3 gen_warehouse.py                      # 洋紅（預設）
#        cd ~/ros2_ws && colcon build --packages-select drone_nodered
#     世界檔名不變（WORLD 這個名字同時綁著 maps/ 和 config/ 兩個檔），
#     所以切換靠重跑產生器，不是換參數。
#
#  ⚠️ 一定要先 guard_or_die。
# =============================================================================
set -e
SECS="${1:-120}"
HERE="$(cd "$(dirname "$0")" && pwd)"
OUT="${2:-$HOME/yolo_data/track_test}"
mkdir -p "$OUT"
LOG="$OUT/launch.log"

source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"
source "$HERE/guard.sh"
guard_or_die || exit 1

# 世界得真的是真車版，不然 YOLO 對著洋紅方盒跑，結果會很難解讀
W="$HOME/ros2_ws/install/drone_nodered/share/drone_nodered/gz/worlds/warehouse.sdf"
if ! grep -q "hatchback_target" "$W"; then
  echo "✗ 目前的世界不是真車版（$W 裡沒有 hatchback_target）"
  echo "  切換：cd scripts && python3 gen_warehouse.py --target hatchback"
  echo "        cd ~/ros2_ws && colcon build --packages-select drone_nodered"
  exit 1
fi

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

echo "=== 啟動模擬（真車版、YOLO 偵測、車會跑）==="
setsid ros2 launch drone_nodered warehouse_all.launch.py \
  vision:=true detector:=yolo model:=x500_cam_down target:=true rviz:=false \
  > "$LOG" 2>&1 &
PGID=$(ps -o pgid= $! | tr -d ' ')
echo "log=$LOG  PGID=$PGID"

python3 "$HERE/wait_ready.py" 200 || { tail -25 "$LOG"; exit 1; }

# YOLO 節點要載權重 + 暖機（實測 1.4 秒），等它說話了再起飛
echo "=== 等 YOLO 節點就緒 ==="
for _ in $(seq 1 40); do
  grep -q "YOLO 偵測啟動" "$LOG" && break
  sleep 1
done
grep -q "YOLO 偵測啟動" "$LOG" \
  && echo "  ✓ $(grep -m1 '權重載入' "$LOG" | sed 's/.*]: //')" \
  || { echo "✗ YOLO 節點沒起來："; grep -iE "yolo|error" "$LOG" | tail -10; exit 1; }

echo "=== 起飛 3 m ==="
ros2 service call /MAV1/api/takeoff drone_nodered_interfaces/srv/Takeoff \
  "{altitude: 3.0}" | tail -2
python3 "$HERE/wait_task.py" takeoff 60 || exit 1

echo "=== 開始追蹤（distance=0 → 節點自己算最佳距離）==="
# search_x/y 是搜尋環線（map 座標），和車繞的環線同一條但方向相反 ——
# 反向相遇最快。這組數字來自 config/warehouse_geometry.yaml 的 search_loop。
ros2 service call /MAV1/api/track drone_nodered_interfaces/srv/Track \
  "{distance: 0.0, altitude: 3.0, duration: $((SECS + 60)).0,
    search_x: [5.6, 5.6, 0.0, 0.0], search_y: [-9.0, 9.0, 9.0, -9.0]}" | tail -2

echo "=== 記錄 $SECS 秒 ==="
# 同時把 api/status 的追蹤階段存起來，事後對得出「什麼時候鎖定的」
setsid ros2 topic echo /MAV1/api/status --full-length > "$OUT/status.log" 2>&1 &
ECHO_PID=$!
python3 "$HERE/watch_chase.py" "$SECS" "$OUT/chase.csv"
kill "$ECHO_PID" 2>/dev/null || true

echo "=== YOLO 節點的偵測率 ==="
grep "最近 5 秒" "$LOG" | tail -8

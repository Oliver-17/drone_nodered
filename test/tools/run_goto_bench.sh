#!/usr/bin/env bash
# =============================================================================
#  run_goto_bench.sh —— 開倉庫堆疊 → 量一趟 api/goto → 收乾淨
#
#  用來比較「改 Nav2 速度參數前後」。參數是 launch 時讀進去的，
#  所以每量一次都要重開整套（不是改個數字就好）。
#
#  ⚠️ 不要加 set -u：ROS 的 setup.bash 在 set -u 下會炸（未定義變數一堆）。
#  ⚠️ 收尾一定要對**行程群組**下訊號（kill -INT -$PGID）：
#     ros2 launch 沒有控制終端時不理訊號，單獨 kill 它只會讓底下的節點
#     變孤兒，下一輪的 guard 就會擋住。所以用 setsid 開。
# =============================================================================
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LABEL="${1:-無標籤}"
OUT="${2:-/tmp/goto_bench}"
mkdir -p "$OUT"
LOG="$OUT/launch_$(date +%H%M%S).log"

source "$HERE/guard.sh"
guard_or_die || exit 1

source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

echo "=== 開倉庫堆疊（headless，log：$LOG）==="
setsid ros2 launch drone_nodered warehouse_all.launch.py \
    headless:=true rviz:=false target:=false vision:=false \
    > "$LOG" 2>&1 &
LAUNCH_PID=$!
PGID="$(ps -o pgid= -p $LAUNCH_PID | tr -d ' ')"

cleanup() {
    echo "=== 收尾（對行程群組 $PGID 送 INT）==="
    kill -INT -"$PGID" 2>/dev/null || true
    for _ in $(seq 1 20); do
        kill -0 -"$PGID" 2>/dev/null || break
        sleep 1
    done
    kill -0 -"$PGID" 2>/dev/null && { echo "  還沒關，送 TERM"; kill -TERM -"$PGID"; sleep 3; }
    echo "  剩下的："
    pgrep -af 'px4_sitl_default/bin|MicroXRCEAgent|drone_api_node|bt_navigator' \
        | grep -v shell-snapshots | grep -v 'pgrep -af' | sed 's/^/    /' || echo "    （沒有）✓"
}
trap cleanup EXIT

# ⚠️ 等「堆疊就緒」那行**不等於** Nav2 可用（它是 launch 自己印的）。
#    要等的是 Nav2 生命週期真的 active。
echo "=== 等 Nav2 的 Managed nodes are active（最多 180 秒）==="
for i in $(seq 1 180); do
    if grep -q "Managed nodes are active" "$LOG" 2>/dev/null; then
        echo "  好了（第 ${i} 秒）"
        break
    fi
    if [ "$i" = 180 ]; then
        echo "✗ Nav2 沒就緒（180 秒）。最可能是這個已知的雷："
        echo "   bringup 進行中有新節點加入 → lifecycle 卡在 Configuring，"
        echo "   log 裡會看到 rmw_response.cpp:154「client will not receive response」。"
        echo "   這裡的新節點就是 45 秒才起來的 drone_api_node —— Nav2 慢一點就會撞上。"
        echo "   處置：直接重跑一次（時序問題，不是參數錯）。"
        grep -n "will not receive response\|Configuring" "$LOG" | tail -5
        exit 3
    fi
    sleep 1
done
sleep 5     # 讓 drone_api_node 的訂閱都接上

python3 "$HERE/measure_goto.py" --label "$LABEL" "${@:3}" | tee "$OUT/result_$(echo "$LABEL" | tr ' /' '__').txt"

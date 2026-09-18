#!/usr/bin/env bash
# =============================================================================
#  console_stop.sh —— 桌面圖示「無人機控制台（停）」按下去執行的東西
#
#  停底座就夠了（drone-sim 是 PartOf=drone-nodered，會跟著停），但這裡還是
#  明確停一次模擬 —— 因為模擬可能是你自己單獨 start 的，那時底座根本沒在跑，
#  只停底座不會動到它。
#
#  最後會檢查有沒有殘留再回報，讓你點完就知道電腦真的乾淨了。
# =============================================================================
set -u

notify() { command -v notify-send >/dev/null 2>&1 && notify-send -a "無人機控制台" "無人機控制台" "$1" || true; }

systemctl --user stop drone-sim.service 2>/dev/null || true
systemctl --user stop drone-nodered.service 2>/dev/null || true

# 給 systemd 一點時間收完整組程序（Gazebo 關閉要幾秒）
for _ in $(seq 1 15); do
    if ! systemctl --user is-active --quiet drone-nodered.service \
       && ! systemctl --user is-active --quiet drone-sim.service; then
        break
    fi
    sleep 1
done

# 用程序名稱確認，不是只看服務狀態 —— 服務說停了但程序還在，才是我們真正怕的情況
left="$(ps -eo comm --no-headers | awk '$1=="px4"||$1=="ruby"||$1=="MicroXRCEAgent"||$1=="node-red"' | sort -u | tr '\n' ' ')"
if [ -n "${left// /}" ]; then
    notify "已送出停止，但還看到： $left（可能是你自己另外開的）"
else
    notify "已全部關閉，沒有殘留"
fi

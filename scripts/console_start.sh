#!/usr/bin/env bash
# =============================================================================
#  console_start.sh —— 桌面圖示「無人機控制台（開）」按下去執行的東西
#
#  做三件事：開底座服務 → 等 Node-RED 真的開始聽 → 開瀏覽器。
#
#  為什麼要「等」：systemctl start 一回來只代表程序啟動了，不代表 Node-RED 已經
#  可以接受連線。太早開瀏覽器會看到「無法連線」，使用者會以為壞了。
#
#  為什麼失敗要跳視窗：圖示是用點的，沒有終端機可以看錯誤訊息。
#  出錯就用 zenity 把 journal 的最後幾行顯示出來，不用開終端機也查得到原因。
# =============================================================================
set -u

# 可以用環境變數覆寫：換成別的埠、或拿測試用的服務來驗證這支腳本，
# 都不用改檔案（正常點圖示時用的就是下面這兩個預設值）
SERVICE="${DRONE_CONSOLE_SERVICE:-drone-nodered.service}"
PORT="${DRONE_CONSOLE_PORT:-1880}"
URL="http://localhost:$PORT"
WAIT_SEC=60

notify() { command -v notify-send >/dev/null 2>&1 && notify-send -a "無人機控制台" "無人機控制台" "$1" || true; }

die() {
    local msg="$1"
    local log
    log="$(journalctl --user -u "$SERVICE" -n 20 --no-pager 2>/dev/null)"
    if command -v zenity >/dev/null 2>&1; then
        zenity --error --width=700 --title="無人機控制台啟動失敗" \
               --text="$msg

最後的 log：
$log" 2>/dev/null
    else
        echo "$msg"; echo "$log"
    fi
    exit 1
}

port_open() { ss -ltn 2>/dev/null | grep -q ":$PORT "; }

if systemctl --user is-active --quiet "$SERVICE"; then
    notify "已經在跑了，直接開啟畫面"
else
    systemctl --user start "$SERVICE" || die "systemctl start $SERVICE 失敗。"
    notify "啟動中…"
fi

waited=0
while ! port_open; do
    sleep 1
    waited=$((waited + 1))
    # 服務中途掛掉就不用再等了，直接把原因秀出來
    if ! systemctl --user is-active --quiet "$SERVICE"; then
        die "服務啟動後又停掉了（等了 ${waited} 秒）。"
    fi
    if [ "$waited" -ge "$WAIT_SEC" ]; then
        die "等了 ${WAIT_SEC} 秒，Node-RED 還沒開始聽 $PORT 埠。"
    fi
done

xdg-open "$URL" >/dev/null 2>&1 &
notify "已開啟 $URL（${waited} 秒）"

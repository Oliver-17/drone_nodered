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
# 要開哪些頁：空字串＝編輯器、/dashboard＝儀表板。
# 用逗號分隔可以一次開多個，順序＝分頁順序（最後開的那個通常會在前景）。
# ⚠️ 這裡用 ${VAR-預設} 而不是 ${VAR:-預設}，少一個冒號差很多：
#      ${VAR:-預設}  變數「沒設定 或 是空字串」都用預設
#      ${VAR-預設}   只有「沒設定」才用預設，空字串會被尊重
#    我們的「編輯器」剛好就是用空字串表示，所以不能用 :- ，
#    否則 DRONE_CONSOLE_PATH="" 會被當成沒設，又把儀表板一起打開。
PATHS="${DRONE_CONSOLE_PATH-/dashboard,/}"
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

IFS=',' read -ra PAGES <<< "$PATHS"
opened=""
for p in "${PAGES[@]}"; do
    xdg-open "http://localhost:$PORT$p" >/dev/null 2>&1 &
    opened="$opened localhost:$PORT$p"
    # 間隔一下：瀏覽器同時收到兩個 xdg-open 有時只會開其中一個
    sleep 1
done
notify "已開啟${opened}（${waited} 秒）"

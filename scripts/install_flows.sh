#!/usr/bin/env bash
# =============================================================================
#  install_flows.sh —— 把套件裡的 Node-RED 流程裝進 Node-RED（第一次安裝／之後更新都用這支）
#
#  用法：
#      ~/ros2_ws/src/drone_nodered/scripts/install_flows.sh
#      ~/ros2_ws/src/drone_nodered/scripts/install_flows.sh --dry-run          # 只看會做什麼
#      ~/ros2_ws/src/drone_nodered/scripts/install_flows.sh --user-dir /tmp/nr # 裝到別的地方
#
#  為什麼要有這支，不在編輯器裡「匯入」就好：
#      匯入只適合「第一次」。之後版本更新再匯入，Node-RED 會發現節點 id 已經存在，
#      問你要取代還是匯入副本 —— 選錯就變成兩份重複的流程和積木，
#      而且要在編輯器裡一個一個刪，非常痛苦（2026-09-18 實際踩到）。
#      直接換掉流程檔就沒有這個問題，而且別人拿到這個專案也能一行指令搞定。
#
#  ⚠️ 這支會覆蓋 Node-RED 的 flows.json。覆蓋前一定先備份成 flows.json.bak.<日期時間>，
#     所以你在編輯器裡改過的東西不會消失，只是要自己去備份檔撈回來。
# =============================================================================
set -e

SERVICE="drone-nodered.service"
PKG_SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
USER_DIR="${NODE_RED_USER_DIR:-$HOME/.node-red}"
DRY_RUN=0

while [ $# -gt 0 ]; do
    case "$1" in
        --dry-run) DRY_RUN=1; shift ;;
        --user-dir) USER_DIR="$2"; shift 2 ;;
        *) echo "不認得的參數：$1"; exit 1 ;;
    esac
done

# 流程檔：從原始碼目錄執行時在 <pkg>/nodered/flows；從安裝目錄執行時問 ROS 2 要路徑
SRC="$PKG_SRC/nodered/flows/drone_basic.json"
if [ ! -f "$SRC" ]; then
    SRC="$(ros2 pkg prefix drone_nodered 2>/dev/null)/share/drone_nodered/nodered/flows/drone_basic.json"
fi
if [ ! -f "$SRC" ]; then
    echo "✗ 找不到 drone_basic.json（試過 $PKG_SRC/nodered/flows 和安裝目錄）"
    exit 1
fi

DST="$USER_DIR/flows.json"
echo "來源： $SRC"
echo "目標： $DST"

# 內容一樣就不用做事 —— 免得白白多一個備份檔，也免得白重啟服務
if [ -f "$DST" ] && cmp -s "$SRC" "$DST"; then
    echo "✓ 內容已經一樣，不需要更新"
    exit 0
fi

# Node-RED 關閉時會把記憶體裡的流程寫回 flows.json，
# 所以「開著的時候改檔案」會被它蓋回去。一定要先停。
WAS_ACTIVE=0
if systemctl --user is-active --quiet "$SERVICE"; then
    WAS_ACTIVE=1
    echo "控制台正在跑 → 會先停掉、換檔案、再開回來"
fi

if [ "$DRY_RUN" = 1 ]; then
    echo "[dry-run] 會備份 $DST → $DST.bak.<日期時間>"
    echo "[dry-run] 會複製流程檔過去"
    [ "$WAS_ACTIVE" = 1 ] && echo "[dry-run] 會重新啟動 $SERVICE"
    echo "（dry-run，沒有實際更動）"
    exit 0
fi

if [ "$WAS_ACTIVE" = 1 ]; then
    systemctl --user stop "$SERVICE"
    # 等它真的寫完檔再動手
    sleep 2
fi

mkdir -p "$USER_DIR"
if [ -f "$DST" ]; then
    BAK="$DST.bak.$(date +%Y%m%d-%H%M%S)"
    cp -p "$DST" "$BAK"
    echo "  ✓ 舊的流程已備份： $BAK"
fi
cp "$SRC" "$DST"
echo "  ✓ 已更新 $DST"

if [ "$WAS_ACTIVE" = 1 ]; then
    systemctl --user start "$SERVICE"
    echo "  ✓ 控制台已重新啟動"
fi

echo
echo "完成。瀏覽器重新整理 http://localhost:1880 就會看到新的流程。"
echo "（編輯器裡如果還開著舊畫面，按 F5 重新整理才看得到）"

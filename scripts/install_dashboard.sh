#!/usr/bin/env bash
# =============================================================================
#  install_dashboard.sh —— 裝 Node-RED 儀表板（Dashboard 2.0）
#
#  用法：
#      ~/ros2_ws/src/drone_nodered/scripts/install_dashboard.sh
#      ~/ros2_ws/src/drone_nodered/scripts/install_dashboard.sh --dry-run
#      ~/ros2_ws/src/drone_nodered/scripts/install_dashboard.sh --user-dir /tmp/nr
#
#  裝完之後：
#      編輯器  http://localhost:1880            ← 你組流程的地方
#      儀表板  http://localhost:1880/dashboard  ← 給操作的人用的畫面
#
#  這是這個專案**唯一**需要的第三方 Node-RED 套件。裝在 Node-RED 的目錄
#  （~/.node-red/node_modules），不是系統目錄，所以不影響別的東西。
#
#  ⚠️ 兩個要知道的事：
#    1. 套件名稱是 @flowfuse/node-red-dashboard（Dashboard 2.0）。
#       網路上很多教學寫的是舊的 node-red-dashboard，**已經停止維護**，不要裝那個。
#    2. 儀表板網址**不需要登入**就能打開。編輯器的帳號密碼管不到它。
#       要加密碼得改 ~/.node-red/settings.js 的 httpNodeAuth（這支腳本不會動它）。
# =============================================================================
set -e

PKG="@flowfuse/node-red-dashboard"
SERVICE="drone-nodered.service"
USER_DIR="${NODE_RED_USER_DIR:-$HOME/.node-red}"
DRY_RUN=0

while [ $# -gt 0 ]; do
    case "$1" in
        --dry-run) DRY_RUN=1; shift ;;
        --user-dir) USER_DIR="$2"; shift 2 ;;
        *) echo "不認得的參數：$1"; exit 1 ;;
    esac
done

command -v npm >/dev/null || { echo "✗ 找不到 npm（Node.js 沒裝好？）"; exit 1; }
echo "Node-RED 目錄： $USER_DIR"

if [ -d "$USER_DIR/node_modules/$PKG" ]; then
    VER="$(node -p "require('$USER_DIR/node_modules/$PKG/package.json').version" 2>/dev/null || echo "?")"
    echo "✓ 已經裝過了（版本 $VER）"
    exit 0
fi

if [ "$DRY_RUN" = 1 ]; then
    echo "[dry-run] 會在 $USER_DIR 執行： npm install $PKG"
    systemctl --user is-active --quiet "$SERVICE" && echo "[dry-run] 裝完會重新啟動 $SERVICE"
    echo "（dry-run，沒有實際安裝）"
    exit 0
fi

mkdir -p "$USER_DIR"
cd "$USER_DIR"
echo "安裝中…（第一次要下載，可能要一兩分鐘）"
npm install "$PKG"

# Node-RED 只在啟動時掃描節點，裝完一定要重啟才看得到儀表板的積木
if systemctl --user is-active --quiet "$SERVICE"; then
    systemctl --user restart "$SERVICE"
    echo "  ✓ 控制台已重新啟動"
else
    echo "  ⚠ 控制台沒在跑。下次啟動才會載入儀表板"
fi

echo
echo "裝好了。接下來："
echo "  1. 更新流程： ~/ros2_ws/src/drone_nodered/scripts/install_flows.sh"
echo "  2. 打開儀表板： http://localhost:1880/dashboard"

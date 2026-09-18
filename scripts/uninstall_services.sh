#!/usr/bin/env bash
# =============================================================================
#  uninstall_services.sh —— 移除 install_services.sh 裝的兩個服務
#
#  用法：
#      ~/ros2_ws/src/drone_nodered/scripts/uninstall_services.sh
#      ~/ros2_ws/src/drone_nodered/scripts/uninstall_services.sh --dry-run
#
#  做的事：先停掉服務 → 取消自動啟動（如果有設）→ 刪掉服務檔 → daemon-reload。
#
#  安全規則：只刪「第一行有本專案標記」的檔案。
#  同名但不是我們產生的檔案一律不動，因為那可能是你自己寫的。
# =============================================================================
set -e

MARKER="# generated-by: drone_nodered/scripts/install_services.sh"
UNIT_DIR="$HOME/.config/systemd/user"

DRY_RUN=0
[ "${1:-}" = "--dry-run" ] && DRY_RUN=1

remove_one() {
    local name="$1"
    local dst="$UNIT_DIR/$name"

    if [ ! -e "$dst" ]; then
        echo "  － $name 本來就不在，跳過"
        return 0
    fi
    if ! head -1 "$dst" | grep -qF "$MARKER"; then
        echo "  ✗ $dst 不是這個專案產生的 —— 不刪，請自己確認。"
        return 0
    fi

    if [ "$DRY_RUN" = 1 ]; then
        echo "  [dry-run] 會停掉並刪除 $dst"
        return 0
    fi

    # 先停再刪：檔案刪掉後 systemctl 就找不到這個單元，會變成停不掉的殘留程序
    systemctl --user stop "$name" 2>/dev/null || true
    # 沒 enable 過的話這行不會做事，加著是為了你哪天自己 enable 了也能乾淨移除
    systemctl --user disable "$name" 2>/dev/null || true
    rm -f "$dst"
    echo "  ✓ 已停止並刪除 $dst"
}

remove_desktop() {
    local path="$1"
    if [ ! -e "$path" ]; then
        echo "  － $path 本來就不在，跳過"
        return 0
    fi
    # .desktop 的標記在檔尾，所以整份檔案找
    if ! grep -qF "$MARKER" "$path"; then
        echo "  ✗ $path 不是這個專案產生的 —— 不刪。"
        return 0
    fi
    if [ "$DRY_RUN" = 1 ]; then
        echo "  [dry-run] 會刪除 $path"
        return 0
    fi
    rm -f "$path"
    echo "  ✓ 已刪除 $path"
}

echo "移除服務…（只動有本專案標記的檔案）"
# 先移除模擬：它 PartOf 底座，底座先停的話模擬會被連帶停掉，順序反了不影響結果，
# 但這樣訊息比較好讀
remove_one drone-sim.service
remove_one drone-nodered.service

echo "移除桌面圖示…"
APPS_DIR="$HOME/.local/share/applications"
DESKTOP_DIR="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")"
for d in drone-nodered-start.desktop drone-nodered-stop.desktop; do
    remove_desktop "$APPS_DIR/$d"
    remove_desktop "$DESKTOP_DIR/$d"
done

if [ "$DRY_RUN" = 1 ]; then
    echo; echo "（dry-run，沒有實際更動）"; exit 0
fi

systemctl --user daemon-reload
echo "  ✓ systemctl --user daemon-reload"
echo
echo "移除完成。~/.config/systemd/user/ 內其他檔案沒有被動過。"

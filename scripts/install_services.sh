#!/usr/bin/env bash
# =============================================================================
#  install_services.sh —— 把兩個 systemd 使用者服務裝到 ~/.config/systemd/user/
#
#  用法：
#      ~/ros2_ws/src/drone_nodered/scripts/install_services.sh
#      ~/ros2_ws/src/drone_nodered/scripts/install_services.sh --dry-run   # 只看會做什麼
#
#  裝好之後：
#      systemctl --user start drone-nodered     # 開底座
#      systemctl --user start drone-sim         # 開模擬
#      systemctl --user status drone-nodered    # 看狀態
#      journalctl --user -u drone-sim -f        # 看 log
#
#  ⚠️ 這支腳本「不會」enable 服務 —— 登入時不會自動啟動，要自己點才開。
#     要移除請用 uninstall_services.sh。
#
#  安全規則：只碰自己產生的檔案。目標位置已經有同名檔案而且不是這支腳本寫的，
#  就停下來不覆蓋（那可能是你自己或別的套件放的）。
# =============================================================================
set -e

# 這行會寫進產生出來的檔案第一行，當作「這是我們產生的」的憑證。
# 解除安裝時也靠它判斷能不能刪。
MARKER="# generated-by: drone_nodered/scripts/install_services.sh"

DRY_RUN=0
[ "${1:-}" = "--dry-run" ] && DRY_RUN=1

PKG_SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# 工作空間 = src 的上一層（<ws>/src/drone_nodered/scripts → <ws>）
WORKSPACE="$(cd "$PKG_SRC/../.." && pwd)"
ROS_SETUP="/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
UNIT_DIR="$HOME/.config/systemd/user"

echo "套件原始碼： $PKG_SRC"
echo "工作空間：   $WORKSPACE"
echo "ROS 環境：   $ROS_SETUP"
echo "安裝到：     $UNIT_DIR"
echo

# --- 先檢查前提，錯的路徑寫進服務檔會變成很難查的問題 ---------------------------
fail=0
if [ ! -f "$ROS_SETUP" ]; then
    echo "✗ 找不到 $ROS_SETUP（ROS 2 沒裝，或 ROS_DISTRO 不是 ${ROS_DISTRO:-humble}）"; fail=1
fi
if [ ! -f "$WORKSPACE/install/setup.bash" ]; then
    echo "✗ 找不到 $WORKSPACE/install/setup.bash —— 請先在工作空間執行 colcon build"; fail=1
fi
if [ ! -f "$PKG_SRC/systemd/drone-nodered.service.in" ]; then
    echo "✗ 找不到服務範本（$PKG_SRC/systemd/）"; fail=1
fi
[ "$fail" = 1 ] && exit 1
echo "✓ 前提檢查通過"

mkdir -p "$UNIT_DIR"

install_one() {
    local name="$1"
    local src="$PKG_SRC/systemd/$name.in"
    local dst="$UNIT_DIR/$name"

    # 安全檢查：已存在而且不是我們寫的 → 不覆蓋
    if [ -e "$dst" ] && ! head -1 "$dst" | grep -qF "$MARKER"; then
        echo "✗ $dst 已存在，而且不是這支腳本產生的 —— 為了安全不覆蓋。"
        echo "  確認過內容不要了的話，自己刪掉再跑一次。"
        return 1
    fi

    local action="新增"
    [ -e "$dst" ] && action="覆蓋（舊的也是本腳本產生的）"

    if [ "$DRY_RUN" = 1 ]; then
        echo "  [dry-run] 會$action $dst"
        return 0
    fi

    { echo "$MARKER"
      sed -e "s|@WORKSPACE@|$WORKSPACE|g" \
          -e "s|@ROS_SETUP@|$ROS_SETUP|g" \
          -e "s|@PKG_SRC@|$PKG_SRC|g" "$src"
    } > "$dst"
    echo "  ✓ $action $dst"
}

install_desktop() {
    local name="$1" dst_dir="$2" mark_trusted="$3"
    local src="$PKG_SRC/desktop/$name.in"
    local dst="$dst_dir/$name"

    # 安全檢查同上。差別是標記放在檔尾 —— .desktop 的第一行必須是 [Desktop Entry]
    if [ -e "$dst" ] && ! grep -qF "$MARKER" "$dst"; then
        echo "✗ $dst 已存在且不是本腳本產生的 —— 不覆蓋。"
        return 1
    fi

    if [ "$DRY_RUN" = 1 ]; then
        echo "  [dry-run] 會寫入 $dst"
        return 0
    fi

    mkdir -p "$dst_dir"
    { sed -e "s|@PKG_SRC@|$PKG_SRC|g" "$src"; echo "$MARKER"; } > "$dst"
    # 放在桌面的 .desktop 要可執行，GNOME 才願意把它當程式啟動
    chmod +x "$dst"
    # 而且要標成「信任」，否則點下去只會用文字編輯器打開它
    if [ "$mark_trusted" = 1 ] && command -v gio >/dev/null 2>&1; then
        gio set "$dst" metadata::trusted true 2>/dev/null || true
    fi
    echo "  ✓ 寫入 $dst"
}

echo
echo "安裝服務檔…"
install_one drone-nodered.service
install_one drone-sim.service

echo
echo "安裝桌面圖示…"
APPS_DIR="$HOME/.local/share/applications"
DESKTOP_DIR="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")"
for d in drone-nodered-start.desktop drone-nodered-stop.desktop; do
    # 應用程式目錄：會出現在「顯示應用程式」選單裡（不需要 trusted）
    install_desktop "$d" "$APPS_DIR" 0
    # 桌面：點兩下就能開
    [ -d "$DESKTOP_DIR" ] && install_desktop "$d" "$DESKTOP_DIR" 1
done

if [ "$DRY_RUN" = 1 ]; then
    echo; echo "（dry-run，沒有實際寫入任何東西）"; exit 0
fi

# 讓 systemd 重讀設定，新檔案才看得到
systemctl --user daemon-reload
echo "  ✓ systemctl --user daemon-reload"

echo
echo "=================================================="
echo " 裝好了。兩個服務都「沒有」設成自動啟動。"
echo "=================================================="
echo "開底座： systemctl --user start drone-nodered"
echo "開模擬： systemctl --user start drone-sim"
echo "全關：   systemctl --user stop drone-nodered   （模擬會跟著停）"
echo "看狀態： systemctl --user status drone-nodered"
echo "看 log： journalctl --user -u drone-sim -f"
echo "移除：   $PKG_SRC/scripts/uninstall_services.sh"

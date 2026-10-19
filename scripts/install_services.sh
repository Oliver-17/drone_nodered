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
# 空白世界（drone-sim.service）2026-10-12 退休 —— Oliver 說舊地圖不用了。
# 東西都還留著（systemd/drone-sim.service.in、gz/worlds/empty*.sdf），
# 帶 --with-empty-sim 就裝回來。
WITH_EMPTY_SIM=0
while [ $# -gt 0 ]; do
    case "$1" in
        --dry-run)        DRY_RUN=1 ;;
        --with-empty-sim) WITH_EMPTY_SIM=1 ;;
        -h|--help)
            echo "用法： $0 [--dry-run] [--with-empty-sim]"
            echo "  --dry-run         只印會做什麼，不寫任何檔"
            echo "  --with-empty-sim  連舊的空白世界服務（drone-sim）一起裝"
            exit 0 ;;
        *) echo "✗ 不認得的參數：$1（--help 看用法）"; exit 1 ;;
    esac
    shift
done

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

# --- 場景參數（2026-10-08）-------------------------------------------------
# drone_api_node 需要知道兩件和場景綁定的事：
#     local_origin_map   PX4 本地原點（＝生成點）在 map 的座標
#     camera_pitch_deg   相機俯角，算 api/track 的「最佳跟隨距離」用
#
# 為什麼在安裝時填進服務檔、而不是讓節點自己讀幾何檔：
#   drone_api_node 刻意不知道地圖的存在（它只和 PX4 說話），這樣換場地、
#   甚至換成別的載具都不用改它。場景知識由啟動的人注入。
#
# 為什麼一定要和 px4_bridge 的 odom_origin 一致：
#   兩者描述同一件事。不一致的話 api/track 的搜尋航點會整組平移，
#   **而且不會報錯**。warehouse_all.launch.py 內部是共用同一個變數，
#   但底座的 drone_api 是另一個服務，只能靠這裡填對。
GEOM="$PKG_SRC/config/warehouse_geometry.yaml"
LOCAL_ORIGIN=""
if [ -f "$GEOM" ]; then
    LOCAL_ORIGIN="$(sed -n 's/^spawn: *\[ *\([-0-9.]*\) *, *\([-0-9.]*\) *\].*/\1,\2/p' "$GEOM")"
fi
if [ -z "$LOCAL_ORIGIN" ]; then
    echo "⚠ 讀不到 $GEOM 的 spawn —— local_origin_map 會留空。"
    echo "  後果：api/track 帶搜尋環線時會被拒絕（節點算不出 map→NED）。"
    echo "  解法：python3 $PKG_SRC/scripts/gen_warehouse.py 之後重跑這支。"
fi
# 相機俯角要和 x500_cam_down 的模型一致（vision.launch.py 也是依機體選 67）
CAMERA_PITCH="67.0"

# --- 跟隨段的禁飛方框（2026-10-19 新增）------------------------------------
# 和 LOCAL_ORIGIN 完全同一個理由：drone_api_node 刻意不知道地圖的存在，
# 場景知識由啟動的人注入，而底座（drone-nodered.service）是獨立的服務，
# warehouse_all.launch.py 內部那條線接不到它 —— 只能靠這裡填。
#
# ⚠️ 這件事漏掉的後果特別陰險：api/track 的**跟隨段**是 cmd_vel 直通
#    offboard、不經過 Nav2，所以那一段完全沒有避障。沒有方框的話飛機
#    照樣起飛、照樣追、照樣回報成功，直到某一次站位點算錯就直接撞上去
#    —— 2026-10-19 就是這樣撞上第 2 排貨架、從 5 m 高翻覆墜地。
#    節點啟動時會印黃色警告說「沒有禁飛方框」，看到就是這裡沒填對。
NO_FLY=""
if [ -f "$GEOM" ]; then
    NO_FLY="$(sed -n 's/^no_fly_boxes: *\[\(.*\)\].*/\1/p' "$GEOM" | tr -d ' ')"
fi
if [ -z "$NO_FLY" ]; then
    echo "⚠ 讀不到 $GEOM 的 no_fly_boxes —— 跟隨段會沒有禁飛保險。"
    echo "  解法：python3 $PKG_SRC/scripts/gen_warehouse.py --target hatchback"
    echo "        之後重跑這支。"
fi

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
          -e "s|@PKG_SRC@|$PKG_SRC|g" \
          -e "s|@LOCAL_ORIGIN@|$LOCAL_ORIGIN|g" \
          -e "s|@CAMERA_PITCH@|$CAMERA_PITCH|g" \
          -e "s|@NO_FLY@|$NO_FLY|g" "$src"
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
install_one drone-warehouse.service
if [ "$WITH_EMPTY_SIM" = 1 ]; then
    install_one drone-sim.service
else
    # 不裝，而且把以前裝過的清掉 —— 留著會變成「沒有任何按鈕會去按」的孤兒 unit，
    # 而且 Node-RED 的「系統」分頁已經沒有對應的按鈕了。
    # 只刪本腳本產生的（第一行有 MARKER），別人手寫的不動。
    stale="$UNIT_DIR/drone-sim.service"
    if [ -e "$stale" ]; then
        if head -1 "$stale" | grep -qF "$MARKER"; then
            if [ "$DRY_RUN" = 1 ]; then
                echo "  [dry-run] 會移除已退休的 $stale"
            else
                systemctl --user stop drone-sim.service 2>/dev/null || true
                rm -f "$stale"
                echo "  ✓ 移除已退休的 drone-sim.service（空白世界）"
                echo "    要叫回來： $0 --with-empty-sim"
            fi
        else
            echo "  ⚠ $stale 不是本腳本產生的 —— 保留不動"
        fi
    fi
fi

echo
echo "安裝桌面圖示…"
APPS_DIR="$HOME/.local/share/applications"
DESKTOP_DIR="$(xdg-user-dir DESKTOP 2>/dev/null || echo "$HOME/Desktop")"
for d in drone-nodered-start.desktop drone-nodered-dashboard.desktop drone-nodered-stop.desktop; do
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

#!/usr/bin/env bash
# =============================================================================
#  start_empty_sitl.sh — 在空白世界（gz/worlds/empty.sdf）啟動單台 PX4 SITL
#
#  用法：
#      ~/ros2_ws/src/drone_nodered/scripts/start_empty_sitl.sh
#      HEADLESS=1 ~/ros2_ws/src/drone_nodered/scripts/start_empty_sitl.sh     # 不開視窗
#      FOREGROUND=1 ~/ros2_ws/src/drone_nodered/scripts/start_empty_sitl.sh   # 前景模式，Ctrl+C 收乾淨
#
#  開完之後（另開終端）：
#      MicroXRCEAgent udp4 -p 8888
#      ros2 launch drone_nodered drone_api.launch.py
#
#  停止：
#      前景模式（FOREGROUND=1）→ 按 Ctrl+C，腳本自己收掉 PX4 和 Gazebo
#      預設模式              → pkill -x px4 ; pkill -f "gz sim"
#      （第二行一定要用 -f：gz 是 Ruby 包裝腳本，程序名是 ruby，-x 抓不到）
#
#  為什麼要有前景模式：
#      預設模式把 PX4 丟到背景後腳本就結束，PX4 和 Gazebo 留著沒人管，要自己 pkill。
#      交給 systemd 當服務跑時這樣行不通 —— systemd 看「主程式」判斷服務在不在，
#      腳本一結束它就認定服務結束，把整組程序收掉，Gazebo 開起來馬上被關。
#      前景模式讓「腳本的壽命 = 模擬的壽命」，收到停止訊號才去收尾。
#
#  ⚠️ 這支是從 drone_nav2_apriltag/scripts/start_arena_sitl.sh 複製改來的。
#     差別只有：世界換成 empty、機型用 PX4 內建的 x500、只開一台、放在世界原點。
#     NVIDIA 檢查、GZ_IP、預檢參數這些「踩坑修好的設定」完全照抄 ——
#     之後其中一支又修了新的坑，記得另一支也要跟著改。
#
#  為什麼用內建 x500 而不是 x500_nav2：
#     x500_nav2（前後相機 + 光達）放在 drone_nav2_apriltag 裡，用它就得依賴那個套件。
#     起飛／盤旋／降落用不到感測器，用內建的 x500 這個套件就完全獨立。
# =============================================================================
set -e

PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
BUILD_DIR="$PX4_DIR/build/px4_sitl_default"
PKG_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

if [ ! -x "$BUILD_DIR/bin/px4" ]; then
    echo "找不到 $BUILD_DIR/bin/px4，請先執行： cd $PX4_DIR && make px4_sitl_default"
    exit 1
fi

# 世界檔的位置：從原始碼目錄執行時在 <pkg>/gz/worlds；
# 從安裝目錄（install/lib/drone_nodered）執行時，往上一層找不到，改問 ROS 2 套件路徑。
WORLDS_DIR="$PKG_DIR/gz/worlds"
if [ ! -f "$WORLDS_DIR/empty.sdf" ]; then
    WORLDS_DIR="$(ros2 pkg prefix drone_nodered 2>/dev/null)/share/drone_nodered/gz/worlds"
fi
if [ ! -f "$WORLDS_DIR/empty.sdf" ]; then
    echo "找不到 empty.sdf（試過 $PKG_DIR/gz/worlds 和安裝目錄）"
    exit 1
fi

# --- 強制 Gazebo 走 NVIDIA 獨顯（照抄 start_arena_sitl.sh）-----------------------
# 雙顯卡預設走內顯 → 物理步進被拖慢 → lockstep 下 PX4 收不到 IMU → 預檢失敗。
# ⚠️ 要問「獨顯現在能不能用」，不是「nvidia-smi 在不在」：
#    2026-09-11 驅動升級沒重開機，nvidia-smi 還在但跑不動，硬推會讓 Gazebo 默默關掉。
if nvidia-smi -L >/dev/null 2>&1; then
    export __NV_PRIME_RENDER_OFFLOAD=1
    export __GLX_VENDOR_LIBRARY_NAME=nvidia
    export __VK_LAYER_NV_optimus=NVIDIA_only
    echo "已啟用 NVIDIA offload"
elif command -v nvidia-smi >/dev/null 2>&1; then
    echo "⚠️  nvidia-smi 跑不起來，獨顯現在無法使用 —— 改走內顯。原因："
    nvidia-smi -L 2>&1 | sed 's/^/     /'
    echo "     最常見的是「驅動升級後還沒重開機」，重開機就會好。"
else
    echo "找不到 nvidia-smi，維持預設顯示卡"
fi

# --- 把 gz-transport 綁在回環位址（照抄；2026-08-31 的關鍵修正）------------------
# 不設的話 IMU 傳遞抖動 → Accel TIMEOUT → EKF 劣化 → 起飛後失效保護 → 墜毀。
export GZ_IP=127.0.0.1

# --- Gazebo 資源路徑 ---------------------------------------------------------------
# 先 source gz_env.sh（它無條件覆寫 PX4_GZ_WORLDS），再把世界改指到本套件。順序不能反。
# PX4_GZ_MODELS 不動：用 PX4 自己的模型目錄，裡面就有 x500。
# shellcheck disable=SC1091
source "$BUILD_DIR/rootfs/gz_env.sh"
export PX4_GZ_WORLDS="$WORLDS_DIR"
# ⚠️ 必須和 empty.sdf 裡的 <world name="empty"> 一致（原因見 empty.sdf 檔頭）
export PX4_GZ_WORLD=empty

NAME="MAV1"
INSTANCE=0
# 空白世界沒有東西要閃，直接放在世界原點
POSE="0,0"
# PX4_SIM_MODEL 而不是 PX4_GZ_MODEL：後者自 v1.15 起已廢棄，設了沒有作用
SIM_MODEL="x500"

echo "世界： $PX4_GZ_WORLDS/$PX4_GZ_WORLD.sdf"
echo "機體： $PX4_GZ_MODELS/$SIM_MODEL/model.sdf"

wait_for() {
    local desc="$1" timeout="$2"; shift 2
    local waited=0
    while ! "$@" >/dev/null 2>&1; do
        sleep 1
        waited=$((waited + 1))
        if [ "$waited" -ge "$timeout" ]; then
            echo "  ✗ 逾時（${timeout}s）：$desc"
            return 1
        fi
    done
    echo "  ✓ $desc（耗時 ${waited}s）"
    return 0
}

world_is_up()       { gz topic -l 2>/dev/null | grep -qE "/world/empty/clock"; }
instance_is_ready() { grep -q "uxrce_dds_client.*vehicle_local_position" "$1"; }

if ! pgrep -x MicroXRCEAgent >/dev/null 2>&1; then
    echo
    echo "  ⚠ 沒偵測到 MicroXRCEAgent。"
    echo "    只想看場景 → 不用理它，等下的「已連上 XRCE Agent」會逾時，那是正常的。"
    echo "    要用 ROS 2 控制 → 先另開一個終端跑： MicroXRCEAgent udp4 -p 8888"
    echo
    WAIT_AGENT=15
else
    WAIT_AGENT=90
fi

# --- 預檢參數（照抄 start_arena_sitl.sh，原因的長說明在那支）---------------------
fix_preflight_params() {
    local i="$1"
    local param="$BUILD_DIR/bin/px4-param"
    # 沒開 QGC 也能 ARM
    "$param" --instance "$i" set NAV_DLL_ACT 0          >/dev/null 2>&1 || return 1
    "$param" --instance "$i" set CBRK_SUPPLY_CHK 894281 >/dev/null 2>&1 || return 1
    # 模擬電池預設 60 秒耗到 50% → 觸發返航
    "$param" --instance "$i" set SIM_BAT_DRAIN 86400    >/dev/null 2>&1 || return 1
    "$param" --instance "$i" set SIM_BAT_MIN_PCT 99     >/dev/null 2>&1 || return 1
    "$param" --instance "$i" set COM_LOW_BAT_ACT 0      >/dev/null 2>&1 || return 1
    # offboard 失聯判定從 1 秒放寬到 10 秒；真的失聯時原地懸停（5 = Hold）
    "$param" --instance "$i" set COM_OF_LOSS_T 10.0     >/dev/null 2>&1 || return 1
    "$param" --instance "$i" set COM_OBL_RC_ACT 5       >/dev/null 2>&1 || return 1
    # 磁偏角：empty.sdf 的磁場和 nav2_arena.sdf 相同 → atan2(6e-6, 2.3e-5) = 14.62 度。
    # 不設的話 PX4 用經緯度查表（約 3 度），航向差十幾度。
    "$param" --instance "$i" set EKF2_DECL_TYPE 0       >/dev/null 2>&1 || return 1
    "$param" --instance "$i" set EKF2_MAG_DECL 14.62    >/dev/null 2>&1 || return 1
    "$param" --instance "$i" set EKF2_MAG_TYPE 0        >/dev/null 2>&1 || return 1
    "$param" --instance "$i" save                       >/dev/null 2>&1 || return 1
    return 0
}

# --- 收尾：只收這次自己開的那些 ------------------------------------------------------
# Gazebo 不是這支腳本開的，是 PX4 啟動時執行 px4-rc.gzsim 開的：
#   :50  gz sim -r -s <世界>   （伺服器，算物理）
#   :54  gz sim -g             （視窗，沒設 HEADLESS 才開）
# 兩行都加了 &，所以 PX4 結束時不會把它們帶走，要我們自己收。
gz_leftovers() {
    # 比對啟動時間：只認「比這支腳本啟動 PX4 還晚出現」的，才不會誤殺你另外開的模擬
    local now age_limit p etimes
    now="$(date +%s)"
    age_limit=$(( now - LAUNCH_EPOCH + 2 ))   # +2 秒容忍取樣誤差
    for p in $(pgrep -f "gz sim" 2>/dev/null); do
        etimes="$(ps -o etimes= -p "$p" 2>/dev/null | tr -d ' ')"
        [ -n "$etimes" ] && [ "$etimes" -le "$age_limit" ] && echo "$p"
    done
}

STOPPED=0
stop_sim() {
    trap - INT TERM        # 收尾途中再按 Ctrl+C 不要重複進來
    # 被訊號打斷時，trap 收完尾、wait 也會跟著返回 → 下面那行又會呼叫一次，擋掉
    [ "$STOPPED" = 1 ] && return 0
    STOPPED=1
    echo
    echo "收工中…"

    # 先抓 PX4 的子程序再殺 PX4 —— 順序反過來的話，PX4 一死子程序就被 init 收養，
    # pgrep -P 就查不到了。
    local kids targets p
    kids="$(pgrep -P "$PX4_PID" 2>/dev/null | tr '\n' ' ')"
    targets="$(gz_leftovers | tr '\n' ' ')"
    echo "  PX4 PID $PX4_PID，子程序： ${kids:-（無）}"

    kill "$PX4_PID" 2>/dev/null || true
    for p in $kids $targets; do
        kill "$p" 2>/dev/null || true
    done

    # 給它們 10 秒好好關（Gazebo 要存檔、釋放顯卡資源），超時才強制
    local waited=0
    while kill -0 "$PX4_PID" 2>/dev/null || [ -n "$(gz_leftovers)" ]; do
        sleep 1
        waited=$((waited + 1))
        if [ "$waited" -ge 10 ]; then
            echo "  ⚠ 10 秒還沒關完，強制結束"
            kill -9 "$PX4_PID" 2>/dev/null || true
            for p in $(gz_leftovers); do kill -9 "$p" 2>/dev/null || true; done
            break
        fi
    done
    echo "  ✓ 收乾淨了"
    # 直接結束腳本：trap 跑完 bash 會回到原本的位置繼續（例如還在等 Gazebo 的迴圈），
    # 那時 PX4 已經被收掉，再等下去只是白等到逾時，systemd 也會一直卡在停止中。
    exit 0
}

# --- 清理舊程序 --------------------------------------------------------------------
echo "清掉可能殘留的舊程序…"
pkill -x px4 || true
pkill -f "gz sim" || true
sleep 2

# --- 啟動 --------------------------------------------------------------------------
WORK_DIR="$BUILD_DIR/instance_$INSTANCE"
mkdir -p "$WORK_DIR"
rm -f "$WORK_DIR/out.log"
echo "啟動 $NAME  (instance $INSTANCE, MAV_SYS_ID $((INSTANCE+1)), 位置 E,N = $POSE)"
# 記下啟動時刻：收尾找 Gazebo 時，用「比腳本晚開的才算我們的」來避免誤殺別人的模擬
LAUNCH_EPOCH="$(date +%s)"
# 這裡刻意不用小括號包起來。小括號會開子 shell，$! 拿到的 PID 傳不回來，
# 而前景模式的 wait / kill 都要指名那一支 PX4（用名字殺會連你另外開的一起殺掉）。
cd "$WORK_DIR"
PX4_UXRCE_DDS_NS="$NAME" \
PX4_SYS_AUTOSTART=4001 \
PX4_SIM_MODEL="$SIM_MODEL" \
PX4_GZ_MODEL_POSE="$POSE" \
HEADLESS="${HEADLESS:-}" \
"$BUILD_DIR/bin/px4" -i "$INSTANCE" -d "$BUILD_DIR/etc" \
    > "$WORK_DIR/out.log" 2>&1 &
PX4_PID=$!
cd "$OLDPWD"

# 立刻安裝收尾：啟動要 20 秒左右，這段期間被 systemd 停掉（或按 Ctrl+C）也要收得乾淨。
# 之前把這行放在「就緒」之後，結果啟動中途關閉會留下 Gazebo，
# 最後是 systemd 等 45 秒逾時強制砍掉，服務狀態變成 failed。
if [ -n "${FOREGROUND:-}" ]; then
    trap stop_sim INT TERM
fi

wait_for "Gazebo 世界 empty 已建立" 90 world_is_up
wait_for "$NAME 已連上 XRCE Agent" "$WAIT_AGENT" instance_is_ready "$WORK_DIR/out.log" || true

if fix_preflight_params "$INSTANCE"; then
    echo "  ✓ $NAME 預檢參數已設定（ARM、電池、offboard 失聯、磁偏角）"
else
    echo "  ⚠ $NAME 預檢參數設定失敗 —— ARM 可能會被擋"
fi

echo
echo "=================================================="
echo " 就緒，世界： empty（單機 $NAME）"
echo "=================================================="
echo "log： $WORK_DIR/out.log"

if [ -n "${FOREGROUND:-}" ]; then
    echo "停止： 按 Ctrl+C（前景模式，會自己收掉 PX4 和 Gazebo；trap 在 PX4 啟動時就裝好了）"
    echo
    # 卡在這裡等 PX4 —— 腳本活著才有機會在你喊停時收尾。
    # || true：被訊號打斷時 wait 的回傳值不是 0，會被 set -e 當成錯誤
    wait "$PX4_PID" || true
    # PX4 自己當掉的情況：wait 正常返回，但 Gazebo 還在，一樣要收
    stop_sim
else
    echo "停止： pkill -x px4 ; pkill -f 'gz sim'"
    echo "      （或下次改用 FOREGROUND=1 啟動，Ctrl+C 就會自己收乾淨）"
fi

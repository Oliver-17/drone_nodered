#!/usr/bin/env bash
# =============================================================================
#  t_launch_order.sh —— 回歸測試：launch_arguments 的求值順序
#
#  用法：bash src/drone_nodered/test/t_launch_order.sh      （約 10 秒）
#
#  為什麼需要這個測試：
#      「依 model 決定的值」寫在 IncludeLaunchDescription 的 launch_arguments
#      裡會**算錯**。launch_arguments 是有序的 SetLaunchConfiguration ＋
#      延遲求值，而被 include 的 launch 如果改寫了 model（sim_all 會把它
#      改成「<model>_0」加上 PX4 的實例編號），外層後面才求值的表達式就會
#      讀到改寫後的值。而且**改寫會外洩到外層作用域**。
#
#      這個雷咬過兩次，症狀都是「看起來有設定但值是錯的，而且不報錯」：
#        2026-10-06  sim_all 的 camera_pitch → 俯角永遠報 45° 而不是 67°
#        2026-10-08  warehouse_all 的 camera_pitch_deg → api/track 的最佳
#                    跟隨距離算成 2.97 m 而不是 1.27 m，下俯餘裕只剩 5.3°，
#                    飛機一加速就把目標頂出畫面而追丟
#
#  正解：**凡是「依 model 決定」的值，一律提到外層 DeclareLaunchArgument**，
#        在任何東西改寫 model 之前就求值，include 裡只轉交不重算。
# =============================================================================
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# ⚠️ 不要用 set -u。ROS 的 setup.bash 會讀 AMENT_TRACE_SETUP_FILES 這類
#    未綁定的變數，set -u 下 source 它會直接失敗（「未綁定的變數」）。
source /opt/ros/humble/setup.bash
fail=0

run() {
    timeout 25 ros2 launch "$HERE/launch_order/$1.launch.py" 2>&1 \
      | grep -oE "(BAD|GOOD) +俯角 = [0-9.]+" | tail -1
}

echo "期望：model=x500_cam_down → 俯角 67.0"
bad="$(run outer_bad)"
good="$(run outer_good)"
echo "  $bad"
echo "  $good"
echo

# 壞的寫法**必須**算錯 —— 它算對了代表 launch 的行為變了，
# 那時候上面那些註解和繞法就該重新檢視（不是壞事，但要知道）。
case "$bad" in
    *45.0) echo "✓ 壞寫法確實算錯（45.0）—— 這個雷還在，繞法仍有必要" ;;
    *67.0) echo "⚠ 壞寫法竟然算對了 —— launch 的求值行為可能變了，"
           echo "  請重新檢視 sim_all / warehouse_all 裡關於 camera_pitch 的註解" ;;
    *)     echo "✗ 壞寫法沒有輸出：$bad"; fail=1 ;;
esac
case "$good" in
    *67.0) echo "✓ 好寫法算對（67.0）" ;;
    *)     echo "✗ 好寫法算錯了：$good —— 提到外層宣告的繞法失效了，要查"; fail=1 ;;
esac
exit "$fail"

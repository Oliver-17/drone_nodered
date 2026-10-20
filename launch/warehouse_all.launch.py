# =============================================================================
#  warehouse_all.launch.py — 倉庫場景的完整堆疊，一行啟動
#
#  用法：
#      ros2 launch drone_nodered warehouse_all.launch.py
#      ros2 launch drone_nodered warehouse_all.launch.py rviz:=false target:=false
#      ros2 launch drone_nodered warehouse_all.launch.py headless:=true
#
#  起來之後（飛機還在地上）：
#      ros2 run drone_control arm_and_takeoff.py --ns MAV1 --altitude 3.0
#      然後在 RViz 用「2D Goal Pose」點走道，或用 /MAV1/api/goto（L2 之後）
#
#  ---------------------------------------------------------------------------
#  為什麼要有這支：不只是少開三個終端機
#
#  手動開這四個節點時，有三個參數打錯**完全不會報錯**：
#
#    odom_origin ≠ 生成點   整張地圖平移。Nav2 照樣規劃、照樣回報「成功」，
#                           但飛機在別的地方。沒有任何錯誤訊息。
#    忘記 params_file       nav2.launch.py:55-58 會退回 arena 那份舊參數，
#                           也就是還帶著 default_server_timeout: 20 的版本
#                           → 第三個目標必定卡死（2026-10-06 實機就是這樣）。
#    忘記 use_sim_time      只影響 RViz 和 goal_tool，看起來沒事。
#
#  所以這支把 params_file / map / odom_origin / use_sim_time **寫死不開放**。
#  它們必須互相一致，開放成參數等於把上面三個靜默失敗放回來。
#
#  ---------------------------------------------------------------------------
#  生成點的單一來源
#
#  spawn 從 config/warehouse_geometry.yaml 讀（那個檔由 gen_warehouse.py 產生）。
#  同一組數字餵給兩個地方：
#      ① sim_all → start_empty_sitl.sh → PX4_GZ_MODEL_POSE （飛機生成在哪）
#      ② px4_bridge 的 odom_origin                        （odom 原點在 map 的哪裡）
#  這兩個不一致就是上面第一條。讀同一個檔是唯一能保證它們相等的做法。
#
#  ---------------------------------------------------------------------------
#  ⚠️ 啟動順序是有理由的，不要為了快而縮短
#
#      t=0s    Gazebo + PX4 + XRCE agent      （sim_all）
#      t=BRIDGE_DELAY   px4_bridge            ← 要等 PX4 的 uORB topic 出來
#      t=NAV2_DELAY     Nav2                  ← 要等 px4_tf_node 開始發 TF
#      t=API_DELAY      drone_api             ← 要等 /navigate_to_pose 上線
#
#  Nav2 一定要在 bridge 之後：lifecycle_manager 在 configure 階段就要查 TF，
#  TF 還沒有的話 costmap 進不了 active，而訊息只說
#  「Timed out waiting for transform」—— 很容易誤判成參數錯。
# =============================================================================

import os
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            LogInfo, TimerAction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

PKG = "drone_nodered"
WORLD = "warehouse"

# 預設機體。兩個都沒有光達，所以 warehouse_nav2.yaml 的 local_costmap
# 一律用 static_layer 而不是 obstacle_layer（見那個檔的檔頭）。
#   x500           內建機體，沒有相機 —— 只跑 Nav2 導航時用這個，最輕
#   x500_cam_down  腹部支架上的相機，俯角 67°、無盲區、拍不到機體
#                  —— api/track 的視覺追蹤要用這個
DEFAULT_MODEL = "x500"

# 各階段的啟動延遲（秒）。這些值是實測調出來的，不是估的：
#   太短 → 看到一串 TF 逾時警告然後 lifecycle 失敗
#   太長 → 只是多等
# 調的時候從 Nav2 的 log 看「Creating bond timer」什麼時候出現。
BRIDGE_DELAY = 25.0
NAV2_DELAY = 35.0
API_DELAY = 45.0


def _spawn():
    """從產生出來的幾何檔讀 spawn，回傳 (x, y) 字串。

    為什麼不用 yaml.safe_load：這支 launch 在 colcon 安裝後執行，
    多一個 import 就多一個可能沒裝的相依。檔案是產生的、格式固定，
    正規表示式夠可靠。

    讀不到就直接 raise —— 絕對不要退回 (0,0)。那會變成「靜默平移」，
    正是這整支 launch 要消除的失敗模式。
    """
    path = os.path.join(get_package_share_directory(PKG), "config",
                        f"{WORLD}_geometry.yaml")
    with open(path, encoding="utf-8") as f:
        m = re.search(r"^spawn:\s*\[\s*([-\d.]+)\s*,\s*([-\d.]+)\s*\]",
                      f.read(), re.M)
    if not m:
        raise RuntimeError(
            f"{path} 裡找不到 spawn —— 請重跑 "
            f"python3 src/{PKG}/scripts/gen_warehouse.py")
    return m.group(1), m.group(2)


def _no_fly():
    """從幾何檔讀禁飛方框，回傳攤平的字串（給 drone_api_node 的參數用）。

    格式和 _spawn() 同一個原則：檔案是產生的、一行一個鍵、用正規表示式讀，
    不為了這個多拉一個 yaml 相依進 launch。

    讀不到就回空字串 —— **這裡和 _spawn() 不一樣，不 raise**。
    理由：沒有禁飛方框只是少了一道保險（節點會明確印出「沒有禁飛方框」），
    而讀不到 spawn 會讓整張地圖平移、靜默地飛錯地方。
    兩者的嚴重性不同，所以失敗處理也不同。
    """
    path = os.path.join(get_package_share_directory(PKG), "config",
                        f"{WORLD}_geometry.yaml")
    try:
        with open(path, encoding="utf-8") as f:
            m = re.search(r"^no_fly_boxes:\s*\[([^\]]*)\]", f.read(), re.M)
    except OSError:
        return ""
    return m.group(1).replace(" ", "") if m else ""


def _target_spawn():
    """從幾何檔讀目標車的生成位置，回傳 "x,y" 字串（讀不到回空字串）。

    只給儀表板畫「還沒偵測到車之前車應該在哪」，所以讀不到就不顯示車 ——
    和 _no_fly() 同一個等級（少一個方便，不會飛錯地方），**不 raise**。
    """
    path = os.path.join(get_package_share_directory(PKG), "config",
                        f"{WORLD}_geometry.yaml")
    try:
        with open(path, encoding="utf-8") as f:
            m = re.search(r"^target_spawn:\s*\[\s*([-\d.]+)\s*,\s*([-\d.]+)\s*\]",
                          f.read(), re.M)
    except OSError:
        return ""
    return f"{m.group(1)},{m.group(2)}" if m else ""


def _default_detector():
    """讀世界檔，決定預設用哪個偵測器。

    世界裡是哪一種目標車，就只有對應的偵測器找得到它：
        magenta_target    洋紅方盒 → color（HSV 找色塊）
        hatchback_target  真車外觀 → yolo（VisDrone 權重）
    配錯的話偵測輸出會是**全空**，而且不會報錯 —— 畫面上只會看到
    「追蹤中但永遠找不到目標」，很難看出是配對問題。

    ⚠️ 為什麼讓 launch 自己讀，而不是在呼叫端寫死：
        世界檔是 gen_warehouse.py --target 產生的，檔名不變（WORLD 這個名字
        同時綁著 maps/ 和 config/ 兩個檔），所以「現在是哪一種」只有檔案內容
        知道。寫死在 systemd 的 ExecStart 裡的話，每次切換世界都要連服務一起改、
        重裝。讀檔就不會有不同步的問題。
        和 scripts/start_empty_sitl.sh 偵測 TARGET_MODEL 是同一個原則。

    明確傳 detector:=xxx 時以傳入的為準（DeclareLaunchArgument 的預設值
    只在沒傳的時候生效）。
    """
    path = os.path.join(get_package_share_directory(PKG), "gz", "worlds",
                        f"{WORLD}.sdf")
    try:
        with open(path, encoding="utf-8") as f:
            text = f.read()
    except OSError:
        # 讀不到就退回 color（原本的行為）。這裡不 raise：世界檔不存在的話
        # 底下 sim_all 會用更清楚的訊息失敗，不需要在這裡搶著報錯。
        return "color"
    if "hatchback_target" in text:
        return "yolo"
    return "color"


def generate_launch_description():
    share = get_package_share_directory(PKG)
    sx, sy = _spawn()
    detector = _default_detector()

    params_file = os.path.join(share, "config", f"{WORLD}_nav2.yaml")
    map_yaml = os.path.join(share, "maps", f"{WORLD}.yaml")
    for p in (params_file, map_yaml):
        if not os.path.isfile(p):
            raise RuntimeError(f"找不到 {p} —— 先 colcon build --packages-select {PKG}")

    ns = LaunchConfiguration("namespace")

    return LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="MAV1"),
        DeclareLaunchArgument("headless", default_value="false",
                              description="true = 不開 Gazebo 視窗（RViz 還是會開）"),
        DeclareLaunchArgument("altitude", default_value="3.0",
                              description="bridge 的巡航高度。貨架高 5 m，不要超過"),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="開 RViz（看 costmap 和規劃出來的路徑）"),
        DeclareLaunchArgument("target", default_value="true",
                              # 車本身是世界檔 include 進來的（warehouse.sdf:259），
                              # 所以 false 不是「移除」而是「不推它」—— 車會靜止
                              # 停在走道裡。它在地上、飛機在 3 m，不影響飛行；
                              # 而且 local_costmap 只有 static_layer，Nav2 看不到它。
                              description="true = 中間走道的車會南北巡迴；"
                                          "false = 車靜止不動（不會消失）"),
        DeclareLaunchArgument("model", default_value=DEFAULT_MODEL,
                              description="x500（無相機，預設）或 "
                                          "x500_cam_down（俯角 67° 腹部相機，"
                                          "視覺追蹤要用）"),
        # ⚠️ 俯角一定要在**外層**宣告，不能寫在 include 的 launch_arguments 裡。
        #    launch_arguments 是有序的 SetLaunchConfiguration ＋ 延遲求值，而
        #    上面的 sim_all 會把 model 改寫成「x500_cam_down_0」（加 PX4 的實例
        #    編號）。寫在裡面的話這個條件判斷會在改寫之後才求值 → 永遠掉到
        #    else 分支。
        #    2026-10-06 在 sim_all 的 camera_pitch 上踩過一次（症狀：俯角永遠
        #    報 45°），2026-10-08 又在這裡踩了一次（症狀：最佳跟隨距離算成
        #    2.97 m 而不是 1.27 m，下俯餘裕只剩 5.3°，一加速就追丟）。
        #    **凡是「依 model 決定」的值，一律提到外層宣告。**
        DeclareLaunchArgument(
            "camera_pitch_deg",
            default_value=PythonExpression(
                ["'67.0' if '", LaunchConfiguration("model"),
                 "' == 'x500_cam_down' else '45.0'"]),
            description="相機安裝俯角（度）。依 model 自動選，"
                        "給 api/track 算最佳跟隨距離用"),
        # 預設值依**世界檔裡是哪種目標車**自動決定（見 _default_detector）。
        # 明確傳 detector:=color 或 yolo 可以覆寫。
        DeclareLaunchArgument("detector", default_value=detector,
                              choices=["color", "yolo"],
                              description="偵測器：color 找洋紅方盒、yolo 認真車外觀。"
                                          "預設依世界檔自動選"),
        LogInfo(msg=f"--- 世界 {WORLD} 的目標車 → 偵測器預設 {detector} ---"),
        DeclareLaunchArgument("vision", default_value="false",
                              description="視覺偵測（色彩偵測→目標世界座標）。"
                                          "要配 model:=x500_cam_down 才有畫面"),
        DeclareLaunchArgument("api", default_value="true",
                              description="drone_api_node（Node-RED 的服務層）"),

        LogInfo(msg=["倉庫堆疊啟動 —— 生成點 (", sx, ", ", sy,
                     ")，同一組值會傳給 odom_origin"]),
        LogInfo(msg=["  參數檔 " + os.path.basename(params_file)
                     + "   地圖 " + os.path.basename(map_yaml)]),
        LogInfo(msg=["  機體 ", LaunchConfiguration("model"),
                     "   視覺 ", LaunchConfiguration("vision")]),

        # --- ① 模擬（Gazebo + PX4 + agent，可選視覺）------------------------
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share, "launch", "sim_all.launch.py")),
            launch_arguments={
                "world": WORLD,
                # ⚠️ 不要在這裡順手傳 camera_pitch —— sim_all 自己會依 model 算，
                #    而 launch_arguments 是有序的 SetLaunchConfiguration ＋ 延遲求值，
                #    在這裡重算會讀到被改寫過的 model（2026-10-06 踩過這個雷，
                #    症狀是俯角永遠報 45° 而不是 67°）。
                "model": LaunchConfiguration("model"),
                "headless": LaunchConfiguration("headless"),
                "vision": LaunchConfiguration("vision"),
                "detector": LaunchConfiguration("detector"),
                # target_mode 留空 → start_empty_sitl.sh 依世界名挑 "aisle"
                # （車只沿走道的 y 走，不會橫越貨架）。關掉車就把速度設 0。
                "target_mode": "",
                "target_speed": PythonExpression(
                    ["'0.5' if '", LaunchConfiguration("target"),
                     "'.lower() in ('true','1','yes') else '0'"]),
            }.items(),
        ),

        # --- ② 橋接（PX4 ↔ ROS 2，含 TF）------------------------------------
        TimerAction(period=BRIDGE_DELAY, actions=[
            LogInfo(msg="--- 開 px4_bridge（PX4 的 uORB 應該已經就緒）---"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory("drone_control"),
                    "launch", "px4_bridge.launch.py")),
                launch_arguments={
                    "namespace": ns,
                    "flight_altitude": LaunchConfiguration("altitude"),
                    # ⚠️ 這裡就是「同一組數字」—— 和 ① 的生成點來自同一個檔。
                    "odom_origin": f"{sx},{sy},0.0",
                }.items(),
            ),
        ]),

        # --- ③ Nav2（要等 ② 的 TF）------------------------------------------
        TimerAction(period=NAV2_DELAY, actions=[
            LogInfo(msg="--- 開 Nav2（px4_tf_node 應該已經在發 map→odom→base_link）---"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory("drone_control"),
                    "launch", "nav2.launch.py")),
                launch_arguments={
                    "namespace": ns,
                    "level": "full",
                    # 寫死：見檔頭「忘記 params_file」那一條
                    "params_file": params_file,
                    "map": map_yaml,
                    # 寫死 false：sim_all 沒有橋 /clock（見 warehouse_nav2.yaml 檔頭）
                    "use_sim_time": "false",
                    "rviz": LaunchConfiguration("rviz"),
                    # goal_tool 是給「只跑 planner」用的捷徑。level=full 時
                    # bt_navigator 自己就接 2D Goal Pose，開著會兩個人搶同一個 topic。
                    "goal_tool": "false",
                }.items(),
            ),
        ]),

        # --- ④ 服務層（要等 ③ 的 /navigate_to_pose）--------------------------
        TimerAction(period=API_DELAY, actions=[
            LogInfo(msg="--- 開 drone_api（Node-RED 的服務層）---"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(share, "launch", "drone_api.launch.py")),
                launch_arguments={
                    "namespace": ns,
                    # ⚠️ 和上面 px4_bridge 的 odom_origin **同一組數字**，
                    #    都來自 config/warehouse_geometry.yaml 的 spawn。
                    #    api/track 的搜尋環線是 map 座標，節點要靠這個換算成
                    #    PX4 本地 NED。兩邊不一致會讓航點整組平移且不報錯 ——
                    #    這就是為什麼它們在這裡共用同一個變數而不是各自寫死。
                    "local_origin_map": f"{sx},{sy}",
                    # 值在外層就算好了（見上面 DeclareLaunchArgument 的說明），
                    # 這裡只是轉交。**不要**在這裡重算 —— model 已經被改寫過。
                    "camera_pitch_deg": LaunchConfiguration("camera_pitch_deg"),
                    # 跟隨段的禁飛方框（貨架 + 外牆），同樣來自 warehouse_geometry.yaml。
                    # 跟隨是 cmd_vel 直通 offboard、不經過 Nav2，所以 costmap 的
                    # 保護在那一段完全不存在 —— 2026-10-19 撞過一次貨架。
                    "no_fly_boxes": _no_fly(),
                    # 儀表板用：車生在哪。紅點是 YOLO 看到的位置，所以飛機
                    # 還沒看到車之前地圖上本來完全沒有車 —— 這個值讓它
                    # 一開始就畫出「車應該在哪」（空心圓圈）。
                    "target_home_map": _target_spawn(),
                }.items(),
                condition=IfCondition(LaunchConfiguration("api")),
            ),
            LogInfo(msg=["堆疊就緒。起飛： ros2 run drone_control "
                         "arm_and_takeoff.py --ns MAV1 --altitude ",
                         LaunchConfiguration("altitude")]),
        ]),
    ])

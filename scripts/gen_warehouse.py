#!/usr/bin/env python3
# =============================================================================
#  gen_warehouse.py — 倉庫場景幾何的唯一真實來源
#
#  用法：
#      python3 src/drone_nodered/scripts/gen_warehouse.py
#      python3 src/drone_nodered/scripts/gen_warehouse.py --check-only   # 只驗證不寫檔
#
#  跑一次會重新產生：
#      gz/worlds/warehouse.sdf        Gazebo 世界（外牆 + 貨架）
#      maps/warehouse.pgm + .yaml     Nav2 靜態地圖（costmap 的 StaticLayer）
#      config/warehouse_geometry.yaml 給 launch / 腳本讀的幾何（spawn、走道中心）
#      docs/warehouse_preview.png     俯視圖（沒裝 matplotlib 就跳過）
#
#  為什麼要有這支腳本：
#      Gazebo 世界的貨架和 Nav2 地圖的貨架必須對得上，但它們是兩種不同格式的
#      檔案。手動改遲早會改到不同步，而且**不同步不會報錯** —— 只會在飛的時候
#      莫名其妙撞貨架，或規劃出一條穿過貨架的路。把幾何寫在同一個地方、
#      一次全部產生，從源頭杜絕。
#
#  改場景的流程：
#      1. 改下面「幾何定義」那一段
#      2. 跑這支腳本（結尾會自動做四項自我檢查）
#      3. colcon build --packages-select drone_nodered
#
#  ⚠️ 這支會「覆寫」上面那幾個檔案。不要直接手改它們，改了會被蓋掉。
#
#  ⚠️ 和 drone_nav2_apriltag/scripts/gen_arena.py 的關係：
#      兩支共用同一套「幾何 → SDF + PGM」的產生邏輯（約 150 行重複）。
#      沒有抽成共用模組，是因為那要把 drone_nav2_apriltag 變成可 import 的
#      Python 套件，違反「一套件一 repo、各自獨立」的結構。
#      **改了其中一支的產生邏輯，想一下另一支要不要跟著改。**
#      幾何定義本身完全不同（arena 是走廊迷宮，這裡是貨架陣列），不會互相影響。
# =============================================================================

import argparse
import math
import os
import sys

# =============================================================================
#  幾何定義 —— 要改場景就改這一段，其他都不用動
# =============================================================================

WALL_HEIGHT = 8.0     # 外牆高。飛行高度約 3 m，8 m 保證飛不出去
WALL_THICK = 0.5
# 貨架高。**一定要比飛行高度高** ——
# Nav2 是 2D，地圖說「不能過」就不能過，但物理上飛得過去。
# 矮貨架會讓「繞過去」變成視覺假象而不是真的避障。
#
# 2026-10-08：5.0 → 7.0。追蹤時為了讓畫面穩一點把飛行高度從 3 m 提到 5 m，
# 而貨架也剛好是 5 m —— 飛機可以直接從上面飛過去，整個場景的前提就壞了。
# 7 m 留 2 m 餘裕，而且還是低於外牆的 8 m。
SHELF_HEIGHT = 7.0

# --- 屋頂（2026-10-08 新增）------------------------------------------------
# 白色半透明，從 Gazebo 的視角可以看穿進去看走道。
#
# ⚠️ 屋頂**不能加進 RECTS**。RECTS 同時餵給 SDF 和地圖的點陣化
#    （build_map_rows / self_check / write_preview 都在掃它），加進去的話
#    整張 36×24 m 的地圖會變成全部不可通行，Nav2 直接癱掉。
#    所以它由 roof_model_xml() 單獨產生，只進 SDF 不進地圖。
ROOF_ENABLED = True
ROOF_HEIGHT = 8.0      # 屋頂下緣的高度。和外牆頂齊平
ROOF_THICK = 0.2
ROOF_ALPHA = 0.25      # 0 = 全透明、1 = 不透明

# --- 倉庫外框 ---------------------------------------------------------------
HALF_X = 18.0         # 外牆內緣 x = ±18 → 36 m 寬
HALF_Y = 12.0         # 外牆內緣 y = ±12 → 24 m 深

# --- 貨架陣列 ---------------------------------------------------------------
# 走道寬度的依據（不要憑感覺改）：
#   模擬的 nav2_params：robot_radius 0.5、inflation_radius 1.0
#   致命環：飛機中心離貨架面至少 0.5 m → 兩側共吃掉 1.0 m
#   膨脹層：再往外 1.0 m 成本遞減 → 兩側共 2.0 m
#   => 走道 3.0 m 的話中間只剩「一條線」，planner 會覺得整條走道都貴
#      走道 4.0 m 中間還有 1.0 m 的零成本帶，而且好事多的走道本來就 3~4 m
#   ⚠️ 以後三機編隊要連 robot_radius 一起調大，那時走道要重算
#      （arena 用 9 m 主通道就是這個原因）
AISLE_WIDTH = 4.0
SHELF_DEPTH = 1.6     # x 方向（走道垂直方向）
SHELF_LENGTH = 14.0   # y 方向，從 y=-7 到 y=+7
SHELF_PITCH = SHELF_DEPTH + AISLE_WIDTH     # 5.6 m

# 四排貨架，對稱排在中線兩側 → 中間那條走道的中心剛好在 x=0，
# 車子在那裡巡迴時座標好記。
SHELF_CENTERS_X = [-1.5 * SHELF_PITCH, -0.5 * SHELF_PITCH,
                   0.5 * SHELF_PITCH, 1.5 * SHELF_PITCH]      # -8.4 -2.8 2.8 8.4

# --- 飛機與目標車 -----------------------------------------------------------
# 飛機 spawn 在西南角的起飛區。從這裡到中間走道沒有任何直線可走 ——
# (-14,-9) 到 (0,0) 的直線在 x=-9.2 時 y≈-5.9，正好穿過第一排貨架。
# 這就是「Nav2 真的有用」的理由。
SPAWN = (-14.0, -9.0)
# ⚠️ 這個值有三個消費者，而且**不一致不會報錯**：
#      ① 這支腳本的自我檢查（spawn 必須是可通行格）
#      ② start_empty_sitl.sh      —— 告訴 PX4 把飛機生成在哪
#      ③ px4_bridge 的 odom_origin —— 告訴 px4_tf_node「odom 原點在 map 的哪裡」
#    ②③ 不一致的後果是整張地圖平移：Nav2 會規劃、會說「成功」、會回報到達，
#    但飛機實際上在別的地方。沒有任何錯誤訊息。
#    所以 ②③ 都改成讀下面產生的 config/warehouse_geometry.yaml，不再各自寫死。

# 目標車巡迴的走道（中間那條）與 y 範圍。給 move_target.py 的預設值參考用，
# 這支腳本只把它寫進註解和預覽圖，不產生車子（車子由 world 的 include 或
# move_target.py 處理）。
PATROL_AISLE_X = 0.0
PATROL_Y_RANGE = (-6.0, 6.0)

# --- 目標車繞行的環線（2026-10-08 改成繞一排貨架轉圈）-----------------------
# 為什麼從「走道內南北來回」改成「繞一排貨架」：
#   來回的路線是一條線段，飛機只要進到那條走道就一直看得到車，
#   「搜尋」這件事沒有難度。繞圈之後車會跑到貨架另一側（視線被擋），
#   飛機必須也繞過去才找得到 —— 搜尋才有意義，而且比較像真實的巡檢。
PATROL_SHELF_INDEX = 2        # 繞 SHELF_CENTERS_X 的第幾排（0 起算）→ x=+2.8
                              # 選這一排的理由：兩側環線剛好落在走道中心
                              # x=0 和 x=+5.6，離貨架面 2 m，最寬鬆
LOOP_OFFSET = 2.0             # 環線離貨架面多遠。2.0 = 走道中心
                              # （走道寬 4.0，貨架面到走道中心剛好 2.0）

# 飛機搜尋用的環線離貨架面多遠。和車**同一條**（也是 2.0），
# 這樣兩者在同一條線上反向跑，相遇最快（47 m 環線、飛機 2 m/s → 最久 19 秒）。
SEARCH_OFFSET = 2.0


def shelf_loop(index, offset):
    """繞第 index 排貨架、離貨架面 offset 公尺的矩形環線。

    回傳四個角（map 座標，逆時針從西南角開始）。

    為什麼是矩形而不是圓：Gazebo 的 VelocityControl 外掛 linear 是世界座標、
    angular.z 無效（move_target.py 檔頭有實測記錄），所以車只能走「分段直線」。
    而飛機這側的巡航是點對點 DO_REPOSITION，也只能走直線。兩邊都是矩形最自然。
    """
    cx = SHELF_CENTERS_X[index]
    hx = SHELF_DEPTH / 2.0 + offset          # 離貨架中心的半寬
    hy = SHELF_LENGTH / 2.0 + offset         # 離貨架中心的半長
    return [(cx - hx, -hy), (cx - hx, hy), (cx + hx, hy), (cx + hx, -hy)]

# 兩條環線。車和飛機用同一個 offset（都是走道中心），所以其實是同一條 ——
# 差別只在**跑的方向相反**，那由 move_target.py 和巡航航點的順序決定。
TARGET_LOOP = shelf_loop(PATROL_SHELF_INDEX, LOOP_OFFSET)
# 飛機的環線反向（車逆時針，飛機順時針）→ 相遇時間從「最久 94 秒」降到「最久 19 秒」。
# 同向時若飛機只比車快一點，會出現「跟在後面很久都碰不到」的尷尬狀況。
SEARCH_LOOP = list(reversed(shelf_loop(PATROL_SHELF_INDEX, SEARCH_OFFSET)))


# 矩形格式：(名稱, 中心x, 中心y, x向長度, y向長度, 高度, 種類, 說明)
# 外牆長度多加 WALL_THICK 是為了讓四個轉角互相重疊，避免接縫漏一條縫。
def _build_rects():
    r = [
        ("wall_west",  -HALF_X, 0.0, WALL_THICK, 2 * HALF_Y + WALL_THICK,
         WALL_HEIGHT, "wall", "西牆（起飛區那一側）"),
        ("wall_east",   HALF_X, 0.0, WALL_THICK, 2 * HALF_Y + WALL_THICK,
         WALL_HEIGHT, "wall", "東牆"),
        ("wall_south", 0.0, -HALF_Y, 2 * HALF_X + WALL_THICK, WALL_THICK,
         WALL_HEIGHT, "wall", "南牆（南向橫走道的外緣）"),
        ("wall_north", 0.0,  HALF_Y, 2 * HALF_X + WALL_THICK, WALL_THICK,
         WALL_HEIGHT, "wall", "北牆（北向橫走道的外緣）"),
    ]
    for i, cx in enumerate(SHELF_CENTERS_X):
        r.append((f"shelf_{i + 1}", cx, 0.0, SHELF_DEPTH, SHELF_LENGTH,
                  SHELF_HEIGHT, "shelf",
                  f"第 {i + 1} 排貨架，x 中心 {cx:+.1f}"))
    return r


RECTS = _build_rects()

# 走道中心線（用來自我檢查「走道真的可通行、而且寬度正確」）
AISLE_CENTERS_X = [(SHELF_CENTERS_X[i] + SHELF_CENTERS_X[i + 1]) / 2.0
                   for i in range(len(SHELF_CENTERS_X) - 1)]      # -5.6 0.0 5.6

# =============================================================================
#  以下是產生邏輯，改場景不需要動
# =============================================================================

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WORLD_NAME = "warehouse"

# 解析度。0.1 m 的依據和 arena 一樣：牆厚 0.5 m，一面牆至少佔 5 格才不會在
# 取樣時出現破洞；再細下去格數是平方成長（38x26 m 在 0.05 m 下是 20 萬格）。
MAP_RES = 0.1

# 地圖邊界要比外牆再外擴一點，否則最外側的牆剛好貼在圖的邊緣，
# 膨脹層會被裁掉一半，規劃器會覺得那裡比實際上更好走。
MAP_MARGIN = 1.0

# ROS 佔據網格慣例：0 = 障礙物、254 = 可通行、205 = 未知。
# 這裡不會產生「未知」—— 整個場地都是我們自己定義的。
PGM_OCCUPIED = 0
PGM_FREE = 254

COLORS = {   # ambient, diffuse
    "wall":  ("0.30 0.30 0.35 1", "0.65 0.65 0.70 1"),
    "shelf": ("0.35 0.22 0.08 1", "0.80 0.52 0.22 1"),   # 橘棕色，和灰牆分得開
}


def point_in_rect(px, py, cx, cy, sx, sy):
    """點在不在某個軸對齊矩形裡。

    這裡的矩形都沒有旋轉（倉庫是正交佈局），所以不像 gen_arena.py 那樣
    需要先轉進矩形自己的座標系。少一層三角函數，也少一個出錯的地方。
    """
    return abs(px - cx) <= sx / 2.0 and abs(py - cy) <= sy / 2.0


def bounds():
    """場地外框（含牆厚）。"""
    xs, ys = [], []
    for _, cx, cy, sx, sy, _, _, _ in RECTS:
        xs += [cx - sx / 2, cx + sx / 2]
        ys += [cy - sy / 2, cy + sy / 2]
    return min(xs), max(xs), min(ys), max(ys)


def map_frame():
    """地圖的左下角座標與格數。世界座標和像素的換算只寫在這裡一處。"""
    x0, x1, y0, y1 = bounds()
    x0 -= MAP_MARGIN
    x1 += MAP_MARGIN
    y0 -= MAP_MARGIN
    y1 += MAP_MARGIN
    w = int(math.ceil((x1 - x0) / MAP_RES))
    h = int(math.ceil((y1 - y0) / MAP_RES))
    return x0, y1, w, h          # 左下角 x、左上角 y、寬、高


def rect_model_xml(name, cx, cy, sx, sy, hz, kind, note):
    amb, dif = COLORS[kind]
    size = f"{sx} {sy} {hz}"
    return f"""
    <!-- {note} -->
    <model name="{name}">
      <static>true</static>
      <pose>{cx} {cy} {hz / 2} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{size}</size></box></geometry>
        </collision>
        <visual name="visual">
          <geometry><box><size>{size}</size></box></geometry>
          <material>
            <ambient>{amb}</ambient>
            <diffuse>{dif}</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>
      </link>
    </model>
"""


def roof_model_xml():
    """白色半透明的屋頂。不在 RECTS 裡（理由見 ROOF_ENABLED 旁的說明）。

    為什麼有 collision：這是室內場景，屋頂應該要擋住飛機。飛行高度 5 m、
    屋頂下緣 8 m，正常不會碰到，但設定錯誤時會撞到屋頂而不是飛走。

    為什麼不會把裡面弄暗：世界的 <shadows> 和光源的 <cast_shadows> 都是
    false（為了讓俯視看得清楚走道），所以屋頂不投影。

    透明度同時用兩種寫法：<material> 的 diffuse/ambient 第四個數字（alpha）
    和 <visual> 的 <transparency> —— 不同的 gz 版本認的不一樣，兩個都給。
    """
    if not ROOF_ENABLED:
        return ""
    sx = 2 * HALF_X + WALL_THICK
    sy = 2 * HALF_Y + WALL_THICK
    cz = ROOF_HEIGHT + ROOF_THICK / 2
    return f"""
    <!-- 屋頂：白色半透明（alpha {ROOF_ALPHA}）。下緣 {ROOF_HEIGHT} m，和外牆頂齊平。
         ⚠️ 刻意不在 RECTS 裡 —— 它進 SDF 但不進 Nav2 的 2D 地圖。 -->
    <model name="roof">
      <static>true</static>
      <pose>0 0 {cz} 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><box><size>{sx} {sy} {ROOF_THICK}</size></box></geometry>
        </collision>
        <visual name="visual">
          <transparency>{1.0 - ROOF_ALPHA}</transparency>
          <geometry><box><size>{sx} {sy} {ROOF_THICK}</size></box></geometry>
          <material>
            <ambient>1 1 1 {ROOF_ALPHA}</ambient>
            <diffuse>1 1 1 {ROOF_ALPHA}</diffuse>
            <specular>0.1 0.1 0.1 {ROOF_ALPHA}</specular>
          </material>
        </visual>
      </link>
    </model>
"""


def build_world_text(target="magenta"):
    x0, x1, y0, y1 = bounds()
    n_shelf = sum(1 for r in RECTS if r[6] == "shelf")
    header = f"""<?xml version="1.0" encoding="UTF-8"?>
<!--
  === 這個檔案是產生出來的，不要手改 ===
  來源：drone_nodered/scripts/gen_warehouse.py，改幾何請改那支再重跑。

  warehouse：室內倉庫，給 Nav2 在貨架之間導航用。

  佈局
  ====
        y=+{HALF_Y:.0f} ─────────────────────────────── 北牆
        y=+{SHELF_LENGTH / 2:.0f}  ┌──┐  ┌──┐  ┌──┐  ┌──┐    ← 北向橫走道
              │貨│  │貨│  │貨│  │貨│
   起飛區     │架│  │架│  │架│  │架│      東側空地
              │  │  │  │  │  │  │  │
        y=-{SHELF_LENGTH / 2:.0f}  └──┘  └──┘  └──┘  └──┘    ← 南向橫走道
     ★spawn
        y=-{HALF_Y:.0f} ─────────────────────────────── 南牆

  設計原則
  ========
  1. 貨架高 {SHELF_HEIGHT:.0f} m，比飛行高度（約 3 m）高 —— 不放矮的、可飛越的東西。
     「從上面飛過去」是地圖設計上的取巧，不是真的避障。
  2. 走道寬 {AISLE_WIDTH:.1f} m 是算出來的，不是看起來順眼：
     robot_radius 0.5 兩側吃 1.0 m、inflation_radius 1.0 兩側再吃 2.0 m
     => 走道 3.0 m 中間只剩一條線，{AISLE_WIDTH:.1f} m 才有 1.0 m 的零成本帶。
  3. 飛機 spawn 在 ({SPAWN[0]:.0f}, {SPAWN[1]:.0f})，到中間走道沒有直線可走 ——
     直線在 x=-{SHELF_CENTERS_X[0] * -1 + SHELF_DEPTH / 2:.1f} 附近就穿進第一排貨架。
     這就是 Nav2 真的有用的理由。

  座標系：ENU（x 東、y 北、z 上）—— 見下方 <world_frame_orientation>。
  PX4 內部是 NED，前兩軸對調，寫任何座標前先確認自己在哪個系統。

  外框：x [{x0:.1f}, {x1:.1f}]  y [{y0:.1f}, {y1:.1f}]   ({x1 - x0:.0f} x {y1 - y0:.0f} m)
  貨架：{n_shelf} 排，走道中心 x = {", ".join(f"{v:+.1f}" for v in AISLE_CENTERS_X)}
  目標車巡迴：x = {PATROL_AISLE_X:+.1f} 的走道，y 從 {PATROL_Y_RANGE[0]:+.0f} 到 {PATROL_Y_RANGE[1]:+.0f}
-->
<sdf version="1.9">
  <world name="{WORLD_NAME}">
    <physics type="ode">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>250</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <magnetic_field>6e-06 2.3e-05 -4.2e-05</magnetic_field>
    <atmosphere type="adiabatic"/>
    <scene>
      <grid>false</grid>
      <ambient>0.4 0.4 0.4 1</ambient>
      <background>0.7 0.7 0.7 1</background>
      <!-- 影子關掉：倉庫裡四排高貨架會投下大片陰影，俯視時很難看清楚走道，
           而且陰影對 Nav2 完全沒有影響（它只讀靜態地圖）。要開回來改成 true，
           連同下面光源的 cast_shadows 一起。 -->
      <shadows>false</shadows>
    </scene>

    <!-- PX4 的 GPS 模擬需要一個原點經緯度；world_frame_orientation 決定
         SDF 的 <pose> 怎麼解讀。數值沿用 PX4 內建 world。 -->
    <spherical_coordinates>
      <surface_model>EARTH_WGS84</surface_model>
      <world_frame_orientation>ENU</world_frame_orientation>
      <latitude_deg>47.397971057728974</latitude_deg>
      <longitude_deg>8.546163739800146</longitude_deg>
      <elevation>0</elevation>
    </spherical_coordinates>

    <light name="sunUTC" type="directional">
      <pose>0 0 500 0 -0 0</pose>
      <cast_shadows>false</cast_shadows>
      <intensity>1</intensity>
      <direction>0.001 0.625 -0.78</direction>
      <diffuse>0.904 0.904 0.904 1</diffuse>
      <specular>0.271 0.271 0.271 1</specular>
      <attenuation>
        <range>2000</range><linear>0</linear><constant>1</constant><quadratic>0</quadratic>
      </attenuation>
      <spot><inner_angle>0</inner_angle><outer_angle>0</outer_angle><falloff>0</falloff></spot>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal><size>1 1</size></plane></geometry>
          <surface><friction><ode/></friction><bounce/><contact/></surface>
        </collision>
        <visual name="visual">
          <geometry><plane><normal>0 0 1</normal><size>200 200</size></plane></geometry>
          <material>
            <ambient>0.55 0.55 0.58 1</ambient>
            <diffuse>0.62 0.62 0.65 1</diffuse>
            <specular>0.2 0.2 0.2 1</specular>
          </material>
        </visual>
      </link>
    </model>

    <!-- ===== 外牆與貨架 ===== -->"""

    body = "".join(rect_model_xml(*r) for r in RECTS) + roof_model_xml()

    # 目標車：兩種外觀二選一（--target）。
    #   magenta   洋紅方塊 —— 顏色偵測 100% 準，拿來驗證控制邏輯用
    #   hatchback 真車外觀 —— 現成的航拍 YOLO 權重（VisDrone）認得，給視覺偵測用
    # 兩者尺寸幾乎一樣（0.6 x 0.4 vs 0.6 x 0.32），所以在畫面裡佔的比例相同，
    # 追蹤的站位和高度不用跟著改。
    # z 不一樣是因為模型原點的定義不同：magenta_target 的原點在車體中心
    # （所以要抬 0.075），hatchback_target 的原點在車輪著地的平面（所以給 0）。
    _targets = {
        "magenta":   ("magenta_target",   0.075),
        "hatchback": ("hatchback_target", 0.0),
    }
    _tname, _tz = _targets[target]
    footer = f"""
    <!-- ===== 目標車（{target}）=====
         VelocityControl 讓它可以用 /model/{_tname}/cmd_vel 被推動
         （move_target.py 在做這件事，換目標要加 model 選項）。
         ⚠️ 那個外掛的 angular.z 完全沒有作用（實測 0.5 rad/s 連續 21 秒，
            yaw 只變 0.00003），而 linear 是**世界座標**。所以車子只會平移、
            不會轉向 —— 在走道裡上下跑的時候看起來像側滑，那不是 bug。 -->
    <include>
      <uri>model://{_tname}</uri>
      <name>{_tname}</name>
      <pose>{PATROL_AISLE_X} {PATROL_Y_RANGE[0]} {_tz} 0 0 0</pose>
    </include>

  </world>
</sdf>
"""
    return header + body + footer


def build_map_rows():
    """回傳 PGM 的每一列（bytes）。

    座標系對應（最容易錯的地方）：
        PGM 的第 0 列是「影像最上面」，對應「世界 y 最大」那一邊，
        所以寫檔時 y 要從大到小掃。搞反的話地圖會上下顛倒，
        而顛倒的地圖在 RViz 裡看起來「也像一張地圖」，很難一眼看出不對。
    """
    x0, ytop, w, h = map_frame()
    rows = []
    for row in range(h):
        wy = ytop - (row + 0.5) * MAP_RES      # +0.5 取格子中心
        line = bytearray([PGM_FREE]) * w
        for col in range(w):
            wx = x0 + (col + 0.5) * MAP_RES
            for _, cx, cy, sx, sy, _, _, _ in RECTS:
                if point_in_rect(wx, wy, cx, cy, sx, sy):
                    line[col] = PGM_OCCUPIED
                    break
        rows.append(bytes(line))
    return rows


# -----------------------------------------------------------------------------
#  自我檢查 —— 產生完立刻驗，不要等到飛的時候才發現
# -----------------------------------------------------------------------------
def self_check(rows):
    x0, ytop, w, h = map_frame()
    fails = []

    def cell(wx, wy):
        col = int((wx - x0) / MAP_RES)
        row = int((ytop - wy) / MAP_RES)
        if not (0 <= col < w and 0 <= row < h):
            return None
        return rows[row][col]

    # ① 每個矩形的中心和四角往內縮一點，都必須是「佔據」
    for name, cx, cy, sx, sy, _, _, _ in RECTS:
        pts = [(cx, cy)]
        for ex in (-1, 1):
            for ey in (-1, 1):
                pts.append((cx + ex * (sx / 2 - MAP_RES), cy + ey * (sy / 2 - MAP_RES)))
        for px, py in pts:
            if cell(px, py) != PGM_OCCUPIED:
                fails.append(f"① {name} 在 ({px:.2f},{py:.2f}) 的地圖格不是障礙物")

    # ② 每條走道的中線取樣都必須「可通行」
    for ax in AISLE_CENTERS_X:
        for wy in [v / 2.0 for v in range(int(-SHELF_LENGTH), int(SHELF_LENGTH) + 1)]:
            if cell(ax, wy) != PGM_FREE:
                fails.append(f"② 走道 x={ax:+.1f} 在 y={wy:+.1f} 不可通行")
                break

    # ③ 沿 y=0 橫掃，量每條走道的連續可通行寬度
    row0 = int((ytop - 0.0) / MAP_RES)
    runs, cur = [], 0
    for col in range(w):
        if rows[row0][col] == PGM_FREE:
            cur += 1
        else:
            if cur:
                runs.append(cur)
            cur = 0
    if cur:
        runs.append(cur)
    want = int(round(AISLE_WIDTH / MAP_RES))
    inner = [r for r in runs if abs(r - want) <= 1]
    if len(inner) != len(AISLE_CENTERS_X):
        fails.append(f"③ y=0 橫掃找到 {len(inner)} 條寬度 {want}±1 格的走道，"
                     f"預期 {len(AISLE_CENTERS_X)} 條（實際的連續段：{runs}）")

    # ④ spawn 點必須在場地內、而且是可通行的，離任何矩形至少 robot_radius
    if cell(*SPAWN) != PGM_FREE:
        fails.append(f"④ spawn {SPAWN} 不是可通行格")
    for name, cx, cy, sx, sy, _, _, _ in RECTS:
        dx = max(abs(SPAWN[0] - cx) - sx / 2, 0.0)
        dy = max(abs(SPAWN[1] - cy) - sy / 2, 0.0)
        d = math.hypot(dx, dy)
        if d < 0.5:
            fails.append(f"④ spawn 離 {name} 只有 {d:.2f} m（< robot_radius 0.5）")

    # ⑤ spawn 到中間走道的直線必須被貨架擋住 —— 這是「Nav2 有用」的前提
    gx, gy = PATROL_AISLE_X, 0.0
    blocked = False
    for i in range(201):
        t = i / 200.0
        px = SPAWN[0] + (gx - SPAWN[0]) * t
        py = SPAWN[1] + (gy - SPAWN[1]) * t
        if cell(px, py) == PGM_OCCUPIED:
            blocked = True
            break
    if not blocked:
        fails.append("⑤ spawn 到中間走道的直線沒有被擋住 —— Nav2 不需要繞路，"
                     "場景設計失去意義")

    # ⑥ 環線的每一段直線都必須可飛（中心離障礙 >= robot_radius）
    #
    # 為什麼一定要檢查：車和飛機在環線上都是走**直線**（車是 Gazebo
    # VelocityControl、飛機是點對點 DO_REPOSITION），兩者都不會自己避開貨架。
    # 所以航點之間的直線必須全程在空的走道裡 —— 這件事不能用眼睛看，
    # 改了 AISLE_WIDTH / SHELF_LENGTH / LOOP_OFFSET 任何一個都可能破掉。
    def leg_clear(a, b, need=0.5):
        """沿線取樣，每個取樣點以 need 為半徑掃一圈，碰到障礙就回傳那一點。"""
        dist = math.hypot(b[0] - a[0], b[1] - a[1])
        steps = max(2, int(dist / MAP_RES))
        ring = max(1, int(need / MAP_RES))
        for k in range(steps + 1):
            t = k / steps
            px0 = a[0] + (b[0] - a[0]) * t
            py0 = a[1] + (b[1] - a[1]) * t
            for i in range(-ring, ring + 1):
                for j in range(-ring, ring + 1):
                    if i * i + j * j > ring * ring:
                        continue
                    if cell(px0 + i * MAP_RES, py0 + j * MAP_RES) != PGM_FREE:
                        return (px0, py0)
        return None

    for label, loop in (("目標車", TARGET_LOOP), ("飛機搜尋", SEARCH_LOOP)):
        for i in range(len(loop)):
            a, b = loop[i], loop[(i + 1) % len(loop)]
            bad = leg_clear(a, b)
            if bad is not None:
                fails.append(
                    f"⑥ {label}環線第 {i} 段 ({a[0]:+.1f},{a[1]:+.1f})→"
                    f"({b[0]:+.1f},{b[1]:+.1f}) 在 ({bad[0]:+.2f},{bad[1]:+.2f}) "
                    f"離障礙 < 0.5 m（robot_radius）")

    # ⑦ 環線必須真的繞過貨架 —— 兩個對側點之間的直線要被擋住。
    #    不然「繞圈」就只是在空地上畫方形，搜尋又失去意義（和 ⑤ 同一個精神）。
    a, b = SEARCH_LOOP[0], SEARCH_LOOP[2]      # 西南角 ↔ 東北角
    crossed = False
    for i in range(201):
        t = i / 200.0
        if cell(a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t) == PGM_OCCUPIED:
            crossed = True
            break
    if not crossed:
        fails.append("⑦ 環線對角線沒有被貨架擋住 —— 這圈沒有真的繞過東西")

    return fails, runs


def write_preview():
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.patches as mpatches
        import matplotlib.pyplot as plt
    except ImportError:
        return None
    x0, x1, y0, y1 = bounds()
    fig, ax = plt.subplots(figsize=(10, 7))
    for name, cx, cy, sx, sy, _, kind, _ in RECTS:
        fc = "#9aa0a6" if kind == "wall" else "#cc8533"
        ax.add_patch(mpatches.Rectangle((cx - sx / 2, cy - sy / 2), sx, sy,
                                        facecolor=fc, edgecolor="black", lw=0.6))
    for axc in AISLE_CENTERS_X:
        ax.plot([axc, axc], [-SHELF_LENGTH / 2, SHELF_LENGTH / 2],
                "--", color="#3a7bd5", lw=1)
    # 標籤刻意用英文：matplotlib 的預設字型沒有中文字符，會印成一堆方框。
    # 這張圖只是開發時的目視確認，中文說明在腳本檔頭和 SDF 註解裡。
    ax.plot([PATROL_AISLE_X, PATROL_AISLE_X], list(PATROL_Y_RANGE),
            "-", color="#d13b8a", lw=3, label="target car patrol")
    ax.plot(*SPAWN, "*", color="#1a7f37", markersize=18, label="drone spawn")
    ax.set_xlim(x0 - 1, x1 + 1)
    ax.set_ylim(y0 - 1, y1 + 1)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m, East)")
    ax.set_ylabel("y (m, North)")
    ax.set_title(f"warehouse  {x1 - x0:.0f} x {y1 - y0:.0f} m   "
                 f"aisle {AISLE_WIDTH:.1f} m")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="upper right", fontsize=8)
    out_dir = os.path.join(PKG_DIR, "docs")
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, "warehouse_preview.png")
    fig.savefig(path, dpi=110, bbox_inches="tight")
    plt.close(fig)
    return path


def loop_length(loop):
    """閉合環線的周長（公尺）。用來算相遇時間。"""
    return sum(math.hypot(loop[(i + 1) % len(loop)][0] - loop[i][0],
                          loop[(i + 1) % len(loop)][1] - loop[i][1])
               for i in range(len(loop)))


def _loop_yaml(loop):
    """[[x, y], [x, y], …] 的單行 YAML（產生的檔案要能被 sed/正規表示式讀）。"""
    return ", ".join(f"[{x:.3f}, {y:.3f}]" for x, y in loop)


def write_geometry():
    """寫出 config/warehouse_geometry.yaml。

    為什麼要這個檔：launch 和 shell 腳本都需要知道 spawn 在哪，但它們沒辦法
    import 這支 Python（一個是 shell、一個在 colcon 安裝後的 share 目錄裡）。
    產生一份純資料的 YAML 是唯一能讓三邊共用同一個數字的做法。

    刻意只放「別人真的會讀」的欄位，不把整組幾何常數都傾倒進去 ——
    放進來的每一個欄位都是一份要維護的對外介面。
    """
    out_dir = os.path.join(PKG_DIR, "config")
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, f"{WORLD_NAME}_geometry.yaml")
    with open(path, "w", encoding="utf-8") as f:
        f.write(
            "# === 這個檔案是產生出來的，不要手改 ===\n"
            "# 來源：drone_nodered/scripts/gen_warehouse.py，改幾何請改那支再重跑。\n"
            "#\n"
            "# 讀這個檔的人：\n"
            "#   launch/warehouse_all.launch.py   spawn → PX4 生成點 ＋ odom_origin\n"
            "#   scripts/start_empty_sitl.sh      spawn → PX4_GZ_MODEL_POSE\n"
            "#\n"
            "# 單位都是公尺，座標是 Gazebo 世界座標（ENU，x 東 y 北），\n"
            "# 而這個場景的 map 座標系和它完全相同（maps/%s.yaml 的 origin\n"
            "# 就是照這組幾何算的），所以同一組數字兩邊都能直接用。\n"
            "\n"
            "# 飛機生成點（西南角起飛區）。到中間走道沒有直線可走 —— 這是\n"
            "# 「Nav2 真的有用」的理由，不要改成空曠的地方。\n"
            "spawn: [%.3f, %.3f]\n"
            "\n"
            "# 可通行走道的中心 x。給「去第 N 條走道」這類指令用。\n"
            "aisle_centers_x: [%s]\n"
            "# 貨架中心 x（= 不可通行）。給自動產生測試點時排除用。\n"
            "shelf_centers_x: [%s]\n"
            "# 走道在 y 方向的可用範圍（貨架只到 ±%.1f）。\n"
            "aisle_y_range: [%.3f, %.3f]\n"
            "\n"
            "# 目標車巡迴的走道與範圍（move_target.py 的 aisle 模式，已不是預設）。\n"
            "patrol_aisle_x: %.3f\n"
            "patrol_y_range: [%.3f, %.3f]\n"
            "\n"
            "# --- 環線（2026-10-08 新增）---------------------------------------\n"
            "# 目標車繞第 %d 排貨架（x=%+.1f）轉圈，離貨架面 %.1f m。\n"
            "# move_target.py --mode loop 會依序走這些點並回到第一點。\n"
            "# ⚠️ 每一段都是直線，而且車和飛機都不會自己避障 ——\n"
            "#    這些點經過 gen_warehouse.py 的自我檢查⑥（離障礙 >= 0.5 m）。\n"
            "patrol_shelf_index: %d\n"
            "target_loop: [%s]\n"
            "\n"
            "# 飛機搜尋目標時巡航的環線。和車**同一條但方向相反** ——\n"
            "# 反向相遇最快（%.0f m 環線，飛機 2 m/s 對車 0.5 m/s → 最久 %.0f 秒）。\n"
            "# 同向的話最久要 %.0f 秒。\n"
            "search_loop: [%s]\n"
            % (WORLD_NAME,
               SPAWN[0], SPAWN[1],
               ", ".join(f"{v:.3f}" for v in AISLE_CENTERS_X),
               ", ".join(f"{v:.3f}" for v in SHELF_CENTERS_X),
               SHELF_LENGTH / 2.0,
               -SHELF_LENGTH / 2.0, SHELF_LENGTH / 2.0,
               PATROL_AISLE_X, PATROL_Y_RANGE[0], PATROL_Y_RANGE[1],
               PATROL_SHELF_INDEX, SHELF_CENTERS_X[PATROL_SHELF_INDEX],
               LOOP_OFFSET,
               PATROL_SHELF_INDEX, _loop_yaml(TARGET_LOOP),
               loop_length(SEARCH_LOOP),
               loop_length(SEARCH_LOOP) / (2.0 + 0.5),
               loop_length(SEARCH_LOOP) / (2.0 - 0.5),
               _loop_yaml(SEARCH_LOOP))
        )
    return path


def main():
    ap = argparse.ArgumentParser(description="產生倉庫場景（世界檔 + Nav2 地圖）")
    ap.add_argument("--check-only", action="store_true",
                    help="只做自我檢查，不寫任何檔案")
    ap.add_argument("--target", choices=["magenta", "hatchback"], default="magenta",
                    help="目標車的外觀：magenta=洋紅方塊（顏色偵測用，預設）、"
                         "hatchback=真車外觀（YOLO 用）")
    args = ap.parse_args()

    x0, x1, y0, y1 = bounds()
    mx0, mytop, mw, mh = map_frame()
    print(f"倉庫外框：{x1 - x0:.1f} x {y1 - y0:.1f} m   "
          f"x [{x0:.1f}, {x1:.1f}]  y [{y0:.1f}, {y1:.1f}]")
    print(f"貨架：{len(SHELF_CENTERS_X)} 排，深 {SHELF_DEPTH} / 長 {SHELF_LENGTH} / "
          f"高 {SHELF_HEIGHT} m，x 中心 "
          f"{', '.join(f'{v:+.1f}' for v in SHELF_CENTERS_X)}")
    print(f"走道：{AISLE_WIDTH:.1f} m 寬，中心 x = "
          f"{', '.join(f'{v:+.1f}' for v in AISLE_CENTERS_X)}")
    print(f"飛機 spawn：{SPAWN}")
    sx = SHELF_CENTERS_X[PATROL_SHELF_INDEX]
    print(f"目標車環線：繞第 {PATROL_SHELF_INDEX} 排貨架（x={sx:+.1f}），"
          f"離貨架面 {LOOP_OFFSET:.1f} m，周長 {loop_length(TARGET_LOOP):.1f} m")
    print(f"            {' → '.join(f'({x:+.1f},{y:+.1f})' for x, y in TARGET_LOOP)} → 回第一點")
    print(f"飛機搜尋環線：同一條但**反向**，周長 {loop_length(SEARCH_LOOP):.1f} m")
    print(f"            {' → '.join(f'({x:+.1f},{y:+.1f})' for x, y in SEARCH_LOOP)} → 回第一點")
    L = loop_length(SEARCH_LOOP)
    print(f"            相遇時間（車 0.5 m/s）：飛機 2.0 m/s 反向最久 {L/2.5:.0f} s"
          f"（同向要 {L/1.5:.0f} s）")
    print(f"地圖：{mw} x {mh} 格 @ {MAP_RES} m，左下角 "
          f"({mx0:.2f}, {mytop - mh * MAP_RES:.2f})")
    print()

    rows = build_map_rows()
    fails, runs = self_check(rows)
    occ = sum(r.count(PGM_OCCUPIED) for r in rows)
    print(f"障礙物格數 {occ} / {mw * mh} = {occ / (mw * mh) * 100:.1f}%")
    print(f"y=0 橫掃的連續可通行段（格）：{runs}")
    print()
    if fails:
        print("✗ 自我檢查沒過：")
        for f in fails:
            print("   " + f)
        return 1
    print("✓ 自我檢查全部通過（矩形佔據、走道通行、走道寬度、spawn、直線被擋、"
          "環線可飛、環線有繞過貨架）")

    if args.check_only:
        print("\n--check-only：沒有寫出任何檔案")
        return 0

    wpath = os.path.join(PKG_DIR, "gz", "worlds", f"{WORLD_NAME}.sdf")
    os.makedirs(os.path.dirname(wpath), exist_ok=True)
    with open(wpath, "w", encoding="utf-8") as f:
        f.write(build_world_text(args.target))
    print(f"\n寫出 {wpath}   （目標車：{args.target}）")

    map_dir = os.path.join(PKG_DIR, "maps")
    os.makedirs(map_dir, exist_ok=True)
    pgm = os.path.join(map_dir, f"{WORLD_NAME}.pgm")
    # P5 = 二進位灰階。自己寫檔而不用 PIL：這支只靠標準函式庫就能跑
    #（preview 那張才需要 matplotlib，而且是選用的）。
    with open(pgm, "wb") as f:
        f.write(b"P5\n# \xe7\x94\xb1 drone_nodered/scripts/gen_warehouse.py "
                b"\xe7\x94\xa2\xe7\x94\x9f\xef\xbc\x8c\xe4\xb8\x8d\xe8\xa6\x81"
                b"\xe6\x89\x8b\xe6\x94\xb9\n")
        f.write(f"{mw} {mh}\n255\n".encode())
        for r in rows:
            f.write(r)
    print(f"寫出 {pgm}   （{mw} x {mh} 格）")

    yml = os.path.join(map_dir, f"{WORLD_NAME}.yaml")
    ybottom = mytop - mh * MAP_RES
    with open(yml, "w", encoding="utf-8") as f:
        f.write(
            "# === 這個檔案是產生出來的，不要手改 ===\n"
            "# 來源：drone_nodered/scripts/gen_warehouse.py，改幾何請改那支再重跑。\n"
            "#\n"
            "# 給 nav2_map_server 載入，再由 costmap 的 StaticLayer 使用。\n"
            "# origin 是「左下角」的世界座標（x 最小、y 最小），第三個數字是 yaw。\n"
            f"image: {WORLD_NAME}.pgm\n"
            f"resolution: {MAP_RES:.6f}\n"
            f"origin: [{mx0:.6f}, {ybottom:.6f}, 0.000000]\n"
            "negate: 0\n"
            "# 門檻沿用 nav2 範例地圖的值。我們只產生 0 和 254 兩種像素，\n"
            "# 所以落在門檻之間的「未知」不會出現。\n"
            "occupied_thresh: 0.65\n"
            "free_thresh: 0.196\n"
        )
    print("寫出 " + yml)

    gpath = write_geometry()
    print("寫出 " + gpath)

    pv = write_preview()
    print("寫出 " + pv if pv else "（沒裝 matplotlib，跳過預覽圖）")
    return 0


if __name__ == "__main__":
    sys.exit(main())

#!/usr/bin/env python3
# =============================================================================
#  move_target.py —— 讓模擬世界裡的洋紅目標移動（繞圈或直線來回）
#
#  用法（模擬跑起來之後，另開一個終端）：
#      python3 ~/ros2_ws/src/drone_nodered/scripts/move_target.py
#      python3 ~/ros2_ws/src/drone_nodered/scripts/move_target.py --mode line --length 4
#      python3 ~/ros2_ws/src/drone_nodered/scripts/move_target.py --mode aisle --y-min -6 --y-max 6
#      python3 ~/ros2_ws/src/drone_nodered/scripts/move_target.py --stop    # 停下來
#
#  兩種走法：
#      circle（預設）繞圈 —— 目標一直在移動，飛機就會一直跟，適合看追蹤效果
#      line          直線來回 —— 動作單純好對答案，適合驗證位置算得準不準
#
#  為什麼繞圈比較適合看追蹤：
#      追蹤法則是「保持 2 公尺距離」，所以目標停下來時飛機也會停。
#      直線來回的話，目標往回走時距離變近，飛機不但不動，目標還會因為
#      相機下傾 45°、垂直視野只有 ±21° 而掉出畫面（靠到 1.2 m 就看不見了）。
#      繞圈則是一直在遠離，飛機會持續跟著轉。
#
#  ---------------------------------------------------------------------------
#  ⚠️ 為什麼不用 VelocityControl 外掛推，改成直接設 pose（2026-10-15 改）
#  ---------------------------------------------------------------------------
#  原本是發 cmd_vel 給 VelocityControl 外掛。那條路有個治不好的毛病：
#  **車頭方向控制不了**，所以車在南北走道裡跑的時候看起來是整台橫著側滑。
#
#  目標換成真車外觀之後這件事就不能接受了（而且 VisDrone 訓練資料裡的車
#  都是順著行進方向的）。2026-10-15 在空世界重測，確認外掛本身做不到：
#      ① 只給 angular.z=0.5 連續 4 秒  → yaw 變化 0.00°（理論 114.8°）
#      ② linear.x=0.4 + angular.z=0.5 → yaw 只變 14.7°，而且是**加速**增長的
#         （0.7 → 2.6 → 4.3 → 7.8 → 14.7），那是滑行摩擦造成的自旋，不是受控轉向
#      ③ 只給 linear.x=0.4（對照組）  → yaw 不變
#  沒有參數可調，所以改成繞過它：自己算位置和朝向，用 Gazebo 的
#  /world/<world>/set_pose 服務直接把車放到該在的位置和角度。
#
#  實測 set_pose 的成本（同一次測試）：
#      2/5/10/20 Hz 各跑 4 秒，**零失敗**，呼叫延遲中位 0.7~0.8 ms。
#      20 Hz 的預算是 50 ms，用掉不到 1 ms —— 完全不是瓶頸。
#      ⚠️ 但 timeout 不能給太小：第一次呼叫要建連線，實測吃掉 300 ms，
#         當初 timeout 設 300 ms 就是在那裡失敗的。現在給 2000 ms。
#
#  ⚠️ 代價：車**不再有物理**（每一拍被放到指定位置），撞不到東西也推不動東西。
#     這個目標本來就只是視覺標的（model.sdf 的 collision 早就簡化成 box 了），
#     所以可以接受。將來若要讓車真的會被撞、會卡住，就得改用 DiffDrive 或
#     AckermannSteering 外掛 —— 但那需要先把輪子拆成獨立的 link 和 joint，
#     而且會引進真實轉彎半徑，未必走得出環線那種 90 度直角。
#
#  ⚠️ 還留著的 VelocityControl：SDF 裡的外掛沒有移除（移掉洋紅版也要跟著改）。
#     啟動時會先送一則零速度把它歸零，免得殘留速度和 set_pose 打架。
#
#  ---------------------------------------------------------------------------
#  ⚠️ 時鐘要訂 /world/<world>/clock，**不能**拿 pose 訊息的時間戳當時鐘
#  ---------------------------------------------------------------------------
#  第一版就是拿 pose 訊息的 header.stamp 當時鐘，結果車「開三秒停一秒再往前噴」。
#  根因是把自己的輸出接回了自己的輸入：
#      set_pose → Gazebo 偵測到車動了 → 發 dynamic_pose（時間前進）→ dt>0 → 再 set_pose
#                           ↓ 某一拍沒發
#                      dt=0 → 車不動 → Gazebo 更沒理由發 → 卡住
#                           ↓ 直到別的事件打破僵局
#                      dt 累積很大 → 一口氣往前噴
#  2026-10-15 在跑著的完整模擬上量到的數字：
#      clock 8 秒內 2252 則、模擬時間前進 8.01 秒（RTF 1.00，模擬本身完全正常）
#      車的 pose 50.0 Hz、間隔穩定 20 ms
#      **但其中位置真的變了的只有 30%** —— 車只跑了指令速度的 40%
#  clock 跟模型動不動完全無關，車停著它照樣穩定發，所以不會形成那個迴圈。
#
#  ⚠️ 為什麼不能改用牆鐘（那樣更簡單）：車是模擬世界裡的物體，速度要以模擬時間
#     為準。RTF 不保證是 1（2026-10-06 整套 headless 跑時量到 0.83），
#     用牆鐘積分的話車在模擬世界裡就會走錯速度。
#
#  ⚠️ 這台有**三套時間**，不要混用（2026-10-15 查證）：
#        Gazebo 模擬時間   /world/<w>/clock —— Gazebo 物理、感測器時間戳、pose header
#        牆鐘              **所有 ROS 節點**（這個工作區沒橋 /clock，
#                          use_sim_time 全部寫死 false，見 config/warehouse_nav2.yaml 檔頭）
#        PX4 自己的時鐘    PX4 發出來的 uORB 訊息裡的 timestamp
#     這支不是 ROS 節點（直接用 gz.transport），所以用 Gazebo 的 clock 最自然，
#     而且只是訂閱、不是設定，不影響任何其他人。
#
#  ⚠️ 為什麼不用 `gz topic -p` 指令：那支 CLI 每跑一次要 0.9 秒（大部分是啟動開銷），
#     根本送不到 10 Hz。改用 Python 的 gz.transport13 常駐節點。
# =============================================================================

import argparse
import math
import signal
import sys
import time

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.clock_pb2 import Clock
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.twist_pb2 import Twist
from gz.transport13 import Node

# 預設目標。--model 可以換成別的（例如 hatchback_target，外觀是真車的那台）。
# 這兩個留成模組層級的全域，是因為底下好幾個函式都要用，
# 一路傳參數會把每個函式簽名都弄髒；main() 解析完參數後覆寫它們。
MODEL_NAME = "magenta_target"
TOPIC = f"/model/{MODEL_NAME}/cmd_vel"
# loop 模式：離航點這麼近就算到了，換下一個。
# 0.3 m 的依據：車速 0.5 m/s、位置回授約 60 Hz，一步只走 0.008 m，
# 所以 0.3 m 不會錯過；而且比 robot_radius 0.5 小，轉彎切角不會切進貨架。
# loop 模式：離航點多近算「到了」。
#
# ⚠️ 這個值會直接變成車在下一段直線上的橫向偏移 —— 車在還差這麼遠時就轉向，
#    等於從偏離中心這麼多的地方開始走，然後沿途慢慢收斂回來。
#    2026-10-15 實測（容忍 0.3 m、x=0 走道、從東邊那段轉進來）：
#        y -6~-3  x 平均 +0.109     y 0~+3   x 平均 +0.060
#        y -3~ 0  x 平均 +0.085     y +3~+6  x 平均 +0.036
#    永遠偏東（俯視圖的右邊），因為上一段固定從東邊過來。走道半寬只有 2 m，
#    偏 0.3 m 肉眼看得很清楚。
#
# 0.3 是**速度控制時代**的遺留：那時車是被外掛推著走、位置有誤差，容忍給小了
# 會錯過航點繞不回來。改成 set_pose 之後位置完全受控，不可能錯過，
# 所以改成「一拍半的步長」—— 只要這一拍走得到就算到，再把車精確對齊航點
# （見 Mover.snap）。偏移因此降到一拍的距離以內。
LOOP_ARRIVE_STEPS = 1.5
# 20 Hz：set_pose 是瞬移，更新率就是動畫的影格率。實測 20 Hz 零失敗、
# 延遲不到 1 ms，所以直接用 20（原本發 cmd_vel 時是 10）。
RATE_HZ = 20.0
DISCOVERY_S = 1.0       # gz-transport 要先發現訂閱端才送得出去，太快發第一則會被丟掉
# 車頭最快轉多快（rad/s）。環線的轉角是 90°，直接套 atan2 車頭會瞬間翻過去，
# 看起來像瞬移。90°/s 表示轉一個直角要一秒，和真車在倉庫裡掉頭的感覺差不多。
YAW_RATE = math.radians(90.0)
# ⚠️ set_pose 的 timeout 要**短**，這點很反直覺（2026-10-15 查了很久）：
#
#  gz.transport 的 Python binding 有個毛病 —— **訂閱的 callback 會把同步
#  request 的回應吃掉**。訂得越密、掉得越多，實測同一個完整模擬裡：
#        不訂閱             失敗  0/150
#        只訂 pose (50Hz)   失敗  4/150
#        只訂 clock(300Hz)  失敗 12/150
#        兩個都訂           失敗 15/150 = 10%
#  而掉了的那次回應**永遠不會來**，呼叫端就卡滿整個 timeout。
#  所以 timeout 給 2000 ms 的話，每 10 拍就白白送掉 2 秒 ——
#  這正是「車開三秒停一秒再往前噴」的真正原因（不是時鐘，不是服務慢：
#  成功的呼叫中位只要 0.3 ms）。
#
#  給 25 ms 之後實測 150 拍只花 8.1 秒（理想 7.5），而且成功的中位還是 0.3 ms，
#  證明沒有把「慢」誤判成「失敗」—— 失敗的本來就是回應不會來的那些。
#  失敗那一拍不重試：位置是我們自己記的，下一拍會把累積的位置一次送出去
#  （車在 Gazebo 裡跳 2 拍的距離，約 5 cm，看不出來）。
SET_POSE_TIMEOUT_MS = 25
# 第一次呼叫要建連線，實測吃掉 100~300 ms，用正常的 timeout 一定失敗。
# 所以 __init__ 裡先用這個長一點的 timeout 暖機一次。
SET_POSE_WARMUP_MS = 2000
# 單拍最多認多少模擬時間。模擬偶爾會卡一下，2026-10-15 實測 200 拍裡
# 出現過一次 2060 ms 的 dt —— 照著積分車會一口氣瞬移 1 公尺。
# 截掉之後那一拍只走 0.125 m（0.5 m/s），看起來只是慢了一點而不是瞬移。
# 代價：卡頓的那段路會少走。對四種模式都無所謂 ——
#   loop 用位置回授，少走下一拍自己補回來；
#   aisle/line 是計時反轉，少走只是來回範圍小一點點。
# ⚠️ 這個值要比「正常的一拍」大。正常值取決於 RTF：跑整套模擬時 RTF 約 0.83，
#    20 Hz 一拍約 40 ms；純物理無頭測試 RTF 可到 2.8，一拍約 60~140 ms。
MAX_DT_S = 0.25


class PoseWatcher:
    """訂 Gazebo 的姿態廣播，留下目標車的位置和朝向。

    兩樣都要：
        位置    開機時當積分的起點
        朝向    接續目前的車頭角度轉，不要一啟動就把車瞬間扭過去

    ⚠️ **不要**從這裡拿時間。pose 訊息只在模型「動了」才發，拿它的時間戳
        當時鐘會把自己的輸出接回自己的輸入（檔頭有完整說明）。時鐘用
        SimClock，那是獨立的來源。

    ⚠️ 拿不到位置時**大聲失敗**，不要默默退回計時模式。
        改成 set_pose 之後這點更重要：沒有起點就根本無從算起。
    """

    # SceneBroadcaster 發的兩個候選 topic（不同版本名稱不同，兩個都試）
    #
    # ⚠️ 兩條**不能同時訂**：pose/info 會週期重發所有模型（含靜態的）的 pose，
    #    dynamic_pose/info 只發會動的，兩條都寫同一組變數會互相蓋掉。
    #    2026-10-15 查 angular.z 時就是這樣量出一組自相矛盾的數字。
    #    所以底下是「訂到第一條就 break」，不是兩條都訂。
    CANDIDATES = ("/world/{w}/dynamic_pose/info", "/world/{w}/pose/info")

    def __init__(self, node, world):
        self.xy = None
        self.yaw = 0.0
        self.topic = None
        for pat in self.CANDIDATES:
            t = pat.format(w=world)
            if node.subscribe(Pose_V, t, self._on):
                self.topic = t
                break

    def _on(self, msg):
        for p in msg.pose:
            if p.name == MODEL_NAME:
                self.xy = (p.position.x, p.position.y)
                q = p.orientation
                # 只取繞 z 的轉角。車永遠貼著地面，roll/pitch 沒有意義。
                self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))
                return

    def wait(self, secs=10.0):
        end = time.monotonic() + secs
        while self.xy is None and time.monotonic() < end:
            time.sleep(0.05)
        return self.xy is not None


class SimClock:
    """訂 Gazebo 的模擬時鐘。

    這是整支程式唯一的時間來源。和 PoseWatcher 分開是刻意的 ——
    時鐘必須和「車動不動」完全無關，否則就變成自己餵自己（見檔頭）。

    實測（2026-10-15，完整模擬）：8 秒內 2252 則，模擬時間剛好前進 8.01 秒。
    車停著的時候它照樣發。
    """

    def __init__(self, node, world):
        self.t = None
        self.topic = f"/world/{world}/clock"
        self.ok = node.subscribe(Clock, self.topic, self._on)

    def _on(self, msg):
        self.t = msg.sim.sec + msg.sim.nsec * 1e-9

    def wait(self, secs=10.0):
        end = time.monotonic() + secs
        while self.t is None and time.monotonic() < end:
            time.sleep(0.05)
        return self.t is not None


def seg_dist(p, a, b):
    """點 p 到線段 ab 的距離，以及最近點在線段上的參數 t（0=a、1=b）。

    給 loop 模式挑起點用。用線段而不是端點是關鍵，見 nearest_leg。
    """
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 < 1e-12:                      # 退化成一個點
        return math.hypot(p[0] - ax, p[1] - ay), 0.0
    t = ((p[0] - ax) * dx + (p[1] - ay) * dy) / L2
    t = max(0.0, min(1.0, t))
    return math.hypot(p[0] - (ax + t * dx), p[1] - (ay + t * dy)), t


def nearest_leg(here, pts):
    """車目前離哪一段環線最近 → 回傳「下一個該去的航點」的索引。

    ⚠️ 為什麼看線段而不是看航點（2026-10-15 改）：
        原本是挑「最近的**航點**」，結果車生成在 (0,-6)、環線第一點在 (0,-9)
        時，車會先往南倒退 3 公尺去踩那個點，才開始順時針繞 —— 看起來像
        「出場先走錯方向」。
        但車其實**已經在** (0,-9)→(0,9) 這條邊上了（距離 0），正確的行為是
        直接朝那條邊的終點走。
        挑線段還有一個好處：車被移到環線上任何位置（例如 move_target 重啟過）
        都能平順接上，不會倒退。
    """
    best_i, best_d = 0, float("inf")
    for k in range(len(pts)):
        d, _t = seg_dist(here, pts[k], pts[(k + 1) % len(pts)])
        if d < best_d:
            best_d, best_i = d, k
    # 最近的是第 best_i 段（pts[best_i] → pts[best_i+1]），所以下一個目標是它的終點
    return (best_i + 1) % len(pts), best_d


def wrap_pi(a):
    """把角度收進 (-pi, pi]，算角度差時一定要做，否則 179° 到 -179° 會被
    當成轉了 358° 而不是 2°。"""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class Mover:
    """自己積分位置和朝向，用 set_pose 把車放到該在的地方。

    對外的介面刻意維持 send(vx, vy) 不變 —— 四種模式算速度向量的邏輯
    一行都不用改，換掉的只是「速度怎麼變成移動」這一層。
    """

    def __init__(self, world):
        self.node = Node()
        self.service = f"/world/{world}/set_pose"
        self.clock = SimClock(self.node, world)
        if not self.clock.ok:
            print(f"✗ 訂不到模擬時鐘 {self.clock.topic}", flush=True)
            print(f"  確認模擬有在跑，而且 --world 給對了（現在是 {world}）", flush=True)
            sys.exit(1)
        self.watcher = PoseWatcher(self.node, world)
        if self.watcher.topic is None:
            print(f"✗ 訂不到姿態 topic（試過 "
                  f"{', '.join(c.format(w=world) for c in PoseWatcher.CANDIDATES)}）",
                  flush=True)
            print(f"  確認模擬有在跑，而且 --world 給對了（現在是 {world}）", flush=True)
            sys.exit(1)
        if not self.watcher.wait():
            print(f"✗ 訂到了 {self.watcher.topic} 但 10 秒內沒看到 {MODEL_NAME}",
                  flush=True)
            print("  確認世界裡真的有這個模型（--model 給對了嗎）", flush=True)
            sys.exit(1)
        if not self.clock.wait():
            print(f"✗ 訂到了 {self.clock.topic} 但 10 秒內沒收到時間", flush=True)
            print("  模擬是不是暫停了？", flush=True)
            sys.exit(1)

        # VelocityControl 外掛還掛在模型上。先送一則零速度把它歸零，
        # 否則殘留的速度會在每個 step 把車推走，和我們設的 pose 打架。
        self.vpub = self.node.advertise(TOPIC, Twist)
        time.sleep(DISCOVERY_S)
        self.halt()

        self.x, self.y = self.watcher.xy
        self.yaw = self.watcher.yaw      # 從車目前的朝向接續，不要一啟動就扭一下
        self.t_sim = self.clock.t
        self.calls = self.fails = 0      # set_pose 的統計（見 _set_pose）
        self.stalls = 0                  # 連續幾拍沒等到時間前進（診斷用）

        # 位置拿到了就把 pose 退訂掉。之後的位置是我們自己積分的，不需要再讀，
        # 而每一個訂閱都會提高 set_pose 掉回應的機率（見 SET_POSE_TIMEOUT_MS）：
        # 實測少掉這個 50 Hz 的訂閱，失敗率從 15/150 降到 12/150。
        self.node.unsubscribe(self.watcher.topic)

        # 暖機：第一次呼叫要建連線（100~300 ms），用正常的短 timeout 必定失敗。
        # 設成目前的位置和朝向，所以車不會動。
        self._set_pose(SET_POSE_WARMUP_MS)
        self.calls = self.fails = 0
        # ⚠️ 暖機那一下可能卡滿 2 秒（建連線 + 可能掉回應），這段時間模擬還在跑。
        #    不重設的話第一拍的 dt 會是那 2 秒，車一開始就往前竄一步
        #    （雖然 MAX_DT_S 會截住，但那一步還是比正常的大十倍）。
        self.t_sim = self.clock.t

    def halt(self):
        """送一則零速度給 VelocityControl。車本身不會因此停下來（它根本
        不是靠速度在動），這純粹是避免外掛殘留速度干擾 set_pose。"""
        msg = Twist()
        msg.linear.x = msg.linear.y = msg.linear.z = 0.0
        self.vpub.publish(msg)

    def _set_pose(self, timeout_ms=None):
        req = Pose()
        req.name = MODEL_NAME
        req.position.x = float(self.x)
        req.position.y = float(self.y)
        # z 固定 0：hatchback 的模型原點就在車輪著地面。magenta_target 的
        # 原點在車體中心（0.075），但它不在乎朝向，本來也不會用這條路。
        req.position.z = 0.0
        req.orientation.w = math.cos(self.yaw / 2.0)
        req.orientation.z = math.sin(self.yaw / 2.0)
        # 第三個參數是**型別**不是實例（binding 的簽名是 request_type）。
        # 傳實例也能動，但實測最大延遲 101 ms vs 1 ms，傳類別乾淨得多。
        ok, _ = self.node.request(self.service, req, Pose, Boolean,
                                  timeout_ms or SET_POSE_TIMEOUT_MS)
        # 失敗是常態（見 SET_POSE_TIMEOUT_MS 的說明，約一成），不重試也不吵。
        # 但要留著統計 —— 失敗率突然暴增表示服務真的不通了，那是另一回事。
        self.calls += 1
        if not ok:
            self.fails += 1
        if self.calls >= 200:                       # 20 Hz 下約 10 秒
            if self.fails > self.calls * 0.5:
                print(f"⚠️ set_pose 最近 {self.calls} 次有 {self.fails} 次沒有回應"
                      f"（{self.fails / self.calls * 100:.0f}%）—— 模擬還在嗎？",
                      flush=True)
            self.calls = self.fails = 0
        return ok

    def snap(self, x, y):
        """把車精確放到某個點（loop 模式換航點時用）。

        不這樣做的話，每一段直線都會從「上一段的到達容忍」那麼遠的地方起步，
        整段都歪著走（見 LOOP_ARRIVE_STEPS 的實測數字）。
        朝向不動 —— 轉向交給下一拍的 send()，才會是平滑轉過去而不是瞬間翻。
        """
        self.x, self.y = float(x), float(y)
        self._set_pose()

    def step_len(self, speed):
        """這一拍大概會走多遠。給 loop 模式算到達容忍用。

        用名目的 1/RATE_HZ 而不是實際的 dt：dt 要等 send() 才知道，
        而到達判定在 send() 之前。差異只影響「提早或晚一拍轉彎」，
        而 snap 會把位置對齊，所以不影響最終路徑。
        """
        return speed / RATE_HZ

    def send(self, vx, vy=0.0):
        """吃一個世界座標的速度向量，把它變成這一拍的位移和轉向。

        速度的意義和改版前完全一樣（世界座標、m/s），所以四種模式不用動。
        """
        now = self.clock.t
        if now is None or self.t_sim is None:
            return
        dt = now - self.t_sim
        # dt <= 0 只剩一個原因：模擬被暫停了。這一拍就不要動 ——
        # 硬積分會在暫停時把車推走。
        # （時鐘換成 clock 之前還有第二個原因：pose 訊息沒發。那是個會卡死的
        #   迴圈，見檔頭。所以這裡順便數連續次數，真的卡住要講出來。）
        if dt <= 0.0:
            self.stalls += 1
            if self.stalls == 200:       # 20 Hz 下約 10 秒
                print("⚠️ 模擬時間連續 10 秒沒有前進 —— 模擬暫停了嗎？", flush=True)
            return
        self.stalls = 0
        self.t_sim = now
        # 卡頓時截斷，不要讓車瞬移（見 MAX_DT_S）
        dt = min(dt, MAX_DT_S)

        self.x += float(vx) * dt
        self.y += float(vy) * dt

        speed = math.hypot(vx, vy)
        if speed > 1e-6:
            # 車頭朝行進方向。限速轉過去，不要瞬間翻（環線的轉角是 90°）。
            diff = wrap_pi(math.atan2(vy, vx) - self.yaw)
            lim = YAW_RATE * dt
            self.yaw = wrap_pi(self.yaw + max(-lim, min(lim, diff)))
        self._set_pose()


def main():
    # ⚠️ global 一定要在本函式第一次用到這兩個名稱之前宣告，
    #    而下面 add_argument 的 default=MODEL_NAME 就算一次使用 ——
    #    放到 parse_args 之後會變成 SyntaxError（而且 ast.parse 檢查不出來，
    #    那是編譯建符號表時才報的錯，要用 compile() 才驗得到）。
    global MODEL_NAME, TOPIC
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["circle", "line", "aisle", "loop"],
                    default="circle",
                    help="circle 繞圈（預設）／line 直線來回／"
                         "aisle 走道內南北巡迴／loop 依航點繞一圈（讀位置回授）")
    ap.add_argument("--loop", default="",
                    help='loop 模式的航點，世界座標，格式 "x1,y1;x2,y2;…"。'
                         "會依序走並回到第一點。由 gen_warehouse.py 產生的 "
                         "config/warehouse_geometry.yaml 的 target_loop 提供")
    ap.add_argument("--model", default=MODEL_NAME,
                    help="要推哪個目標模型（magenta_target 或 hatchback_target）")
    ap.add_argument("--world", default="warehouse",
                    help="loop 模式讀位置用的世界名稱")
    ap.add_argument("--speed", type=float, default=0.4, help="移動速度 m/s")
    ap.add_argument("--radius", type=float, default=2.5, help="繞圈半徑 m")
    ap.add_argument("--length", type=float, default=3.0, help="直線來回的距離 m")
    ap.add_argument("--y-min", type=float, default=-6.0,
                    help="aisle 模式：走道南端（公尺，世界座標）")
    ap.add_argument("--y-max", type=float, default=6.0,
                    help="aisle 模式：走道北端")
    ap.add_argument("--stop", action="store_true", help="讓它停下來然後結束")
    args = ap.parse_args()

    # 覆寫模組層級的目標名稱。要在任何 Node/publisher 建立之前做，
    # 否則 advertise 出去的還是預設的那個 topic。
    MODEL_NAME = args.model
    TOPIC = f"/model/{MODEL_NAME}/cmd_vel"

    mover = Mover(args.world)

    # ⚠️ 一定要接 SIGTERM：從腳本背景啟動的程序會「忽略 SIGINT」，
    #    只用 Ctrl+C 的訊號收不掉它（2026-09-28 實測留下一個程序一直推車，
    #    害後續測試的目標跑到 11 公尺外）。
    #
    # 改用 set_pose 之後「停止」的意思變了：車不是被速度推著走，所以只要
    # 這個程序結束、不再設 pose，車就停在原地了。halt() 是補一刀把
    # VelocityControl 歸零，免得外掛殘留速度在我們退出後繼續推車。
    def _stop(signum, frame):
        mover.halt()
        time.sleep(0.2)          # 留一點時間讓最後這則真的送出去
        print("\n收到訊號，目標已停止", flush=True)
        sys.exit(0)
    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)

    if args.stop:
        mover.halt()
        time.sleep(0.2)
        print("已停止", flush=True)
        return

    dt = 1.0 / RATE_HZ
    if args.mode == "circle":
        # 速度固定、方向以 ω = v/r 旋轉 → 走出半徑 r 的圓。
        # t=0 時朝世界 +x，向左轉，所以圓心在起點的 +y 側 r 公尺處。
        w = args.speed / args.radius
        period = 2 * math.pi / w
        print(f"目標開始繞圈：速度 {args.speed} m/s、半徑 {args.radius} m"
              f"（繞一圈約 {period:.0f} 秒）", flush=True)
        print("Ctrl+C 停止（會自動讓目標停下來）", flush=True)
        t0 = time.monotonic()
        while True:
            a = w * (time.monotonic() - t0)
            mover.send(args.speed * math.cos(a), args.speed * math.sin(a))
            time.sleep(dt)

    if args.mode == "loop":
        pts = []
        for chunk in args.loop.split(";"):
            chunk = chunk.strip()
            if not chunk:
                continue
            try:
                x, y = (float(v) for v in chunk.split(","))
            except ValueError:
                print(f"✗ --loop 的 「{chunk}」 不是 x,y 格式", flush=True)
                sys.exit(1)
            pts.append((x, y))
        if len(pts) < 3:
            print(f"✗ --loop 至少要 3 個點，現在有 {len(pts)}", flush=True)
            sys.exit(1)

        # 改用 set_pose 之後 loop 不再需要「持續」讀位置 —— 位置是 Mover
        # 自己積分的，到達判定也用它。Mover 建構時讀過一次起點就把 pose 退訂了
        # （每個訂閱都會讓 set_pose 更容易掉回應，見 SET_POSE_TIMEOUT_MS）。

        peri = sum(math.hypot(pts[(i + 1) % len(pts)][0] - pts[i][0],
                              pts[(i + 1) % len(pts)][1] - pts[i][1])
                   for i in range(len(pts)))
        print(f"目標開始繞環線：{len(pts)} 個點、周長 {peri:.1f} m、"
              f"速度 {args.speed} m/s（繞一圈約 {peri / args.speed:.0f} 秒模擬時間）",
              flush=True)
        print(f"  航點：{' → '.join(f'({x:+.1f},{y:+.1f})' for x, y in pts)} → 回第一點",
              flush=True)
        print("Ctrl+C 停止（會自動讓目標停下來）", flush=True)

        # 接到離目前位置最近的那「一段」上，朝它的終點走（見 nearest_leg）
        here = (mover.x, mover.y)
        i, leg_d = nearest_leg(here, pts)
        print(f"  目前在 ({here[0]:+.1f},{here[1]:+.1f})，離環線 {leg_d:.2f} m，"
              f"直接朝第 {i} 點 ({pts[i][0]:+.1f},{pts[i][1]:+.1f}) 走",
              flush=True)
        arrive = max(0.02, mover.step_len(args.speed) * LOOP_ARRIVE_STEPS)
        laps = 0
        while True:
            # ⚠️ 用 mover 自己積出來的位置，不要用 watcher.xy ——
            #    改成 set_pose 之後位置是**我們指定**的，mover 才是權威來源。
            #    watcher.xy 是 Gazebo 回報的，比我們慢一拍；拿它做到達判定
            #    會在每個航點前多走一小段（20 Hz、0.4 m/s 約 2 cm，不多但沒必要）。
            here = (mover.x, mover.y)
            tx, ty = pts[i]
            dx, dy = tx - here[0], ty - here[1]
            d = math.hypot(dx, dy)
            if d < arrive:
                # 精確踩在航點上再轉向，下一段才會走在走道正中間
                mover.snap(tx, ty)
                i = (i + 1) % len(pts)
                if i == 0:
                    laps += 1
                    print(f"  繞完第 {laps} 圈", flush=True)
                continue
            # 單位向量 x 速度。每一拍都重算 → 誤差不會累積，這是用位置回授的全部理由。
            mover.send(args.speed * dx / d, args.speed * dy / d)
            time.sleep(dt)

    if args.mode == "aisle":
        # 倉庫的走道是南北向的，所以只動 y、完全不動 x ——
        # 車子永遠待在走道中線上，不可能橫向撞進貨架。
        #
        # ⚠️ 為什麼可以不讀位置回授（和 circle / line 一樣的理由，但這裡要多想一層）：
        #     走道兩端是開放的橫走道，所以就算計時累積誤差讓它多走一點，
        #     也只是跑進橫走道，**不會撞到貨架**（貨架在左右兩側，x 方向永遠沒動）。
        #     真的在意累積誤差的話再加位置回授，但那要多訂一個 topic。
        #
        # ⚠️ 實際走到的範圍會比設定的小。2026-10-06 實測：設定 y -6~+6（12 m），
        #     實際只走到 -5.9 ~ +4.2（約 10 m），每趟約 25 秒而不是 30 秒。
        #     原因是這裡用 time.monotonic()（牆鐘）計時，但車子是在**模擬時間**裡
        #     移動 —— headless 加上 PX4、Gazebo、取樣迴圈一起跑時 RTF 約 0.83，
        #     所以牆鐘 30 秒只對應約 25 秒的模擬時間。
        #     對「給飛機追的移動目標」來說沒差（車子永遠在走道裡），
        #     要精確範圍就得改成讀位置回授而不是計時。
        #
        # 走道來回正是「車頭方向」最明顯的地方：以前車是橫著側滑過去的
        #     （VelocityControl 轉不了向），現在走到底轉頭要花一秒（YAW_RATE）。
        span = args.y_max - args.y_min
        if span <= 0:
            print("✗ --y-max 必須大於 --y-min", flush=True)
            sys.exit(1)
        leg = span / args.speed
        print(f"目標開始在走道巡迴：y {args.y_min:+.1f} ~ {args.y_max:+.1f}"
              f"（{span:.1f} m）、速度 {args.speed} m/s，每 {leg:.1f} 秒轉向",
              flush=True)
        print("Ctrl+C 停止（會自動讓目標停下來）", flush=True)
        # 車子生成在南端，所以第一段往北（+y）
        vy = args.speed
        t0 = time.monotonic()
        while True:
            if time.monotonic() - t0 >= leg:
                vy = -vy
                t0 = time.monotonic()
            mover.send(0.0, vy)
            time.sleep(dt)

    # 直線來回：沿世界 +x／-x，走完一趟要多久 = 距離 / 速度，時間到就反向。
    # 不讀位置回授 —— 少一個訂閱、少一個會壞的地方。
    leg = args.length / args.speed
    print(f"目標開始來回：速度 {args.speed} m/s、距離 {args.length} m（每 {leg:.1f} 秒轉向）", flush=True)
    print("Ctrl+C 停止（會自動讓目標停下來）", flush=True)
    vx = args.speed
    t0 = time.monotonic()
    while True:
        # 每一拍都要呼叫 send()：位置是一拍一拍積出來的，少一拍車就少走一段。
        if time.monotonic() - t0 >= leg:
            vx = -vx
            t0 = time.monotonic()
        mover.send(vx)
        time.sleep(dt)


if __name__ == "__main__":
    main()

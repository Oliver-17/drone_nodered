#!/usr/bin/env python3
# =============================================================================
#  wait_task.py —— 等 api/status 的 task 做完
#
#  用法（一定要在呼叫服務「之後」才跑）：
#      python3 wait_task.py goto 60
#
#  為什麼需要這支：api/goto 這類服務只負責「開始」，服務回應的時候飛機
#  根本還沒飛（GoTo.srv 的註解寫得很清楚）。用 sleep 猜一個秒數的話，
#  Nav2 繞個路就不夠，給太多又在那邊乾等。
#
#  ⚠️ 判定看的是 task.seq 不是只看 state：上一個同名任務如果已經 done，
#     只看 state 會「等到一個早就結束的任務」立刻返回。這支啟動時先記下
#     當下的 seq（服務已經呼叫過了，所以那就是這次的任務），再等它收尾。
# =============================================================================
import json
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

NAME = sys.argv[1]
TIMEOUT = float(sys.argv[2]) if len(sys.argv) > 2 else 90.0

rclpy.init()
n = Node("wait_task")
state = {}
n.create_subscription(String, "/MAV1/api/status",
                      lambda m: state.update(json.loads(m.data)), 10)

t0 = time.time()
seq = None
last = t0
while time.time() - t0 < TIMEOUT:
    rclpy.spin_once(n, timeout_sec=0.5)
    t = state.get("task") or {}
    if not t:
        continue
    if seq is None:
        # 第一次收到狀態時把 seq 記下來。服務已經呼叫過，所以這就是這次的任務；
        # 名字對不上表示呼叫根本沒成功，早點講比等到逾時好。
        seq = t.get("seq")
        if t.get("name") != NAME:
            print(f"✗ 目前的任務是 {t.get('name')} 不是 {NAME} —— 服務沒呼叫成功？")
            rclpy.try_shutdown()
            sys.exit(1)
    if t.get("seq") == seq and t.get("state") != "running":
        ok = t.get("state") == "done"
        nav = state.get("nav") or {}
        print(f"{'✓' if ok else '✗'} {NAME} → {t.get('state')}"
              f"（{time.time() - t0:.0f} 秒）{t.get('message') or ''}"
              f"{'  nav=' + str(nav.get('state')) if nav else ''}")
        rclpy.try_shutdown()
        sys.exit(0 if ok else 1)
    if time.time() - last > 15:
        last = time.time()
        nav = state.get("nav") or {}
        print(f"  {NAME} 進行中 {time.time() - t0:.0f}s"
              f"  剩餘距離={nav.get('distance_remaining')}")

print(f"✗ 等 {NAME} 逾時（{TIMEOUT:.0f} 秒）：task={state.get('task')}")
rclpy.try_shutdown()
sys.exit(1)

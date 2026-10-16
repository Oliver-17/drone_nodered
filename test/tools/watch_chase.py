#!/usr/bin/env python3
# =============================================================================
#  watch_chase.py —— 量「飛機有沒有跟上車」
#
#  用法（模擬跑起來之後）：
#      python3 watch_chase.py <秒數> [輸出 csv]
#
#  為什麼兩個位置都從 Gazebo 拿，而不是用 api/target：
#      api/target 是**偵測算出來的**位置 —— 拿它當答案等於用考生的答案改考卷。
#      Gazebo 的 pose 是模擬世界的事實，飛機和車都在裡面，這才是獨立的量尺。
#
#  ⚠️ 這支只用 gz.transport，不碰 ROS。混用會出事：gz 的訂閱 callback 會把
#     同步 request 的回應吃掉（move_target.py 的檔頭有完整的實測數字）。
#     要看 api/status 請另開一個 ros2 topic echo。
# =============================================================================
import math
import sys
import time

from gz.msgs10.pose_v_pb2 import Pose_V
from gz.transport13 import Node

DRONE = "x500_cam_down_0"      # PX4 的第 0 架，名字會多一個 _0
CAR = "hatchback_target"

st = {"drone": None, "car": None, "t": None}


def on_pose(msg):
    t = msg.header.stamp.sec + msg.header.stamp.nsec * 1e-9
    for p in msg.pose:
        if p.name == DRONE:
            st["drone"] = (p.position.x, p.position.y, p.position.z)
            st["t"] = t
        elif p.name == CAR:
            st["car"] = (p.position.x, p.position.y)


def main():
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 90.0
    out = sys.argv[2] if len(sys.argv) > 2 else None

    n = Node()
    # 兩個都訂：飛機一直在動（dynamic_pose 有它），車也是
    n.subscribe(Pose_V, "/world/warehouse/dynamic_pose/info", on_pose)
    n.subscribe(Pose_V, "/world/warehouse/pose/info", on_pose)
    end = time.monotonic() + 15.0
    while (st["drone"] is None or st["car"] is None) and time.monotonic() < end:
        time.sleep(0.1)
    if st["drone"] is None or st["car"] is None:
        print(f"✗ 15 秒內沒看到 {DRONE} 或 {CAR} 的 pose")
        print(f"   drone={st['drone']} car={st['car']}")
        return 1

    rows = []
    t0 = time.monotonic()
    print(f"開始記錄 {secs:.0f} 秒  飛機={DRONE}  車={CAR}", flush=True)
    while time.monotonic() - t0 < secs:
        d, c = st["drone"], st["car"]
        if d and c:
            dist = math.hypot(d[0] - c[0], d[1] - c[1])
            rows.append((time.monotonic() - t0, dist, d[0], d[1], d[2], c[0], c[1]))
        time.sleep(0.25)

    if out:
        with open(out, "w") as f:
            f.write("t,dist,dx,dy,dz,cx,cy\n")
            for r in rows:
                f.write(",".join(f"{v:.3f}" for v in r) + "\n")

    ds = [r[1] for r in rows]
    print(f"\n取樣 {len(rows)} 點（{rows[-1][0]:.0f} 秒）")
    print(f"水平距離：最小 {min(ds):.2f} m  中位 {sorted(ds)[len(ds)//2]:.2f} m  "
          f"最大 {max(ds):.2f} m")
    # 分段看趨勢，才知道是「一直沒追到」還是「追到之後穩住了」
    nseg = 6
    seg = max(1, len(rows) // nseg)
    print("\n分段（看有沒有從遠變近再穩住）：")
    for i in range(nseg):
        part = ds[i * seg:(i + 1) * seg]
        if not part:
            continue
        t_lo = rows[i * seg][0]
        t_hi = rows[min(len(rows) - 1, (i + 1) * seg - 1)][0]
        print(f"  {t_lo:5.0f}~{t_hi:5.0f}s  距離 平均 {sum(part)/len(part):5.2f} m"
              f"  最小 {min(part):5.2f}  最大 {max(part):5.2f}")
    # 「跟上了」的定義：離車 3 m 以內（追蹤目標距離是 1.3~1.9 m，給一倍餘裕）
    close = [r for r in rows if r[1] < 3.0]
    print(f"\n距離 < 3 m 的時間佔比：{len(close)/len(rows)*100:.0f}%"
          f"（{len(close)*0.25:.0f} 秒 / {len(rows)*0.25:.0f} 秒）")
    if out:
        print(f"csv：{out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

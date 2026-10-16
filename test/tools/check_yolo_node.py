#!/usr/bin/env python3
# =============================================================================
#  check_yolo_node.py —— 不用飛就驗 yolo_detector_node 寫對了沒
#
#  用法（要在 YOLO 的 venv 裡，而且節點要先跑起來）：
#      source ~/ros2_ws/src/drone_vision/scripts/yolo_env.sh
#      ros2 run drone_vision yolo_detector_node.py --ros-args \
#          -p weights:=$HOME/yolo_data/weights/yolov8s-visdrone.pt &
#      python3 check_yolo_node.py ~/yolo_data/frames_hover
#
#  做法：把存好的影像當成相機發出去，收節點發回來的框，
#       再和「這支腳本自己直接呼叫 ultralytics」的結果逐幀比對。
#
#  為什麼這樣驗得有意義：
#      離線那條路是已經驗過的（eval_detect.py 用幾何 ground truth 證實
#      FULL 4/4、零誤檢）。節點只要輸出和它一致，就表示節點沒有把事情做壞 ——
#      cv_bridge 的轉換、選框、座標換算、訊息欄位，全部一次驗完。
#      對不上的話是節點的問題，不是模型的問題，範圍一下子就縮小了。
#
#  ⚠️ 配對用時間戳，不是順序。節點的 QoS 是 BEST_EFFORT（影像是串流，
#     推論跟不上就該丟），所以收到的框會比發出去的影像少 —— 那是正常的。
#     用順序配對會整個錯開，而且錯得很安靜。
# =============================================================================
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray


class Checker(Node):
    def __init__(self, files):
        super().__init__("check_yolo_node")
        self.files = files
        self.br = CvBridge()
        self.got = {}                       # 時間戳(ns) -> (cx, cy, w, h, score, cls)
        self.pub = self.create_publisher(Image, "/camera/image",
                                         qos_profile_sensor_data)
        self.create_subscription(Detection2DArray, "detections",
                                 self._on_det, qos_profile_sensor_data)

    def _on_det(self, msg):
        key = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if msg.detections:
            d = msg.detections[0]
            self.got[key] = (d.bbox.center.position.x, d.bbox.center.position.y,
                             d.bbox.size_x, d.bbox.size_y,
                             d.results[0].hypothesis.score if d.results else 0.0,
                             d.results[0].hypothesis.class_id if d.results else "")
        else:
            self.got[key] = None            # 空陣列也要記 —— 「沒偵測到」也是答案

    def run(self, hz=5.0):
        """一張一張發，每張發完等它的框回來再發下一張。

        慢慢發是刻意的：這支要驗「每一幀的輸出對不對」，不是驗吞吐量。
        用相機的 15 Hz 發的話推論跟不上會丟幀，就比不到全部 118 張了。
        """
        sent = {}
        for i, f in enumerate(self.files):
            img = cv2.imread(str(f))
            msg = self.br.cv2_to_imgmsg(img, encoding="bgr8")
            now = self.get_clock().now().to_msg()
            msg.header.stamp = now
            msg.header.frame_id = "camera_link"
            key = now.sec * 10**9 + now.nanosec
            sent[key] = f.name
            self.pub.publish(msg)
            # 等這一幀的結果（最多 5 秒 —— 第一幀包含載權重和暖機）
            end = time.monotonic() + 5.0
            while key not in self.got and time.monotonic() < end:
                rclpy.spin_once(self, timeout_sec=0.05)
            if (i + 1) % 25 == 0:
                print(f"  已送 {i + 1}/{len(self.files)} 張", flush=True)
        return sent


def main():
    fr = Path(sys.argv[1]).expanduser()
    files = sorted(fr.glob("f*.jpg"))
    if not files:
        print(f"✗ {fr} 裡沒有 f*.jpg")
        return 1
    print(f"素材：{len(files)} 張  {fr}")

    rclpy.init()
    node = Checker(files)
    print("等節點就緒…")
    end = time.monotonic() + 30.0
    while node.count_subscribers("/camera/image") == 0 and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.count_subscribers("/camera/image") == 0:
        print("✗ 沒有人訂閱 /camera/image —— 節點沒跑起來？")
        rclpy.try_shutdown()
        return 1
    print("節點已連上，開始送圖…")
    sent = node.run()
    # 再轉一下，收尾的訊息可能還在路上
    end = time.monotonic() + 2.0
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)

    # --- 離線對照：直接呼叫 ultralytics，用節點裡同一套選框規則 ---------------
    from ultralytics import YOLO
    import os
    w = os.environ.get("YOLO_W",
                       str(Path.home() / "yolo_data/weights/yolov8s-visdrone.pt"))
    M = YOLO(w)
    M.to("cuda")
    veh = {i for i, n in M.names.items() if n in ("car", "van", "truck", "bus")}

    name2key = {v: k for k, v in sent.items()}
    agree = mismatch = missing = 0
    miss_names = []
    both_none = 0
    worst = 0.0
    bad = []
    for f in files:
        r = M.predict(str(f), verbose=False, device=0, conf=0.25)[0]
        best = None
        area0 = 0.0
        if r.boxes is not None and len(r.boxes):
            for b, k, c in zip(r.boxes.xyxy.cpu().numpy(),
                               r.boxes.cls.cpu().numpy().astype(int),
                               r.boxes.conf.cpu().numpy()):
                if k not in veh:
                    continue
                a = float((b[2] - b[0]) * (b[3] - b[1]))
                if a > area0:
                    area0, best = a, (b, float(c))
        key = name2key.get(f.name)
        if key is None or key not in node.got:
            missing += 1
            miss_names.append(f.name)
            continue
        g = node.got[key]
        if best is None and g is None:
            both_none += 1
            agree += 1
            continue
        if (best is None) != (g is None):
            mismatch += 1
            bad.append((f.name, "一邊有一邊沒有",
                        "離線無" if best is None else "節點無"))
            continue
        b, c = best
        want = ((b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0, b[2] - b[0], b[3] - b[1])
        d = max(abs(want[j] - g[j]) for j in range(4))
        worst = max(worst, d)
        if d < 0.51 and abs(c - g[4]) < 0.01:
            agree += 1
        else:
            mismatch += 1
            bad.append((f.name, f"框差 {d:.2f} px", f"信心 {c:.3f} vs {g[4]:.3f}"))

    print(f"\n=== 節點輸出 vs 離線推論（{len(files)} 張）===")
    print(f"  一致      {agree}")
    print(f"    其中兩邊都「沒偵測到」 {both_none}")
    print(f"  不一致    {mismatch}")
    print(f"  沒收到框  {missing}"
          f"{'：' + ', '.join(miss_names[:10]) if miss_names else ''}")
    print(f"  框座標最大誤差 {worst:.3f} px")
    for nm, a, b in bad[:10]:
        print(f"    🔴 {nm}  {a}  {b}")
    rclpy.try_shutdown()
    return 0 if (mismatch == 0 and missing == 0) else 1


if __name__ == "__main__":
    sys.exit(main())

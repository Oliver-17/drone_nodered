#!/usr/bin/env python3
# =============================================================================
#  grab_frames.py —— 從模擬抓機載影像存成 jpg，給離線的視覺實驗用
#
#  用法：
#      python3 grab_frames.py <輸出目錄> <每幾幀存一張> <最多存幾張>
#
#  為什麼要「存下來離線跑」而不是直接在模擬裡接偵測器：
#      換權重、改 prompt、調門檻都不用再飛一次。飛一趟 3 分鐘，
#      離線重跑一次 10 秒，而且每次比較的是**同一批影像**，才比得準。
#
#  順便把顏色偵測的框存成 meta.json 當基準線 —— 目標是洋紅方塊時它 100% 準，
#  可以直接當 ground truth 量別的偵測器。目標換成真車之後顏色偵測會全空，
#  那時 meta.json 裡的 color_boxes 就只是「這一幀沒有基準」的意思。
#
#  影像和框用 ExactTime 同步：color_detector_node.py 是
#  `out.header = msg.header` 原封不動複製，兩邊時間戳完全相同。
#  用 ApproximateTime 反而會偶爾把 A 幀的影像配到 B 幀的框，
#  而那種錯誤在後續分析裡完全看不出來。
#
#  ---------------------------------------------------------------------------
#  為什麼要連飛機位姿一起存（2026-10-15 加）
#  ---------------------------------------------------------------------------
#  目標換成真車以後，顏色偵測全空 —— 就沒有任何 ground truth 了。
#  但車是**靜止**的而且位置我們知道，所以只要同時存下飛機的位置和姿態，
#  就能離線「純用幾何」算出每一幀車該不該在畫面裡（eval_detect.py 做這件事），
#  完全不經過任何偵測器。這才有辦法算出真正的偵測率，
#  而不是用「YOLO 沒抓到的那些我看起來也沒車」這種循環論證。
#
#  ⚠️ 位姿**不能**和影像做時間同步，只能存「收到這一幀時的最新值」：
#     PX4 的時間戳來自飛控自己的時鐘，和 Gazebo 相機的時間戳不是同一條，
#     ExactTime 永遠配不到，ApproximateTime 則會為了配對而丟掉大量影像。
#     誤差估計：位置 50 Hz 以上、抓圖約 1.9 Hz，最差差一個位置週期 20 ms，
#     飛機 2 m/s → 4 cm。投影到 480 px 高的畫面上不到 5 px，不影響
#     「在不在畫面裡」的判斷（真正在邊界上的那幾幀本來就該另外看）。
# =============================================================================
import json
import sys
from pathlib import Path

import cv2
import message_filters
import rclpy
from cv_bridge import CvBridge
from px4_msgs.msg import VehicleAttitude, VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import CameraInfo, Image
from vision_msgs.msg import Detection2DArray


class Grab(Node):
    def __init__(self, out: Path, every: int, limit: int, ns: str = "MAV1"):
        super().__init__("grab_frames")
        self.out, self.every, self.limit = out, every, limit
        out.mkdir(parents=True, exist_ok=True)
        self.br = CvBridge()
        self.n = self.saved = 0
        self.meta = []
        self.done = False          # ← 不在 callback 裡呼叫 shutdown（Humble 的 spin 不會因此跳出）
        self.lpos = None           # 最新的 VehicleLocalPosition（NED，原點在起飛點）
        self.att = None            # 最新的 VehicleAttitude（四元數 w,x,y,z：機體 FRD → NED）
        self.cam = None            # 相機內參，只存第一則（整趟不會變）

        img = message_filters.Subscriber(self, Image, "/camera/image",
                                         qos_profile=qos_profile_sensor_data)
        det = message_filters.Subscriber(self, Detection2DArray, "/detections",
                                         qos_profile=qos_profile_sensor_data)
        message_filters.TimeSynchronizer([img, det], 30).registerCallback(self.cb)

        # PX4 的 topic 一律 BEST_EFFORT + VOLATILE，用預設的 RELIABLE 會收不到。
        # ⚠️ 位置是 `_v1` 後綴（PX4 v1.17 的逐訊息版本化），姿態**沒有**後綴 ——
        #    兩條不一致，照 target_world_node.py 的寫法抄，不要自己統一。
        px4_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT,
                             history=HistoryPolicy.KEEP_LAST,
                             durability=DurabilityPolicy.VOLATILE)
        self.create_subscription(VehicleLocalPosition,
                                 f"/{ns}/fmu/out/vehicle_local_position_v1",
                                 lambda m: setattr(self, "lpos", m), px4_qos)
        self.create_subscription(VehicleAttitude, f"/{ns}/fmu/out/vehicle_attitude",
                                 lambda m: setattr(self, "att", m), px4_qos)
        # 內參自己存一份，離線就不用從 SDF 的 horizontal_fov 反推（那會多一個猜測環節）
        self.create_subscription(CameraInfo, "/camera/camera_info",
                                 self._on_info, qos_profile_sensor_data)
        self.get_logger().info(f"開始抓圖 → {out}（每 {every} 幀存一張，上限 {limit}）")

    def _on_info(self, msg):
        if self.cam is None:
            k = msg.k          # 3x3 內參矩陣，列優先：[fx 0 cx; 0 fy cy; 0 0 1]
            self.cam = {"w": msg.width, "h": msg.height,
                        "fx": k[0], "fy": k[4], "cx": k[2], "cy": k[5]}
            self.get_logger().info(f"相機內參：{self.cam}")

    def _pose(self):
        """把當下最新的位姿整理成可以寫進 json 的樣子，沒有就回 None。

        xy_valid / z_valid 要檢查：EKF 還沒收斂時 x/y/z 是沒有意義的數字，
        而不是 NaN —— 不檢查的話會安靜地算出一堆錯的 ground truth。
        """
        lp, att = self.lpos, self.att
        if lp is None or att is None or not lp.xy_valid or not lp.z_valid:
            return None
        return {"x": float(lp.x), "y": float(lp.y), "z": float(lp.z),
                "q": [float(v) for v in att.q]}

    def cb(self, img_msg, det_msg):
        if self.done:
            return
        self.n += 1
        if self.n % self.every:
            return
        name = f"f{self.saved:04d}.jpg"
        frame = self.br.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        cv2.imwrite(str(self.out / name), frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
        self.meta.append({
            "file": name, "w": img_msg.width, "h": img_msg.height,
            "color_boxes": [
                {"cx": d.bbox.center.position.x, "cy": d.bbox.center.position.y,
                 "w": d.bbox.size_x, "h": d.bbox.size_y,
                 "score": d.results[0].hypothesis.score if d.results else 0.0}
                for d in det_msg.detections],
            "pose": self._pose(),
        })
        self.saved += 1
        if self.saved % 25 == 0:
            hit = sum(1 for m in self.meta if m["color_boxes"])
            nop = sum(1 for m in self.meta if m["pose"] is None)
            self.get_logger().info(
                f"已存 {self.saved} 張（顏色偵測命中 {hit}、無位姿 {nop}）")
        if self.saved >= self.limit:
            self.done = True

    def finish(self):
        # 包成 dict 而不是直接寫 frames 那個 list：內參是整趟共用的一份，
        # 塞進每一幀會重複 160 次。舊的 meta.json 是 list，eval 端兩種都要認。
        (self.out / "meta.json").write_text(json.dumps(
            {"camera": self.cam, "frames": self.meta}, indent=1))
        hit = sum(1 for m in self.meta if m["color_boxes"])
        nop = sum(1 for m in self.meta if m["pose"] is None)
        self.get_logger().info(
            f"收工：{self.saved} 張，顏色偵測命中 {hit}"
            f"（{hit / max(1, self.saved) * 100:.0f}%）、無位姿 {nop} 張"
            f" → {self.out}/meta.json")
        if self.cam is None:
            self.get_logger().warn("沒收到 /camera/camera_info —— 離線算幾何時要自己給內參")


def main():
    out, every, limit = Path(sys.argv[1]), int(sys.argv[2]), int(sys.argv[3])
    rclpy.init()
    node = Grab(out, every, limit)
    try:
        # 自己轉迴圈而不用 rclpy.spin()：要能在存滿之後乾淨地跳出來。
        # 在 callback 裡呼叫 rclpy.shutdown() 的話 spin() 不會返回，程式會掛著，
        # 外層 shell 的收尾（kill 行程群組）就永遠等不到它結束。
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.finish()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()

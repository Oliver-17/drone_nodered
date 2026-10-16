#!/usr/bin/env python3
# =============================================================================
#  eval_detect.py —— 用幾何當 ground truth，量偵測器真正的偵測率
#
#  用法（要先進 yolo venv）：
#      source ~/ros2_ws/src/drone_vision/scripts/yolo_env.sh
#      python3 test/tools/eval_detect.py ~/yolo_data/frames_car
#
#  ---------------------------------------------------------------------------
#  為什麼需要這支
#  ---------------------------------------------------------------------------
#  目標換成真車以後顏色偵測全空，就沒有 ground truth 了。於是「YOLO 160 張
#  只抓到 5 張」沒辦法解讀 —— 是偵測爛，還是車根本只有 5 張在畫面裡？
#  用「我看了幾張沒抓到的圖，裡面也沒車」來回答是循環論證：偵測器漏掉的
#  正是最難看出來的那幾張（車在邊角、車很小），抽樣目視剛好會把它們藏起來。
#
#  但車是**靜止**的，位置我們知道（世界 ENU (0, -6)），而 grab_frames.py
#  現在會把每一幀的飛機位姿一起存下來。那就可以純用幾何算出「這一幀車該不該
#  在畫面裡」，完全不經過任何偵測器 —— 這才是獨立的 ground truth。
#
#  ---------------------------------------------------------------------------
#  投影是 target_world_node.py 的反向運算
#  ---------------------------------------------------------------------------
#  那支做「畫面上的點 → 世界座標」，這支做「世界座標 → 畫面上的點」。
#  四次轉換每一步都原封不動倒過來（見 _project），所以符號錯了會被
#  底下的 _selftest 抓到：隨機取點投影成像素、再用那支的正向公式算回世界，
#  對不回原點就是有一步寫錯。座標轉換寫錯最可怕的地方是結果看起來很合理，
#  所以這個自我檢查不是裝飾，是這支腳本可信的唯一理由。
#
#  ---------------------------------------------------------------------------
#  判定分三級，不是「看得到 / 看不到」
#  ---------------------------------------------------------------------------
#  車是 0.6 x 0.32 x 0.24 m 的盒子不是一個點。把 8 個角投影出來取包圍框，
#  再和畫面矩形求交集，得到「露出多少比例」：
#      >= 0.8  FULL —— 整台車都在畫面裡，這些**必須**抓到
#      >= 0.2  EDGE —— 卡在畫面邊緣只露一部分，抓不到可以接受
#      <  0.2  OUT  —— 不在畫面裡，抓到才是誤檢
#  只用中心點判定會把「只露一個車尾」算成完整可見，然後冤枉偵測器。
#
#  ⚠️ 一定要算貨架遮擋：飛機繞環線時大半時間在 x=5.6 那條走道，而車在 x=0,
#     中間隔著 x=2.8、高 7 m 的貨架。純看視角的話這些幀全都「在視野內」，
#     不扣掉遮擋就會多出一大堆假的漏檢，整張報表就廢了。
#
#  ---------------------------------------------------------------------------
#  ⚠️ GT 框只能用來判定「在不在畫面裡」，不要拿它的 IoU 當位置精度
#  ---------------------------------------------------------------------------
#  GT 框是「外接長方體的 8 個角投影出來的包圍框」，它**比車的實際輪廓大**，
#  而且近距離時會往畫面下方（離飛機近的那一側）撐出去：
#  盒子的近角和遠角在近距離投影得很開，而真車的車頂是圓的、車身填不滿盒子。
#  2026-10-15 實測，和偵測框比對（離車距離 → 框中心差）：
#       3.8 / 4.0 m  →   3.6 / 17.8 px
#       2.0 / 2.1 m  →  15.1 / 19.8 px
#       0.64 / 0.69 m →  34.1 / 29.3 px      ← 越近差越大，IoU 掉到 0.25
#  所以 IoU 只有 0.16~0.47 是這個效應，不是偵測框歪也不是座標算錯
#  （座標另外驗過：飛機落點離搜尋環線中位 4.8 cm、高度 3.003 m）。
#  判定用「露出比例」就不受影響 —— 框算大一點只會讓邊界幀的分級保守一點。
# =============================================================================
import json
import math
import sys
from pathlib import Path

import cv2
import numpy as np

# --- 場景與機體常數 ---------------------------------------------------------
# 都是從檔案裡抄來的真值，不是估的。改場景或換機體要跟著改：
#   spawn        config/warehouse_geometry.yaml 的 spawn
#   車的位置     gz/worlds/warehouse.sdf 裡 hatchback_target 的 pose
#   車的尺寸     gz/models/hatchback_target/model.sdf（縮放後 0.600 x 0.321 x 0.235）
#   貨架         scripts/gen_warehouse.py 的 SHELF_* 常數
#   相機俯角     gz/models/x500_cam_down/model.sdf 的 camera_link pose（1.1694 rad = 67°）
#   相機安裝     同上的 .12 0 -.18（SDF 是 FLU，往下為負）→ FRD 要把 z 反號成 +0.18
SPAWN_ENU = (-14.0, -9.0)        # PX4 本地 NED 的原點（飛機生成點）
TARGET_ENU = (0.0, -6.0)         # 車的位置
TARGET_SIZE = (0.600, 0.321, 0.235)   # 長(x) 寬(y) 高(z)，世界軸向（車頭朝 +x）
SHELF_CENTERS_X = (-8.4, -2.8, 2.8, 8.4)
SHELF_HX, SHELF_HY, SHELF_TOP = 0.8, 7.0, 7.0   # 半深、半長、頂高

CAM_PITCH = 1.1694               # rad，往下為正
CAM_MOUNT_FRD = (0.12, 0.0, 0.18)   # 機體 FRD（x 前、y 右、z 下）

FULL_T, EDGE_T = 0.8, 0.2        # 可見比例的兩道門檻
CONF = 0.25                      # YOLO 信心門檻
VEH = ("car", "van", "truck", "bus")   # 算「抓到載具」的類別（不要求分對車型）
WEIGHTS = ("yolov8s-visdrone", "yolov8l-visdrone")


def enu_to_ned(ex, ey, ez):
    """Gazebo 世界 ENU（x 東、y 北、z 上）→ PX4 本地 NED（x 北、y 東、z 下）。

    原點差：PX4 的 (0,0) 在飛機生成點，不是地圖原點。
    """
    return (ey - SPAWN_ENU[1], ex - SPAWN_ENU[0], -ez)


def rot(q, v):
    """用四元數 q=(w,x,y,z) 旋轉 v。和 target_world_node._rotate 同一份公式。"""
    w, x, y, z = (float(c) for c in q)
    vx, vy, vz = (float(c) for c in v)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (vx + w * tx + (y * tz - z * ty),
            vy + w * ty + (z * tx - x * tz),
            vz + w * tz + (x * ty - y * tx))


def conj(q):
    return (q[0], -q[1], -q[2], -q[3])


def cam_origin_ned(pose):
    """相機光心在 NED 的位置：飛機位置 + 旋轉過的安裝位移。"""
    m = rot(pose["q"], CAM_MOUNT_FRD)
    return (pose["x"] + m[0], pose["y"] + m[1], pose["z"] + m[2])


def _project(pose, p_ned, cam):
    """世界 NED 的一點 → 像素 (u, v)，外加 depth（光學系的 z，正值表示在前方）。

    這是 target_world_node._on_ray 的四步反過來走，順序也相反：
        NED → 機體 FRD → 相機 FLU → 相機光學系 → 像素
    """
    o = cam_origin_ned(pose)
    d = (p_ned[0] - o[0], p_ned[1] - o[1], p_ned[2] - o[2])

    # 1) NED → 機體 FRD：q 代表「FRD → NED」，所以反向要用共軛
    f = rot(conj(pose["q"]), d)

    # 2) FRD → 相機 FLU。正向是
    #        fx = c*cx + s*cz ; fy = -cy ; fz = s*cx - c*cz
    #    那個 2x2 矩陣 [[c, s], [s, -c]] 的行列式是 -1 而且**自逆**，
    #    所以反解長得和正解一模一樣（不是手滑抄錯）。
    c, s = math.cos(CAM_PITCH), math.sin(CAM_PITCH)
    ux = c * f[0] + s * f[2]
    uy = -f[1]
    uz = s * f[0] - c * f[2]

    # 3) 相機 FLU（x 前、y 左、z 上）→ 光學系（x 右、y 下、z 前）
    rx, ry, rz = -uy, -uz, ux

    # 4) 針孔投影。rz <= 0 是在相機背後，投影出來的像素沒有意義
    if rz <= 1e-6:
        return None, rz
    return (cam["fx"] * rx / rz + cam["cx"],
            cam["fy"] * ry / rz + cam["cy"]), rz


def target_corners_ned():
    """車的 8 個角，世界 NED。車身沿世界軸向（VelocityControl 的 angular.z 無效，
    模型只平移不轉身，所以車頭永遠朝 +x，不用考慮車身 yaw）。"""
    lx, ly, lz = (v / 2.0 for v in TARGET_SIZE)
    cx, cy = TARGET_ENU
    out = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            # 車的原點在**車輪著地面**（hatchback 的 model.sdf 這樣擺），
            # 所以 z 從 0 到車高，不是 ±半高
            for sz in (0.0, TARGET_SIZE[2]):
                out.append(enu_to_ned(cx + sx * lx, cy + sy * ly, sz))
    return out


def visible_ratio(pose, cam):
    """車露在畫面裡的面積比例（0~1）。"""
    pts = []
    for p in target_corners_ned():
        uv, rz = _project(pose, p, cam)
        if uv is None:
            return 0.0          # 有角在相機背後就當看不到（近距離才會發生）
        pts.append(uv)
    us = [p[0] for p in pts]
    vs = [p[1] for p in pts]
    x0, x1, y0, y1 = min(us), max(us), min(vs), max(vs)
    area = max(1e-9, (x1 - x0) * (y1 - y0))
    # 和畫面矩形的交集
    ix = max(0.0, min(x1, cam["w"]) - max(x0, 0.0))
    iy = max(0.0, min(y1, cam["h"]) - max(y0, 0.0))
    return (ix * iy) / area


def occluded(pose, cam):
    """相機到車中心的視線有沒有被貨架擋住。

    線段 vs AABB 用 slab method：對 x/y/z 三個軸各算出線段進入和離開那一層
    板子的參數區間，三個區間有交集就是穿過盒子。
    """
    o = cam_origin_ned(pose)
    cx, cy = TARGET_ENU
    tgt = enu_to_ned(cx, cy, TARGET_SIZE[2] / 2.0)
    d = (tgt[0] - o[0], tgt[1] - o[1], tgt[2] - o[2])
    for sx in SHELF_CENTERS_X:
        # 貨架 AABB，先寫成 ENU 再換成 NED 的 min/max
        lo = enu_to_ned(sx - SHELF_HX, -SHELF_HY, 0.0)
        hi = enu_to_ned(sx + SHELF_HX, SHELF_HY, SHELF_TOP)
        bmin = tuple(min(a, b) for a, b in zip(lo, hi))
        bmax = tuple(max(a, b) for a, b in zip(lo, hi))
        t0, t1 = 0.0, 1.0
        for i in range(3):
            if abs(d[i]) < 1e-9:
                # 這個軸上不動：只要起點就在板子外面，整條線段都不可能穿過
                if o[i] < bmin[i] or o[i] > bmax[i]:
                    t0 = 2.0
                    break
                continue
            a = (bmin[i] - o[i]) / d[i]
            b = (bmax[i] - o[i]) / d[i]
            t0 = max(t0, min(a, b))
            t1 = min(t1, max(a, b))
        if t0 <= t1:
            return True
    return False


def _selftest(cam):
    """把反向投影和 target_world_node 的正向公式對撞，驗證沒有符號寫錯。

    流程：給一個位姿和地面上的一點 → 投影成像素 → 把像素當成偵測結果，
    用那支的正向公式（光學→FLU→FRD→NED→和地面交會）算回世界座標。
    對得回去才表示兩邊是真的互逆。
    """
    # 挑一個「有傾斜」的姿態：水平姿態會讓很多符號錯誤剛好互相抵銷而驗不出來
    roll, pitch, yaw = math.radians(4), math.radians(-7), math.radians(130)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy_, sy_ = math.cos(yaw / 2), math.sin(yaw / 2)
    q = (cr * cp * cy_ + sr * sp * sy_, sr * cp * cy_ - cr * sp * sy_,
         cr * sp * cy_ + sr * cp * sy_, cr * cp * sy_ - sr * sp * cy_)
    pose = {"x": 3.0, "y": 14.0, "z": -3.0, "q": q}

    worst = 0.0
    for ex, ey in ((0.0, -6.0), (0.4, -6.3), (-0.5, -5.6), (0.2, -5.9)):
        h = TARGET_SIZE[2] / 2.0
        p = enu_to_ned(ex, ey, h)
        uv, rz = _project(pose, p, cam)
        if uv is None:
            continue
        # --- 以下完全照 target_world_node._on_ray 的正向算回去 -----------------
        rx = (uv[0] - cam["cx"]) / cam["fx"]
        ry = (uv[1] - cam["cy"]) / cam["fy"]
        n = math.sqrt(rx * rx + ry * ry + 1.0)
        r = (rx / n, ry / n, 1.0 / n)          # 光學系單位射線
        ux, uy, uz = r[2], -r[0], -r[1]        # 光學 → 相機 FLU
        c, s = math.cos(CAM_PITCH), math.sin(CAM_PITCH)
        fx = c * ux + s * uz
        fy = -uy
        fz = -(-s * ux + c * uz)
        nx, ny, nz = rot(q, (fx, fy, fz))
        o = cam_origin_ned(pose)
        t = (-h - o[2]) / nz                   # 和「車身中間高度」的平面交會
        got = (o[0] + nx * t, o[1] + ny * t)
        worst = max(worst, math.hypot(got[0] - p[0], got[1] - p[1]))
    return worst


def load(fr: Path):
    m = json.loads((fr / "meta.json").read_text())
    if isinstance(m, list):     # 舊格式：只有一個 frames list，沒有相機內參
        return None, m
    return m.get("camera"), m["frames"]


def main():
    global TARGET_ENU
    fr = Path(sys.argv[1]).expanduser()
    # 第二個參數：故意把車的位置往 y 平移這麼多公尺。預設 0。
    #
    # 這不是功能，是給人檢查用的「證偽開關」：ground truth 如果真的是
    # 從幾何算出來的，那把車的位置說錯 2 公尺，FULL 那一欄的偵測率就該崩掉
    # （幾何會指到空地，而 YOLO 的結果一個字都沒變）。
    # 要是改了位置結果卻一樣，那就表示 GT 根本沒在算幾何 —— 這份報表要丟掉。
    shift = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
    # 證偽模式的圖用不同檔名。共用檔名的話，跑完證偽就把正常模式的對照圖
    # 蓋掉了，而圖上看不出是哪個模式 —— 下次看圖的人會以為偵測框歪掉。
    tag = "" if not shift else f"_shift{shift:+.1f}"
    if shift:
        TARGET_ENU = (TARGET_ENU[0], TARGET_ENU[1] + shift)
        print(f"⚠️ 證偽模式：車的位置故意說成 {TARGET_ENU}（平移 {shift:+.1f} m）")
        print("   如果底下 FULL 那一欄沒有崩掉，代表 GT 不是算出來的 —— 報表不可信\n")
    cam, frames = load(fr)
    if cam is None:
        # 從 SDF 的 horizontal_fov 反推，給舊的資料集留一條路。
        # fx = (w/2) / tan(fov/2)；Gazebo 的像素是方的所以 fy = fx。
        w, h, fov = 640, 480, 1.204
        f = (w / 2.0) / math.tan(fov / 2.0)
        cam = {"w": w, "h": h, "fx": f, "fy": f, "cx": w / 2.0, "cy": h / 2.0}
        print(f"⚠️ meta.json 沒有相機內參（舊格式），用 SDF 的 fov={fov} 反推：fx={f:.1f}")
    print(f"素材：{len(frames)} 張  {fr}")
    print(f"相機：{cam['w']}x{cam['h']}  fx={cam['fx']:.1f} fy={cam['fy']:.1f} "
          f"cx={cam['cx']:.1f} cy={cam['cy']:.1f}  俯角 {math.degrees(CAM_PITCH):.0f}°")

    err = _selftest(cam)
    print(f"\n自我檢查（投影→反投影繞一圈回到原點）：最大誤差 {err * 1000:.3f} mm")
    if err > 1e-3:
        print("🔴 誤差太大，投影有一步寫錯了 —— 下面的 ground truth 不可信，停。")
        return 1
    print("   ✅ 和 target_world_node 的正向公式互逆")

    # --- ① 幾何 ground truth（完全不碰偵測器）--------------------------------
    gt = {}
    nopose = []
    for m in frames:
        if not m.get("pose"):
            nopose.append(m["file"])
            continue
        vis = visible_ratio(m["pose"], cam)
        occ = occluded(m["pose"], cam)
        if occ or vis < EDGE_T:
            lab = "OUT"
        elif vis >= FULL_T:
            lab = "FULL"
        else:
            lab = "EDGE"
        gt[m["file"]] = (lab, vis, occ)
    if nopose:
        print(f"\n⚠️ {len(nopose)} 張沒有位姿（EKF 還沒收斂？），不列入統計")
    n_full = sum(1 for v in gt.values() if v[0] == "FULL")
    n_edge = sum(1 for v in gt.values() if v[0] == "EDGE")
    n_occ = sum(1 for v in gt.values() if v[2])
    print(f"\n=== 幾何 ground truth（{len(gt)} 張）===")
    print(f"  FULL 整台車在畫面裡       {n_full} 張")
    print(f"  EDGE 卡在畫面邊緣         {n_edge} 張")
    print(f"  OUT  不在畫面裡           {len(gt) - n_full - n_edge} 張"
          f"（其中被貨架擋住 {n_occ} 張）")

    # --- ② 跑偵測器，和 GT 對帳 ----------------------------------------------
    from ultralytics import YOLO
    W = Path.home() / "yolo_data/weights"
    files = sorted(fr.glob("f*.jpg"))
    results = {}
    for wn in WEIGHTS:
        M = YOLO(str(W / f"{wn}.pt"))
        M.to("cuda")
        veh = {i for i, n in M.names.items() if n in VEH}
        hit = {}
        for f in files:
            r = M.predict(str(f), verbose=False, device=0, conf=CONF)[0]
            cand = [(float(c), b, M.names[int(k)]) for b, k, c in
                    zip(r.boxes.xyxy.cpu().numpy(), r.boxes.cls.cpu().numpy(),
                        r.boxes.conf.cpu().numpy()) if int(k) in veh]
            if cand:
                cand.sort(reverse=True, key=lambda t: t[0])
                hit[f.name] = cand
        results[wn] = hit

        tab = {k: [0, 0] for k in ("FULL", "EDGE", "OUT")}   # [有偵測, 總數]
        for name, (lab, _vis, _occ) in gt.items():
            tab[lab][1] += 1
            if name in hit:
                tab[lab][0] += 1
        print(f"\n--- {wn}（conf>={CONF}，類別 {'/'.join(VEH)} 都算）---")
        for lab in ("FULL", "EDGE", "OUT"):
            d, t = tab[lab]
            rate = f"{d / t * 100:5.1f}%" if t else "    -"
            note = {"FULL": "← 偵測率就是這個數字", "EDGE": "（抓不到可接受）",
                    "OUT": "← 這裡有數字就是誤檢"}[lab]
            print(f"  {lab:4s} {d:3d}/{t:3d}  {rate}  {note}")
        cs = [v[0][0] for v in hit.values()]
        if cs:
            print(f"  信心值：最低 {min(cs):.2f}  中位 {float(np.median(cs)):.2f}  "
                  f"最高 {max(cs):.2f}")
        miss = [n for n, (lab, _v, _o) in gt.items() if lab == "FULL" and n not in hit]
        fp = [n for n, (lab, _v, _o) in gt.items() if lab == "OUT" and n in hit]
        if miss:
            print(f"  🔴 漏檢（FULL 卻沒抓到）：{', '.join(miss[:12])}"
                  f"{' …' if len(miss) > 12 else ''}")
        if fp:
            print(f"  🔴 誤檢（OUT 卻抓到）：{', '.join(fp[:12])}"
                  f"{' …' if len(fp) > 12 else ''}")

    # --- ③ 把每一幀的 GT 和偵測結果列出來（只列車有進畫面的）-------------------
    wn = max(results, key=lambda k: len(results[k]))
    print(f"\n=== 逐幀對照（車進畫面的那些，偵測結果用 {wn}）===")
    rows = [(n, v) for n, v in sorted(gt.items()) if v[0] != "OUT"]
    if not rows:
        print("  幾何算起來整趟都沒有車進畫面 —— 檢查環線設定或車的位置")
    for n, (lab, vis, _occ) in rows:
        h = results[wn].get(n)
        mark = f"抓到 {h[0][2]} {h[0][0]:.2f}" if h else "沒抓到"
        print(f"  {n}  {lab:4s} 露出 {vis * 100:5.1f}%  {mark}")

    # --- ④ 存對照圖：GT 框（藍）和偵測框（綠）畫在一起 -------------------------
    # 兩個框畫在同一張圖上才看得出偵測器是「沒看到」還是「框歪了」。
    # 中文在 cv2.putText 會變成亂碼，所以圖上的標註一律英文。
    picks = [n for n, _ in rows][:9]
    if picks:
        tiles = []
        for n in picks:
            im = cv2.imread(str(fr / n))
            pose = next(m["pose"] for m in frames if m["file"] == n)
            uvs = [uv for uv, _ in (_project(pose, p, cam)
                                    for p in target_corners_ned()) if uv]
            if uvs:
                x0 = int(min(u for u, _ in uvs)); x1 = int(max(u for u, _ in uvs))
                y0 = int(min(v for _, v in uvs)); y1 = int(max(v for _, v in uvs))
                cv2.rectangle(im, (x0, y0), (x1, y1), (255, 160, 0), 2)
                cv2.putText(im, "GT", (x0, max(12, y0 - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 160, 0), 2, cv2.LINE_AA)
            for cf, b, kind in results[wn].get(n, []):
                b = b.astype(int)
                cv2.rectangle(im, (b[0], b[1]), (b[2], b[3]), (0, 255, 0), 2)
                cv2.putText(im, f"{kind} {cf:.2f}", (b[0], min(im.shape[0] - 4, b[3] + 14)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 2, cv2.LINE_AA)
            lab, vis, _ = gt[n]
            bar = np.zeros((22, im.shape[1], 3), np.uint8)
            cv2.putText(bar, f"{n} {lab} vis={vis * 100:.0f}% "
                             f"{'DET' if n in results[wn] else 'MISS'}",
                        (5, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255),
                        1, cv2.LINE_AA)
            tiles.append(np.vstack([bar, im]))
        while len(tiles) % 3:
            tiles.append(np.zeros_like(tiles[0]))
        grid = np.vstack([np.hstack(tiles[i:i + 3]) for i in range(0, len(tiles), 3)])
        out = fr / f"eval_detect{tag}.jpg"
        cv2.imwrite(str(out), grid)
        print(f"\n對照圖（藍=幾何算的位置、綠=偵測框）：{out}")

    # OUT 的取樣圖：幾何說「車不在畫面裡」的那些，挑幾張讓人用眼睛確認。
    # 遮擋和沒遮擋各挑一半 —— 兩種 OUT 的長相完全不同（一種是看到貨架側面，
    # 一種是看到空地），只看其中一種會以為腳本在亂判。
    outs = [(n, v) for n, v in sorted(gt.items()) if v[0] == "OUT"]
    occ_l = [n for n, v in outs if v[2]]
    free_l = [n for n, v in outs if not v[2]]
    picks2 = occ_l[::max(1, len(occ_l) // 4)][:4] + free_l[::max(1, len(free_l) // 5)][:5]
    if picks2:
        tiles = []
        for n in picks2:
            im = cv2.imread(str(fr / n))
            lab, vis, occ = gt[n]
            why = "blocked by shelf" if occ else f"out of view (vis={vis * 100:.0f}%)"
            det = "DET!" if n in results[wn] else "no det"
            bar = np.zeros((22, im.shape[1], 3), np.uint8)
            cv2.putText(bar, f"{n} OUT: {why} / {det}", (5, 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1, cv2.LINE_AA)
            tiles.append(np.vstack([bar, im]))
        while len(tiles) % 3:
            tiles.append(np.zeros_like(tiles[0]))
        g2 = np.vstack([np.hstack(tiles[i:i + 3]) for i in range(0, len(tiles), 3)])
        out2 = fr / f"eval_out_samples{tag}.jpg"
        cv2.imwrite(str(out2), g2)
        print(f"OUT 取樣圖（該看不到車的那些）：{out2}")
    return 0


if __name__ == "__main__":
    sys.exit(main())

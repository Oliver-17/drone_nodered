#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""畫倉庫場景的三張圖（純 matplotlib，不開 Gazebo）。

    docs/warehouse_iso.png       等角俯瞰：整座倉庫，四排貨架和走道都在畫面裡
    docs/warehouse_aisle.png     走道視角：站在走道口往裡看，看得出高度關係
    docs/warehouse_top.png       正俯視：和 Nav2 的地圖同一個視角

為什麼不用 Gazebo 的截圖（2026-10-12）：
  ① Oliver 說他手動喬視角拍起來不好看；
  ② 無頭渲染在這台機器上走不通（EGL 建不出 dri2 screen），要走 X11 就得在
     他的螢幕上彈出視窗，會打擾他。
  ③ 幾何本來就全部在 gen_warehouse.py 裡（SDF 和地圖都是它產生的），
     直接照那份畫就不會有「圖和世界不一致」的問題 —— 這是這個做法真正的好處，
     截圖反而會隨著模型材質改變而和文件脫節。

所有尺寸都從 gen_warehouse 讀，不要在這裡重寫任何一個數字。

用法：
    python3 scripts/render_warehouse.py          # 三張都畫
    python3 scripts/render_warehouse.py --only iso
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import gen_warehouse as g                                        # noqa: E402

import matplotlib                                                # noqa: E402
matplotlib.use("Agg")
import matplotlib.patches as mpatches                            # noqa: E402
import matplotlib.pyplot as plt                                  # noqa: E402
from mpl_toolkits.mplot3d.art3d import Poly3DCollection          # noqa: E402

# 飛行高度。drone-warehouse.service 帶 altitude:=5.0（倉庫的巡航高度），
# 貨架 7 m、屋頂 8 m 都是為了配合它才定的 —— 見 gen_warehouse 檔頭。
FLIGHT_ALT = 5.0

SHELF_FC = "#d08a3e"      # 貨架
SHELF_EC = "#8a5520"
WALL_FC = "#aab0b6"       # 外牆
FLOOR_FC = "#eceff1"
DRONE_C = "#1565c0"
TARGET_C = "#d13b8a"
SPAWN_C = "#1a7f37"

# 標籤一律用英文：matplotlib 的預設字型沒有中文字符，中文會印成一排方框。
# （這條和 gen_warehouse.write_preview 的理由相同。）
OUT_DIR = os.path.join(g.PKG_DIR, "docs")


def _box(ax, cx, cy, sx, sy, h, fc, ec, alpha=1.0, z0=0.0):
    """在 3D 軸上畫一個長方體（六個面）。"""
    x0, x1 = cx - sx / 2, cx + sx / 2
    y0, y1 = cy - sy / 2, cy + sy / 2
    z1 = z0 + h
    faces = [
        [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0)],   # 底
        [(x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)],   # 頂
        [(x0, y0, z0), (x1, y0, z0), (x1, y0, z1), (x0, y0, z1)],   # 南
        [(x0, y1, z0), (x1, y1, z0), (x1, y1, z1), (x0, y1, z1)],   # 北
        [(x0, y0, z0), (x0, y1, z0), (x0, y1, z1), (x0, y0, z1)],   # 西
        [(x1, y0, z0), (x1, y1, z0), (x1, y1, z1), (x1, y0, z1)],   # 東
    ]
    ax.add_collection3d(Poly3DCollection(
        faces, facecolors=fc, edgecolors=ec, linewidths=0.4, alpha=alpha))


def _drone(ax, x, y, z, size=1.2):
    """簡化的四旋翼：兩根交叉的臂 + 四個轉子。看得出是飛機就夠了。"""
    a = size / 2
    for dx, dy in ((a, a), (-a, -a), (a, -a), (-a, a)):
        ax.plot([x, x + dx], [y, y + dy], [z, z], color=DRONE_C, lw=2)
        ax.scatter([x + dx], [y + dy], [z], color=DRONE_C, s=26, depthshade=False)
    ax.scatter([x], [y], [z], color=DRONE_C, s=50, marker="s", depthshade=False)
    # 到地面的細線：沒有它在 3D 圖上完全看不出來飛機在哪個水平位置
    ax.plot([x, x], [y, y], [0, z], color=DRONE_C, lw=0.8, ls=":", alpha=0.7)


def _scene3d(ax, walls=True, roof=False, wall_h=3.0, xlim=None):
    """把倉庫本體畫進 3D 軸（牆、貨架）。

    ⚠️ **不畫地板、預設也不畫屋頂**。matplotlib 的 3D 是「一個 collection
       整體排一次深度」，一塊 36x24 m 的地板平均深度很靠近鏡頭 →
       它會把畫在它上面的東西（航線、飛機、生成點）整個蓋掉，
       而且完全不會報錯，只是東西憑空消失（2026-10-12 第一版就是這樣）。
       地面用座標軸自己的格線表示就夠了。
    """
    for name, cx, cy, sx, sy, h, kind, _ in g.RECTS:
        # ⚠️ 3D 的 collection **不會**被 set_xlim 裁掉，超出範圍的貨架照樣畫，
        #    看起來就是「場地外面漂著兩排箱子」。只能自己先濾。
        if xlim is not None and not (xlim[0] <= cx <= xlim[1]):
            continue
        if kind == "wall":
            if not walls:
                continue
            # 牆只畫矮矮一圈：畫滿 8 m 的話從外面看就是一個灰盒子，
            # 裡面的貨架全被擋住。這裡只是要讓人看出場地的邊界在哪。
            _box(ax, cx, cy, sx, sy, wall_h, WALL_FC, "#78848c", alpha=0.25)
        else:
            _box(ax, cx, cy, sx, sy, h, SHELF_FC, SHELF_EC)
    if roof and g.ROOF_ENABLED:
        x0, x1, y0, y1 = g.bounds()
        if xlim is not None:
            x0, x1 = xlim
        ax.add_collection3d(Poly3DCollection(
            [[(x0, y0, g.ROOF_HEIGHT), (x1, y0, g.ROOF_HEIGHT),
              (x1, y1, g.ROOF_HEIGHT), (x0, y1, g.ROOF_HEIGHT)]],
            facecolors="#ffffff", edgecolors="#90a4ae",
            linewidths=0.5, alpha=g.ROOF_ALPHA))


def _loop(ax3d, pts, color, label, z=0.05, lw=2.5, ls="-"):
    xs = [p[0] for p in pts] + [pts[0][0]]
    ys = [p[1] for p in pts] + [pts[0][1]]
    ax3d.plot(xs, ys, [z] * len(xs), color=color, lw=lw, ls=ls, label=label)


def render_iso(path):
    """A 等角俯瞰。"""
    fig = plt.figure(figsize=(13, 8.5))
    ax = fig.add_subplot(111, projection="3d")
    _scene3d(ax, walls=True, roof=False)
    _loop(ax, g.TARGET_LOOP, TARGET_C, "target car loop (ground)", z=0.15)
    _loop(ax, g.SEARCH_LOOP, DRONE_C,
          f"drone search loop ({FLIGHT_ALT:.0f} m, reversed)",
          z=FLIGHT_ALT, ls="--", lw=1.8)
    ax.scatter([g.SPAWN[0]], [g.SPAWN[1]], [0.15], color=SPAWN_C, s=200,
               marker="*", label="drone spawn", depthshade=False)
    # ⚠️ 飛機和車都擺在**南側的橫走道**（環線的第 0 點那一帶），不要擺在
    #    貨架之間：貨架 7 m 高，從斜上方看會把走道裡的東西整個擋掉 ——
    #    東西還在圖上，只是被 7 m 的箱子遮住，看起來像沒畫出來。
    _drone(ax, g.SEARCH_LOOP[0][0], g.SEARCH_LOOP[0][1], FLIGHT_ALT, size=2.4)
    ax.scatter([g.TARGET_LOOP[0][0]], [g.TARGET_LOOP[0][1]], [0.25],
               color=TARGET_C, s=90, marker="s", label="target car",
               depthshade=False)
    # 地面外框只畫線不畫面（畫面會蓋住東西，理由見 _scene3d）
    gx0, gx1, gy0, gy1 = g.bounds()
    ax.plot([gx0, gx1, gx1, gx0, gx0], [gy0, gy0, gy1, gy1, gy0], [0] * 5,
            color="#90a4ae", lw=1.0)

    x0, x1, y0, y1 = g.bounds()
    ax.set_xlim(x0, x1)
    ax.set_ylim(y0, y1)
    ax.set_zlim(0, g.ROOF_HEIGHT + 1)
    # 等比例：不設的話 matplotlib 會把 z 拉長，8 m 高的倉庫看起來像大樓
    # z 放大 2.5 倍：照真實比例的話 8 m 高的倉庫在 36 m 寬的場地裡
    # 只剩一條線，看不出貨架和飛行高度的關係。
    ax.set_box_aspect((x1 - x0, y1 - y0, (g.ROOF_HEIGHT + 1) * 2.5))
    # 從南邊偏西看：貨架是南北向的，從側面看會變成一道牆，
    # 從南邊看才看得進走道裡（飛機和航線都在走道上）。
    ax.view_init(elev=24, azim=-78)
    ax.set_xlabel("x (m, East)")
    ax.set_ylabel("y (m, North)")
    ax.set_zlabel("z (m)")
    ax.set_title(f"warehouse  {x1 - x0:.0f} x {y1 - y0:.0f} m   "
                 f"shelves {g.SHELF_HEIGHT:.0f} m   roof {g.ROOF_HEIGHT:.0f} m   "
                 f"flight {FLIGHT_ALT:.0f} m")
    ax.legend(loc="upper left", fontsize=8)
    fig.savefig(path, dpi=130, bbox_inches="tight")
    plt.close(fig)


def render_aisle(path):
    """B 走道視角：站在中間那條走道的南端往北看。

    matplotlib 的 3D 相機不能隨便擺在場景裡面，所以做法是
    「把視野裁到只剩那條走道 + 低仰角 + 透視投影」，效果等同站在走道口。
    """
    fig = plt.figure(figsize=(12, 8))
    ax = fig.add_subplot(111, projection="3d")
    _scene3d(ax, walls=False, roof=True,
             xlim=(-g.SHELF_PITCH, g.SHELF_PITCH))
    ax.plot([g.PATROL_AISLE_X] * 2, [-g.SHELF_LENGTH / 2, g.SHELF_LENGTH / 2],
            [0.05, 0.05], color=TARGET_C, lw=3, label="target car patrol")
    _drone(ax, g.PATROL_AISLE_X, 1.0, FLIGHT_ALT, size=1.8)
    ax.scatter([g.PATROL_AISLE_X], [-3.0], [0.2], color=TARGET_C, s=90,
               marker="s", label="target car", depthshade=False)
    # 飛行高度的參考面：這張圖的重點就是「飛機 5 m、貨架 7 m」的高度關係
    ax.plot([-g.SHELF_PITCH, g.SHELF_PITCH], [g.SHELF_LENGTH / 2] * 2,
            [FLIGHT_ALT] * 2, color=DRONE_C, lw=1, ls="--")
    ax.text(-g.SHELF_PITCH, g.SHELF_LENGTH / 2, FLIGHT_ALT + 0.3,
            f"flight {FLIGHT_ALT:.0f} m", color=DRONE_C, fontsize=9)

    # 只看中間那條走道和兩側的貨架
    ax.set_xlim(-g.SHELF_PITCH, g.SHELF_PITCH)
    ax.set_ylim(-g.SHELF_LENGTH / 2 - 2, g.SHELF_LENGTH / 2 + 2)
    ax.set_zlim(0, g.ROOF_HEIGHT + 0.5)
    ax.set_box_aspect((2 * g.SHELF_PITCH, g.SHELF_LENGTH + 4, g.ROOF_HEIGHT * 1.6))
    # 透視投影（預設是正交）。focal_length 要 matplotlib 3.6 才有，
    # 這台是 3.5.1，所以只能用預設焦距。
    ax.set_proj_type("persp")
    ax.view_init(elev=8, azim=-90)
    # y 是「往畫面深處」的方向，刻度標在這個視角下會疊成一團而且沒有資訊量
    ax.set_yticks([])
    ax.set_xlabel("x (m, East)")
    ax.set_zlabel("z (m)")
    ax.set_title(f"aisle view   aisle {g.AISLE_WIDTH:.1f} m wide   "
                 f"shelf {g.SHELF_HEIGHT:.0f} m vs flight {FLIGHT_ALT:.0f} m")
    ax.legend(loc="upper left", fontsize=8)
    fig.savefig(path, dpi=130, bbox_inches="tight")
    plt.close(fig)


def render_top(path):
    """C 正俯視：和 Nav2 讀的那張地圖同一個視角。"""
    x0, x1, y0, y1 = g.bounds()
    fig, ax = plt.subplots(figsize=(12, 8))
    for name, cx, cy, sx, sy, h, kind, _ in g.RECTS:
        fc, ec = (WALL_FC, "#78848c") if kind == "wall" else (SHELF_FC, SHELF_EC)
        ax.add_patch(mpatches.Rectangle((cx - sx / 2, cy - sy / 2), sx, sy,
                                        facecolor=fc, edgecolor=ec, lw=0.8))
        if kind == "shelf":
            ax.text(cx, 0, f"{cx:+.1f}", ha="center", va="center",
                    fontsize=8, color="white", rotation=90)
    for axc in g.AISLE_CENTERS_X:
        ax.plot([axc, axc], [-g.SHELF_LENGTH / 2 - 2, g.SHELF_LENGTH / 2 + 2],
                "--", color="#90a4ae", lw=1)
        # 標籤放下面：上面 y=+9 那一排是環線的轉角和航點編號，會疊在一起
        ax.text(axc, -g.SHELF_LENGTH / 2 - 3.2, f"aisle {axc:+.1f}",
                ha="center", fontsize=8, color="#546e7a")
    # ⚠️ 兩條環線在地圖上是**同一個長方形**（飛機走的就是車那條的反向），
    #    照實畫會完全疊在一起，看起來只有一條。所以飛機那條往內縮 0.45 m
    #    「只是為了畫得出來」，真正的差別是**方向**，用箭頭表示。
    def ring(pts, inset, color, lw, ls, label):
        cx = sum(p[0] for p in pts) / len(pts)
        cy = sum(p[1] for p in pts) / len(pts)
        q = [(x + (1 if cx > x else -1) * inset, y + (1 if cy > y else -1) * inset)
             for x, y in pts]
        ax.plot([p[0] for p in q] + [q[0][0]], [p[1] for p in q] + [q[0][1]],
                ls, color=color, lw=lw, label=label)
        for i in range(len(q)):            # 每一段中點放一個箭頭表示行進方向
            ax_, ay = q[i]
            bx, by = q[(i + 1) % len(q)]
            ax.annotate("", xy=((ax_ + bx) / 2, (ay + by) / 2),
                        xytext=((ax_ + bx) / 2 - (bx - ax_) * 0.12,
                                (ay + by) / 2 - (by - ay) * 0.12),
                        arrowprops=dict(arrowstyle="-|>", color=color, lw=lw))
        return q

    ring(g.TARGET_LOOP, 0.0, TARGET_C, 3, "-", "target car loop (ground)")
    q = ring(g.SEARCH_LOOP, 0.45, DRONE_C, 1.6, "--",
             f"drone search loop ({FLIGHT_ALT:.0f} m, reversed)")
    for i, (px, py) in enumerate(q):
        ax.annotate(str(i), (px, py), textcoords="offset points",
                    xytext=(8, -12 if py > 0 else 8), color=DRONE_C,
                    fontsize=10, weight="bold")
    ax.plot(*g.SPAWN, "*", color=SPAWN_C, markersize=18, label="drone spawn")
    ax.set_xlim(x0 - 1, x1 + 1)
    ax.set_ylim(y0 - 1, y1 + 1)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.set_xlabel("x (m, East)")
    ax.set_ylabel("y (m, North)")
    ax.set_title(f"warehouse top view  {x1 - x0:.0f} x {y1 - y0:.0f} m   "
                 f"aisle {g.AISLE_WIDTH:.1f} m")
    ax.legend(loc="upper right", fontsize=8)
    fig.savefig(path, dpi=130, bbox_inches="tight")
    plt.close(fig)


JOBS = {"iso": ("warehouse_iso.png", render_iso),
        "aisle": ("warehouse_aisle.png", render_aisle),
        "top": ("warehouse_top.png", render_top)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", choices=sorted(JOBS), help="只畫其中一張")
    ap.add_argument("--out", default=OUT_DIR)
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    for key in ([a.only] if a.only else list(JOBS)):
        name, fn = JOBS[key]
        p = os.path.join(a.out, name)
        fn(p)
        print(f"✓ {p}  ({os.path.getsize(p) // 1024} KB)")


if __name__ == "__main__":
    main()

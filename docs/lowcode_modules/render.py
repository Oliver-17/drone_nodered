#!/usr/bin/env python3
"""產生 low-code 四大模組的兩張圖（報告用）。

    python3 docs/lowcode_modules/render.py

    m01_四大模組.html  手寫的 HTML（四張說明卡，「分開」）
    m02_模組串連.html  由這支產生（資料怎麼在模組之間流、以及「自訂任務」的實際串法）
    m03／m04          簡易版（報告用，一眼看得懂）：四大模組、模組串連（只有資料流）
    兩張都用無頭 Chrome 截成同名 PNG。

為什麼用 HTML 截圖、不用 matplotlib（husky_nodered/docs 的報告圖是 matplotlib）：
說明卡是表格加大量換行文字，matplotlib 要自己算每一行的位置；
HTML 讓瀏覽器排版，改字不用重算座標。
"""
import os
import subprocess
import tempfile

from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
W = 1600

# 四個模組的顏色（框、底），和 m01 的卡片一致
# 配色（2026-11-17 改成兩色：投影片上和 Node-RED 截圖放在一起，原本四模組四種顏色太花）
#   藍＝無人機／系統（同 Node-RED 無人機積木的藍）　棕＝地面車（同地面車積木的棕）
#   四大模組全部同一個藍，靠編號區分；④ 系統整合用深一階的藍，表示它包住其他三個。
BLUE = ("#2e6da4", "#eaf2fa", "#1f4e79")
C = {
    1: BLUE, 2: BLUE, 3: BLUE,
    4: ("#1f4e79", "#dce8f4", "#1f4e79"),
    5: ("#b7782c", "#f8eedf", "#8a5a1e"),     # 地面車
    0: ("#7b8794", "#eceff1", "#3e4c59"),     # 載具／平台
}
FONT = "Noto Sans CJK TC"


def esc(s):
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def box(x, y, w, h, k, title, lines, title_size=21, size=15, rx=10):
    edge, fill, ink = C[k]
    out = [f'<rect x="{x}" y="{y}" width="{w}" height="{h}" rx="{rx}" fill="{fill}" '
           f'stroke="{edge}" stroke-width="2.5"/>']
    ty = y + 32
    out.append(f'<text x="{x + w / 2}" y="{ty}" text-anchor="middle" font-size="{title_size}" '
               f'font-weight="700" fill="{ink}">{esc(title)}</text>')
    for i, ln in enumerate(lines):
        out.append(f'<text x="{x + w / 2}" y="{ty + 28 + i * (size + 8)}" text-anchor="middle" '
                   f'font-size="{size}" fill="#323f4b">{esc(ln)}</text>')
    return "\n".join(out)


def arrow(x1, y1, x2, y2, color="#3e4c59", dash=False, width=2.5, label=None,
          lx=None, ly=None, anchor="middle", lsize=14, lcolor=None):
    d = ' stroke-dasharray="7 5"' if dash else ""
    mk = {"#3e4c59": "a", "#d64545": "r", "#d68910": "o", "#8e44ad": "p"}[color]
    out = [f'<line x1="{x1}" y1="{y1}" x2="{x2}" y2="{y2}" stroke="{color}" '
           f'stroke-width="{width}"{d} marker-end="url(#{mk})"/>']
    if label:
        lx = (x1 + x2) / 2 if lx is None else lx
        ly = (y1 + y2) / 2 - 8 if ly is None else ly
        for i, t in enumerate(label.split("\n")):
            out.append(f'<text x="{lx}" y="{ly + i * (lsize + 5)}" text-anchor="{anchor}" '
                       f'font-size="{lsize}" fill="{lcolor or color}">{esc(t)}</text>')
    return "\n".join(out)


def path(d, color="#3e4c59", dash=False, width=2.5):
    mk = {"#3e4c59": "a", "#d64545": "r", "#d68910": "o", "#8e44ad": "p"}[color]
    da = ' stroke-dasharray="7 5"' if dash else ""
    return (f'<path d="{d}" fill="none" stroke="{color}" stroke-width="{width}"{da} '
            f'marker-end="url(#{mk})"/>')


def text(x, y, s, size=15, color="#323f4b", anchor="middle", weight=400):
    return (f'<text x="{x}" y="{y}" text-anchor="{anchor}" font-size="{size}" '
            f'font-weight="{weight}" fill="{color}">{esc(s)}</text>')


def dots(cx, y, mods):
    """積木下方的小色點：這一步動用到哪幾個模組。"""
    r, gap = 11, 26
    x0 = cx - (len(mods) - 1) * gap / 2
    out = []
    for i, m in enumerate(mods):
        edge, fill, ink = C[m]
        out.append(f'<circle cx="{x0 + i * gap}" cy="{y}" r="{r}" fill="{edge}"/>')
        out.append(f'<text x="{x0 + i * gap}" y="{y + 5}" text-anchor="middle" font-size="13" '
                   f'font-weight="700" fill="#fff">{"①②③④"[m - 1]}</text>')
    return "\n".join(out)


def make_m02():
    s = []
    H = 1150
    s.append(text(W / 2, 46, "Low-Code 系統　模組串連", 30, "#1f2933", weight=700))

    # ---------------- 上半：資料怎麼在模組之間流 ----------------
    s.append(text(40, 100, "(A) 資料流：一次「追蹤目標」時，資料怎麼經過四個模組", 19,
                  "#1f2933", "start", 700))
    y, h = 170, 190
    bx = [40, 450, 860, 1270]
    bw = [300, 300, 300, 290]
    s.append(box(bx[0], y, bw[0], h, 1, "① 感測／資料輸入",
                 ["相機＋深度（15 Hz）", "PX4 位置／地面車里程計", "ros_gz_bridge", "積木「ROS2 訂閱」"]))
    s.append(box(bx[1], y, bw[1], h, 2, "② AI 分析",
                 ["YOLO 找車（框）", "框＋深度 → 3D 點", "→ 地圖座標", "畫框影像 debug_image"]))
    s.append(box(bx[2], y, bw[2], h, 3, "③ 控制與流程",
                 ["積木：追蹤目標", "drone_api／husky_api 節點", "跟隨控制 10 Hz、Nav2", "看門狗／PX4 橋"]))
    s.append(box(bx[3], y, bw[3], h, 0, "載具（模擬／實機）",
                 ["無人機：PX4 飛控", "地面車：Husky 差速驅動", "Gazebo Harmonic", "（實機：Jetson＋飛控）"]))
    my = y + 95
    s.append(arrow(bx[0] + bw[0], my, bx[1] - 4, my, label="影像、深度", ly=my - 12))
    s.append(arrow(bx[1] + bw[1], my, bx[2] - 4, my, label="api/target", ly=my - 12))
    s.append(text((bx[1] + bw[1] + bx[2]) / 2, my + 26, "目標地圖座標", 14, "#3e4c59"))
    s.append(arrow(bx[2] + bw[2], my, bx[3] - 4, my, label="cmd_vel", ly=my - 12))
    s.append(text((bx[2] + bw[2] + bx[3]) / 2, my + 26, "Nav2 目標", 14, "#3e4c59"))
    # 閉迴路：載具動了 → 感測到的東西跟著變
    s.append(path(f"M {bx[3] + bw[3] / 2} {y} L {bx[3] + bw[3] / 2} {y - 34} "
                  f"L {bx[0] + bw[0] / 2} {y - 34} L {bx[0] + bw[0] / 2} {y - 4}", dash=True))
    s.append(text(W / 2, y - 42, "閉迴路：載具移動 → 畫面與位置跟著改變 → 再算一次", 15, "#3e4c59"))

    # ④ 整合：橫跨底下
    iy, ih = 470, 120
    s.append(box(40, iy, W - 80, ih, 4, "④ 系統整合",
                 ["rosbridge（ws:9090）連接 Node-RED ↔ ROS 2　·　儀表板（就緒燈、地圖、速度、任務狀態）",
                  "相機小視窗（web_video_server :8080）　·　systemd 服務一鍵開關模擬／底座　·　安裝腳本"],
                 size=16))
    # 上下箭頭：狀態上來、指令下去
    ay1, ay2 = y + h, iy
    s.append(arrow(bx[0] + 150, ay1 + 2, bx[0] + 150, ay2 - 4, color="#3e4c59",
                   label="位置、狀態\napi/status 5 Hz", lx=bx[0] + 162, ly=ay1 + 46, anchor="start"))
    s.append(arrow(bx[1] + 150, ay1 + 2, bx[1] + 150, ay2 - 4, color="#3e4c59",
                   label="畫框影像\n→ 相機小視窗", lx=bx[1] + 162, ly=ay1 + 46, anchor="start"))
    s.append(arrow(bx[2] + 100, ay2 - 2, bx[2] + 100, ay1 + 4, color="#3e4c59",
                   label="觸發任務\n（按鈕／流程）", lx=bx[2] + 88, ly=ay1 + 46, anchor="end"))
    s.append(arrow(bx[2] + 210, ay1 + 2, bx[2] + 210, ay2 - 4, color="#3e4c59",
                   label="完成／失敗\n任務進度", lx=bx[2] + 222, ly=ay1 + 46, anchor="start"))

    # ---------------- 下半：自訂任務的實際串法 ----------------
    s.append(text(40, 660, "(B) 積木串連：Node-RED「自訂任務」分頁（儀表板按「開始任務」觸發）", 19,
                  "#1f2933", "start", 700))
    s.append(text(40, 688, "每塊積木 = 呼叫一個服務並等它真的完成；上出口＝完成接下一塊，下出口＝失敗接處置。"
                  "色點＝這一步動用到的模組（④ 是因為每塊都經過 rosbridge 呼叫服務）。", 15, "#52606d", "start"))

    bw2, bh2, gap = 168, 56, 38
    x0 = 230
    start_y = 840
    # 起點
    s.append(box(40, start_y - 30, 130, 60, 4, "任務開始", [], title_size=18))
    lanes = [
        (760, "無人機", [("起飛", [1, 3, 4]), ("導航到座標", [1, 3, 4]), ("追蹤目標", [1, 2, 3, 4]),
                         ("降落", [1, 3, 4]), ("返航", [1, 3, 4])], "失敗 → 已降落"),
        (950, "地面車", [("車：導航到座標", [1, 3, 4]), ("車：追蹤目標", [1, 2, 3, 4]),
                         ("車：返航", [1, 3, 4])], "失敗 → 停止"),
    ]
    for ly, name, blocks, fail in lanes:
        s.append(text(x0 - 6, ly - 40, name, 16, "#1e7a3a", "start", 700))
        # 起點 → 第一塊
        s.append(path(f"M 170 {start_y} L 200 {start_y} L 200 {ly} L {x0 - 4} {ly}"))
        xs = []
        for i, (b, mods) in enumerate(blocks):
            x = x0 + i * (bw2 + gap)
            xs.append(x)
            edge, fill, ink = C[3]
            s.append(f'<rect x="{x}" y="{ly - bh2 / 2}" width="{bw2}" height="{bh2}" rx="8" '
                     f'fill="{fill}" stroke="{edge}" stroke-width="2.5"/>')
            s.append(text(x + bw2 / 2, ly + 6, b, 17, ink, weight=700))
            s.append(dots(x + bw2 / 2, ly + bh2 / 2 + 22, mods))
            if i:
                s.append(arrow(x - gap, ly - 8, x - 4, ly - 8))
            # 失敗出口：右下角往下接到失敗軌
            s.append(path(f"M {x + bw2} {ly + 14} L {x + bw2 + 14} {ly + 14} "
                          f"L {x + bw2 + 14} {ly + 70}", color="#d64545", dash=True, width=2))
        # 完成
        xe = x0 + len(blocks) * (bw2 + gap)
        s.append(arrow(xe - gap, ly - 8, xe - 4, ly - 8))
        s.append(f'<rect x="{xe}" y="{ly - bh2 / 2}" width="150" height="{bh2}" rx="28" '
                 f'fill="#fff" stroke="#3e4c59" stroke-width="2"/>')
        s.append(text(xe + 75, ly + 6, f"{name}任務完成", 16, "#1f2933", weight=700))
        # 失敗軌
        ry = ly + 76
        s.append(f'<line x1="{x0 + bw2}" y1="{ry}" x2="{xs[-1] + bw2 + 14}" y2="{ry}" '
                 f'stroke="#d64545" stroke-width="2" stroke-dasharray="7 5"/>')
        s.append(text(xs[-1] + bw2 + 30, ry + 5, fail, 15, "#d64545", "start", 700))

    s.append(text(W - 40, start_y - 2, "兩條線同時出發、", 15, "#52606d", "end"))
    s.append(text(W - 40, start_y + 20, "各自用完成／失敗出口往下走", 15, "#52606d", "end"))

    # 圖例
    ly = 1105
    items = [(1, "① 感測／資料輸入"), (2, "② AI 分析"), (3, "③ 控制與流程"), (4, "④ 系統整合"),
             (0, "載具／平台")]
    x = 40
    for k, t in items:
        edge, fill, ink = C[k]
        s.append(f'<rect x="{x}" y="{ly - 15}" width="34" height="20" rx="4" fill="{fill}" '
                 f'stroke="{edge}" stroke-width="2"/>')
        s.append(text(x + 44, ly, t, 15, "#323f4b", "start"))
        x += 230
    s.append(f'<line x1="{x}" y1="{ly - 5}" x2="{x + 40}" y2="{ly - 5}" stroke="#d64545" '
             f'stroke-width="2" stroke-dasharray="7 5"/>')
    s.append(text(x + 50, ly, "失敗出口", 15, "#323f4b", "start"))

    defs = "".join(
        f'<marker id="{k}" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="7" markerHeight="7" '
        f'orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="{c}"/></marker>'
        for k, c in (("a", "#3e4c59"), ("r", "#d64545"), ("o", "#d68910"), ("p", "#8e44ad")))
    svg = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{H}" '
           f'font-family="{FONT}"><defs>{defs}</defs>'
           f'<rect width="{W}" height="{H}" fill="#fff"/>' + "\n".join(s) + "</svg>")
    html = ('<!doctype html><html lang="zh-Hant"><head><meta charset="utf-8">'
            '<!-- 由 render.py 產生，不要手改 --></head>'
            f'<body style="margin:0;background:#fff">{svg}</body></html>')
    with open(os.path.join(HERE, "m02_模組串連.html"), "w", encoding="utf-8") as f:
        f.write(html)


def svg_page(name, body, h):
    defs = "".join(
        f'<marker id="{k}" viewBox="0 0 10 10" refX="9" refY="5" markerWidth="6" markerHeight="6" '
        f'orient="auto-start-reverse"><path d="M0,0 L10,5 L0,10 z" fill="{c}"/></marker>'
        for k, c in (("a", "#3e4c59"), ("r", "#d64545"), ("o", "#d68910"), ("p", "#8e44ad")))
    svg = (f'<svg xmlns="http://www.w3.org/2000/svg" width="{W}" height="{h}" '
           f'font-family="{FONT}"><defs>{defs}</defs>'
           f'<rect width="{W}" height="{h}" fill="#fff"/>' + "\n".join(body) + "</svg>")
    with open(os.path.join(HERE, name + ".html"), "w", encoding="utf-8") as f:
        f.write('<!doctype html><html lang="zh-Hant"><head><meta charset="utf-8">'
                '<!-- 由 render.py 產生，不要手改 --></head>'
                f'<body style="margin:0;background:#fff">{svg}</body></html>')


# ---------------------------------------------------------------------------
#  簡易版（2026-11-17 Oliver：詳細版字太多，要一眼看得懂的）
#  原則：每格只留短句、不放話題名稱和頻率；細節看 m01／m02。
# ---------------------------------------------------------------------------
# 五個欄位照作業要求：模組功能、使用目的、輸入與輸出資料概念、與其他模組關係、適用情境。
# 每格一句、不放話題名稱和頻率（細節看 m01）。
SIMPLE = [
    # (編號, 名稱, 關鍵詞, [功能, 目的, 輸入→輸出, 關係, 情境])
    (1, "感測／資料輸入", "收集資料",
     ["收集相機畫面和載具位置", "讓系統知道看到什麼、在哪裡",
      "相機、飛控 → 影像、位置", "提供資料給 ② 和 ③", "所有任務的第一步"]),
    (2, "AI 分析", "看懂畫面",
     ["用 YOLO 找出畫面中的目標車", "把畫面變成目標的位置",
      "影像 → 目標位置", "用 ① 的影像，結果交給 ③", "追蹤目標、搜尋目標"]),
    (3, "控制與流程", "決定怎麼動",
     ["用積木排任務、算出移動指令", "不寫程式也能組出任務",
      "目標位置 → 移動指令", "用 ①② 的資料，由 ④ 觸發", "自訂任務、按鈕手動操作"]),
    (4, "系統整合", "串起來給人用",
     ["連接 Node-RED 與 ROS 2", "一鍵操作、隨時看狀態",
      "各模組狀態 → 儀表板", "串起 ①②③，給使用者操作", "日常操作、展示、上實機"]),
]
LABELS = ["模組功能", "使用目的", "輸入與輸出", "與其他模組", "適用情境"]


def make_simple_modules():
    s = [text(W / 2, 58, "Low-Code 系統　四大模組", 34, "#1f2933", weight=700)]
    cw, gap, y0, head = 355, 27, 100, 160
    rh = 74                                   # 每一欄：上面小標籤、下面一句話
    ch = head + 18 + rh * len(LABELS)
    x0 = (W - 4 * cw - 3 * gap) / 2          # 整排置中
    for i, (k, name, tag, vals) in enumerate(SIMPLE):
        x = x0 + i * (cw + gap)
        edge, fill, ink = C[k]
        s.append(f'<rect x="{x}" y="{y0}" width="{cw}" height="{ch}" rx="16" fill="#fff" '
                 f'stroke="{edge}" stroke-width="3"/>')
        s.append(f'<path d="M {x} {y0 + 16} a 16 16 0 0 1 16 -16 h {cw - 32} a 16 16 0 0 1 16 16 '
                 f'v {head - 16} h {-cw} z" fill="{fill}"/>')
        s.append(f'<circle cx="{x + cw / 2}" cy="{y0 + 48}" r="26" fill="{edge}"/>')
        s.append(text(x + cw / 2, y0 + 58, str(k), 28, "#fff", weight=700))
        s.append(text(x + cw / 2, y0 + 112, name, 27, ink, weight=700))
        s.append(text(x + cw / 2, y0 + 142, f"「{tag}」", 18, ink))
        for j, (lab, val) in enumerate(zip(LABELS, vals)):
            ry = y0 + head + 18 + j * rh
            if j:
                s.append(f'<line x1="{x + 20}" y1="{ry - 8}" x2="{x + cw - 20}" y2="{ry - 8}" '
                         f'stroke="#e4e7eb" stroke-width="1.5"/>')
            s.append(text(x + 24, ry + 18, lab, 16, edge, "start", 700))
            s.append(text(x + 24, ry + 48, val, 19, "#1f2933", "start"))
    svg_page("m03_四大模組_簡易", s, y0 + ch + 30)


def make_simple_flow():
    s = [text(W / 2, 58, "Low-Code 系統　模組串連", 34, "#1f2933", weight=700),
         text(W / 2, 94, "以「追蹤目標」為例：資料依序經過各模組，載具動了又回到第一步", 19, "#52606d")]
    y, h = 190, 150
    xs, bw = [40, 450, 860, 1270], 290      # 框之間留 120 px 給箭頭上的字
    items = [(1, "① 感測／資料輸入", "收集影像和位置"),
             (2, "② AI 分析", "找出目標在哪裡"),
             (3, "③ 控制與流程", "決定載具怎麼動"),
             (0, "載具", "無人機／地面車")]
    for x, (k, t, sub) in zip(xs, items):
        edge, fill, ink = C[k]
        s.append(f'<rect x="{x}" y="{y}" width="{bw}" height="{h}" rx="14" fill="{fill}" '
                 f'stroke="{edge}" stroke-width="3"/>')
        s.append(text(x + bw / 2, y + 66, t, 26, ink, weight=700))
        s.append(text(x + bw / 2, y + 106, sub, 19, "#323f4b"))
    my = y + h / 2
    for i, lab in enumerate(["影像", "目標位置", "移動指令"]):
        a, b = xs[i] + bw, xs[i + 1]
        s.append(arrow(a + 4, my, b - 6, my, width=3, label=lab, ly=my - 14, lsize=18,
                       lcolor="#1f2933"))
    # 閉迴路
    cx0, cx3 = xs[0] + bw / 2, xs[3] + bw / 2
    s.append(path(f"M {cx3} {y} L {cx3} {y - 40} L {cx0} {y - 40} L {cx0} {y - 6}", dash=True,
                  width=2.5))
    s.append(f'<rect x="{W / 2 - 150}" y="{y - 56}" width="300" height="30" fill="#fff"/>')
    s.append(text(W / 2, y - 34, "載具動了 → 畫面跟著變 → 再算一次", 17, "#3e4c59"))
    # ④
    iy, ih = 470, 110
    edge, fill, ink = C[4]
    s.append(f'<rect x="40" y="{iy}" width="{W - 80}" height="{ih}" rx="14" fill="{fill}" '
             f'stroke="{edge}" stroke-width="3"/>')
    s.append(text(W / 2, iy + 50, "④ 系統整合", 26, ink, weight=700))
    s.append(text(W / 2, iy + 86, "連接所有模組　·　儀表板看狀態　·　一鍵開關模擬", 19, "#323f4b"))
    a1, a2 = y + h + 4, iy - 6
    s.append(arrow(xs[0] + bw / 2, a1, xs[0] + bw / 2, a2, color="#3e4c59", width=3,
                   label="位置、狀態", lx=xs[0] + bw / 2 + 14, ly=(a1 + a2) / 2 + 6, anchor="start",
                   lsize=17))
    s.append(arrow(xs[1] + bw / 2, a1, xs[1] + bw / 2, a2, color="#3e4c59", width=3,
                   label="相機畫面", lx=xs[1] + bw / 2 + 14, ly=(a1 + a2) / 2 + 6, anchor="start",
                   lsize=17))
    s.append(arrow(xs[2] + bw / 2 - 50, a2 + 4, xs[2] + bw / 2 - 50, a1 + 2, color="#3e4c59",
                   width=3, label="下達任務", lx=xs[2] + bw / 2 - 64, ly=(a1 + a2) / 2 + 6,
                   anchor="end", lsize=17))
    s.append(arrow(xs[2] + bw / 2 + 50, a1, xs[2] + bw / 2 + 50, a2, color="#3e4c59", width=3,
                   label="完成／失敗", lx=xs[2] + bw / 2 + 64, ly=(a1 + a2) / 2 + 6,
                   anchor="start", lsize=17))
    svg_page("m04_模組串連_簡易", s, 620)


# ---------------------------------------------------------------------------
#  飛航動作介面（2026-11-17）：作業「Low-Code 可調用之飛航動作接口規格建議」
#  挑無人機三個＋地面車一個，五個欄位照作業要求。內容照實際積木：
#  參數名＝積木的環境變數（ALTITUDE 等），逾時＝積木預設值。
# ---------------------------------------------------------------------------
ACT_LABELS = ["動作用途", "輸入參數", "輸出結果", "流程觸發條件", "與飛控系統介接"]
ACTIONS = [
    (1, "無人機", "起飛", "Takeoff",
     ["解鎖並爬升到指定高度",
      "ALTITUDE 高度\n預設 1.5 m，限 0.5～5 m",
      "成功 → 完成出口\n被拒絕／逾時 90 秒 → 失敗出口",
      "飛機在地面、PX4 已連線",
      "送 PX4 指令：解鎖＋起飛\n（PX4 內建起飛模式）"]),
    (1, "無人機", "導航到座標", "Waypoint Navigation",
     ["飛到地圖上的指定點，\n自動繞開貨架",
      "MAP_X、MAP_Y 座標\nYAW 機頭方向（可不指定）",
      "抵達 → 完成出口\n導航失敗／逾時 240 秒 → 失敗出口",
      "已在空中、Nav2 就緒",
      "Nav2 規劃路徑 → 速度指令\n→ 轉成 PX4 offboard 設定點"]),
    (1, "無人機", "追蹤目標", "Target Tracking",
     ["找到目標車，保持距離\n跟著飛",
      "DISTANCE 距離（0＝自動）\nDURATION 時間、搜尋路線",
      "追滿時間 → 完成出口\n被拒絕／逾時 → 失敗出口",
      "已在空中、YOLO 偵測開著",
      "搜尋：PX4 定點飛行指令\n跟隨：速度指令 → offboard"]),
    (5, "地面車", "車：返航", "Return to Home",
     ["把車開回出發點",
      "不用填\n（家的位置自動從狀態讀取）",
      "到家 → 完成出口\n不知道家在哪／導航失敗 → 失敗出口",
      "地面車就緒\n（Nav2、里程計正常）",
      "沒有飛控：Nav2 → 速度指令\n→ 看門狗 → 車輪驅動"]),
]


def make_actions():
    s = [text(W / 2, 58, "Low-Code 任務流程　飛航動作接口", 34, "#1f2933", weight=700)]
    cw, gap, y0, head = 355, 27, 100, 130
    x0 = (W - 4 * cw - 3 * gap) / 2
    # 每一欄的高度 = 標籤一行 + 內容行數；四張卡同一欄要對齊，取四張裡最多行的
    nlines = [max(len(a[4][j].split("\n")) for a in ACTIONS) for j in range(len(ACT_LABELS))]
    rhs = [34 + 28 * n + 14 for n in nlines]
    ch = head + 14 + sum(rhs)
    for i, (k, veh, name, en, vals) in enumerate(ACTIONS):
        x = x0 + i * (cw + gap)
        edge, fill, ink = C[k]
        s.append(f'<rect x="{x}" y="{y0}" width="{cw}" height="{ch}" rx="16" fill="#fff" '
                 f'stroke="{edge}" stroke-width="3"/>')
        s.append(f'<path d="M {x} {y0 + 16} a 16 16 0 0 1 16 -16 h {cw - 32} a 16 16 0 0 1 16 16 '
                 f'v {head - 16} h {-cw} z" fill="{fill}"/>')
        s.append(f'<rect x="{x + cw / 2 - 44}" y="{y0 + 18}" width="88" height="30" rx="15" '
                 f'fill="{edge}"/>')
        s.append(text(x + cw / 2, y0 + 39, veh, 17, "#fff", weight=700))
        s.append(text(x + cw / 2, y0 + 88, name, 28, ink, weight=700))
        s.append(text(x + cw / 2, y0 + 116, en, 17, ink))
        ry = y0 + head + 14
        for j, (lab, val) in enumerate(zip(ACT_LABELS, vals)):
            if j:
                s.append(f'<line x1="{x + 20}" y1="{ry - 8}" x2="{x + cw - 20}" y2="{ry - 8}" '
                         f'stroke="#e4e7eb" stroke-width="1.5"/>')
            s.append(text(x + 24, ry + 18, lab, 16, edge, "start", 700))
            for m, ln in enumerate(val.split("\n")):
                s.append(text(x + 24, ry + 46 + m * 28, ln, 18, "#1f2933", "start"))
            ry += rhs[j]
    # 共同規格：每個積木都一樣，不放進卡片重複寫
    fy = y0 + ch + 28
    s.append(f'<rect x="{x0}" y="{fy}" width="{4 * cw + 3 * gap}" height="96" rx="12" '
             f'fill="#f5f7fa" stroke="#cbd2d9" stroke-width="1.5"/>')
    s.append(text(x0 + 24, fy + 36, "共同規格", 19, "#3e4c59", "start", 700))
    s.append(text(x0 + 130, fy + 36, "每個動作都是一塊積木：左邊輸入觸發，送出服務後等它「真的做完」才輸出。",
                  18, "#323f4b", "start"))
    s.append(text(x0 + 130, fy + 72, "上出口＝完成（接下一塊）　·　下出口＝失敗（接降落／停止）"
                  "　·　被手動按鈕接手時兩個出口都不輸出，流程安靜停下。", 18, "#323f4b", "start"))
    svg_page("m05_飛航動作接口", s, fy + 96 + 30)


def shoot(name, height):
    """無頭 Chrome 截圖，再把底下多出來的白邊裁掉。"""
    src = os.path.join(HERE, name + ".html")
    out = os.path.join(HERE, name + ".png")
    with tempfile.TemporaryDirectory() as prof:
        subprocess.run(["google-chrome", "--headless=new", "--disable-gpu", "--hide-scrollbars",
                        f"--user-data-dir={prof}", f"--window-size={W},{height}",
                        "--force-device-scale-factor=1", f"--screenshot={out}", "file://" + src],
                       check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    im = Image.open(out).convert("RGB")
    px = im.load()
    bottom = im.height - 1
    while bottom > 0 and all(px[x, bottom] == (255, 255, 255) for x in range(0, im.width, 4)):
        bottom -= 1
    if bottom + 24 < im.height:
        im = im.crop((0, 0, im.width, bottom + 24))
    im.save(out)
    print(f"{out}  {im.width}x{im.height}")


if __name__ == "__main__":
    make_m02()
    make_simple_modules()
    make_simple_flow()
    make_actions()
    shoot("m01_四大模組", 1500)
    shoot("m02_模組串連", 1200)
    shoot("m03_四大模組_簡易", 760)
    shoot("m04_模組串連_簡易", 660)
    shoot("m05_飛航動作接口", 1000)

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""檢查 Node-RED 流程裡有沒有積木在編輯器畫面上重疊。

    python3 test/t_flow_overlap.py                     # 預設檢查兩份 repo 的流程
    python3 test/t_flow_overlap.py ~/.node-red/flows.json

為什麼要有這支：流程檔是程式產生或用腳本改的，座標是人手填的數字，
疊在一起時 JSON 完全看不出來，只有打開編輯器才發現（2026-11-15 Oliver 要求
五個分頁都不能有積木重疊）。

尺寸照 Node-RED 編輯器自己的公式（@node-red/editor-client 的 red.js）：
    寬 = max(100, 20 × ceil((標籤寬 + 50 + 有輸入就 +7) / 20))
    高 = max(30, 輸出數 × 15)
    不顯示標籤的節點（link in/out、l=false）是 30 寬
x、y 是節點的**中心**。標籤寬用 Arial 的字寬表算（編輯器是 14px，
Linux 上落到同尺寸的 Liberation Sans），全形字 14 px。
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULTS = [
    os.path.join(HERE, "..", "nodered", "flows", "drone_basic.json"),
    os.path.join(HERE, "..", "..", "husky_nodered", "nodered", "flows", "husky_basic.json"),
]
MIN_GAP = 0.0          # 邊緣相距小於這個就算重疊（0 = 只要有交集）
NO_INPUT = {"inject", "link in", "catch", "status", "complete", "ui-button",
            "ui-number-input", "ui-text-input", "ui-switch", "ui-slider",
            "ui-dropdown", "comment", "ui-control", "ui-event"}
HIDDEN_LABEL = {"link in", "link out", "junction"}


# Arial（Linux 上是同尺寸的 Liberation Sans）每個字的寬度，單位 1/1000 em。
# 編輯器的標籤是 14px，所以乘 0.014 就是像素。表裡沒有的半形字當 556（數字寬）。
_ARIAL = {
    **{c: 556 for c in "abdeghnopqu0123456789_$#?"},
    **{c: 500 for c in "cksvxyzJ"}, **{c: 278 for c in "ftI/.,:;![] |'"},
    **{c: 222 for c in "ijl"}, "m": 833, "r": 333, "w": 722, "-": 333,
    "(": 333, ")": 333, **{c: 667 for c in "ABEKPSVXY"},
    **{c: 722 for c in "CDHNRUw"}, "F": 611, "T": 611, "Z": 611, "L": 556,
    **{c: 778 for c in "GOQ"}, "M": 833, "W": 944,
}


def text_w(s):
    """標籤寬（px）。全形字（中日韓、全形標點）14 px，表情符號算 18。"""
    w = 0.0
    for c in s:
        o = ord(c)
        if o >= 0x1F000:
            w += 18.0
        elif o > 0x2E80 or 0x2190 <= o <= 0x2BFF:
            w += 14.0
        else:
            w += _ARIAL.get(c, 556) * 0.014
    return w


def label_of(n, subflows):
    if n.get("type", "").startswith("subflow:"):
        return n.get("name") or subflows.get(n["type"].split(":", 1)[1], "subflow")
    for k in ("name", "label", "title"):
        if n.get(k):
            return str(n[k])
    return n.get("type", "")


def box(n, subflows):
    t = n.get("type", "")
    outs = n.get("outputs")
    if outs is None:
        outs = len(n.get("wires") or [])
    h = max(30, int(outs) * 15)
    if t == "junction":
        return 10, 10
    if t in HIDDEN_LABEL or n.get("l") is False:
        return 30, h
    lines = label_of(n, subflows).split("\\n")
    lw = max(text_w(x) for x in lines)
    h = max(h, 6 + 24 * len(lines))
    w = max(100, 20 * math.ceil((lw + 50 + (0 if t in NO_INPUT else 7)) / 20))
    return w, h


def overlaps(nodes):
    subflows = {n["id"]: n.get("name", "") for n in nodes if n.get("type") == "subflow"}
    tabs = {n["id"]: (n.get("label") or n.get("name")) for n in nodes
            if n.get("type") in ("tab", "subflow")}
    by_tab = {}
    for n in nodes:
        if n.get("z") in tabs and "x" in n and "y" in n and n.get("type") != "group":
            w, h = box(n, subflows)
            by_tab.setdefault(n["z"], []).append(
                (n, n["x"] - w / 2, n["y"] - h / 2, n["x"] + w / 2, n["y"] + h / 2))
    out = []
    for z, items in by_tab.items():
        for i in range(len(items)):
            a = items[i]
            for b in items[i + 1:]:
                if (a[1] < b[3] + MIN_GAP and b[1] < a[3] + MIN_GAP
                        and a[2] < b[4] + MIN_GAP and b[2] < a[4] + MIN_GAP):
                    out.append((tabs[z], a[0], b[0]))
    return out


def main():
    paths = sys.argv[1:] or DEFAULTS
    bad = 0
    for p in paths:
        nodes = json.load(open(p, encoding="utf-8"))
        subflows = {n["id"]: n.get("name", "") for n in nodes if n.get("type") == "subflow"}
        hits = overlaps(nodes)
        name = os.path.basename(p)
        if not hits:
            print(f"  [通] {name}：沒有重疊的積木")
            continue
        bad += len(hits)
        print(f"  [✗] {name}：{len(hits)} 組重疊")
        for tab, a, b in hits:
            print(f"        {tab}｜{label_of(a, subflows)!r} ({a['x']},{a['y']}) 和 "
                  f"{label_of(b, subflows)!r} ({b['x']},{b['y']})")
    if bad:
        print(f"✗ 有 {bad} 組積木重疊")
        sys.exit(1)
    print("✓ 流程裡沒有重疊的積木")


if __name__ == "__main__":
    main()

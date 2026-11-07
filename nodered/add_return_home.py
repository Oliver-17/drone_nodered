#!/usr/bin/env python3
"""把「返航」積木併進現有的 drone_basic.json，不動其他任何節點。

為什麼不直接跑 make_drone_basic.py：
    產生器對儀表板那幾個節點**不是**單一來源 —— 地圖 widget、速度錶和
    「拆成畫面欄位」都是在 Node-RED 裡做好之後匯出蓋回來的，產生器做不出來。
    直接重跑會被 _ok_to_overwrite 擋下（正確），--force 則會把它們刪掉。
    所以這支只負責「加上返航相關的節點」，其他原封不動。

產生器那邊的改動（return_home_subflow / 按鈕）仍然留著 —— 哪天真的從
骨架重建時返航會一起出來，兩邊不會分叉。

可以重複執行：先移除舊的返航節點再加回來。
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import make_drone_basic as M                                    # noqa: E402

FLOW = os.path.join(HERE, "flows", "drone_basic.json")
BTN, INST = M.nid(577), M.nid(578)


def main():
    with open(FLOW, encoding="utf-8") as f:
        nodes = json.load(f)
    before = len(nodes)

    # 冪等：把上一次加的全部拿掉（含 subflow 定義、它的內部節點、按鈕和實例）
    drop = {M.RTH_ID, BTN, INST}
    nodes = [n for n in nodes
             if n["id"] not in drop and n.get("z") != M.RTH_ID]
    removed = before - len(nodes)

    # subflow 定義要放在使用它的節點前面（Node-RED 匯入時不強制，但這樣
    # 檔案讀起來和產生器的輸出順序一致）
    rth_nodes = M.return_home_subflow()
    button = M.ui_button(577, "d0c0ffeeuig00004", "🏠 返航", 3, INST, 880, x=300)
    inst = M.rth(578, 880, None, None, x=560, z=M.DASH)

    # 插在最後一個 subflow 定義之後
    last_sf = max(i for i, n in enumerate(nodes) if n["type"] == "subflow")
    nodes[last_sf + 1:last_sf + 1] = rth_nodes
    nodes += [button, inst]

    with open(FLOW, "w", encoding="utf-8") as f:
        json.dump(nodes, f, ensure_ascii=False, indent=2)
        f.write("\n")
    print(f"移除舊的 {removed} 個、加入 {len(rth_nodes) + 2} 個 → 共 {len(nodes)} 個節點")
    return 0


if __name__ == "__main__":
    sys.exit(main())

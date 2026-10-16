#!/usr/bin/env python3
# =============================================================================
#  merge_flows.py —— 更新流程時，只換掉「我們產生的」分頁，保留使用者自己加的
#
#  用法（install_flows.sh 會自己呼叫，平常不用手動跑）：
#      python3 merge_flows.py 新的.json 現有的flows.json 輸出.json
#
#  為什麼需要它：
#      之前 install_flows.sh 是整個檔案換掉，使用者自己拉的任務會不見。
#      但也不能「都不換」—— 那就更新不到新功能。所以照「誰產生的」來分：
#
#        我們產生的（會被換掉）：無人機控制、系統、任務、零件庫，和三個積木的定義
#        使用者自己的（保留）  ：他自己新增的分頁、自己建的設定節點
#
#        使用者放在自己分頁上的**動作積木**也保留 —— 積木的定義會換成新版，
#        但他拉出來的那個實例（含他填的參數、接的線）原地不動
#
#  ⚠️ 例外：如果東西是加在「我們的分頁裡面」，更新時會跟著那一頁被換掉。
#     所以說明裡會叫使用者把自己的東西放在新分頁。
# =============================================================================

import json
import re
import sys


MARKER = "generated-by: drone_nodered"


def base_name(name):
    """去掉 Node-RED 匯入時加的「 (1)」後綴，才認得出重複匯入的副本。"""
    return re.sub(r"\s*\(\d+\)$", "", (name or "").strip())


def is_ours(n, ids, labels):
    """判斷一個分頁／積木是不是本專案的。

    三種認法，缺一不可：
      1. id 一樣 —— 正常用 install_flows.sh 裝的
      2. info 裡有標記 —— 以防以後改 id
      3. 名稱一樣（去掉「 (1)」）—— 使用者用編輯器「匯入」過，
         Node-RED 會重新配 id、還會複製出「ROS2 呼叫服務 (1)」這種副本，
         只靠 id 會認不出來，那些垃圾就永遠清不掉（2026-09-23 實際遇到）
    """
    if n["id"] in ids:
        return True
    if MARKER in (n.get("info") or ""):
        return True
    return base_name(n.get("label") or n.get("name")) in labels


def merge(new_nodes, cur_nodes):
    ids = {n["id"] for n in new_nodes}
    labels = {base_name(n.get("label") or n.get("name"))
              for n in new_nodes if n["type"] in ("tab", "subflow")}

    # 現有檔案裡「屬於本專案」的分頁和積木，它們裡面的節點也要一起換掉
    containers = {n["id"] for n in cur_nodes
                  if n["type"] in ("tab", "subflow") and is_ours(n, ids, labels)}

    kept, dropped = [], 0
    for n in cur_nodes:
        ours = n["id"] in ids or n.get("z") in containers
        if n["type"] in ("tab", "subflow"):
            ours = ours or is_ours(n, ids, labels)
        # 積木的「實例」（使用者從左側面板拖出來的那一個）要不要清掉，
        # 看它指向的定義**更新後還在不在**：
        #   在（id 相同、只是內容換新）→ 留著。使用者放在自己分頁上的積木
        #     不能因為更新就消失（2026-09-28 發現「自訂任務」分頁的積木被清光）
        #   不在（匯入時複製出來的副本，Node-RED 會重新配 id）→ 連實例一起清，
        #     不然會留下指向不存在定義的壞節點
        if n["type"].startswith("subflow:"):
            sid = n["type"][8:]
            if sid in containers and sid not in ids:
                ours = True
        if ours:
            dropped += 1
            continue
        kept.append(n)

    return new_nodes + kept, kept, dropped


def main():
    new_path, cur_path, out_path = sys.argv[1], sys.argv[2], sys.argv[3]
    new_nodes = json.load(open(new_path, encoding="utf-8"))
    cur_nodes = json.load(open(cur_path, encoding="utf-8"))

    merged, kept, dropped = merge(new_nodes, cur_nodes)

    # 保留下來的東西裡，哪些是使用者自己的分頁 —— 印出來讓他確認沒被動到
    tabs = [n.get("label", "(未命名)") for n in kept if n["type"] == "tab"]
    json.dump(merged, open(out_path, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
    print(f"  保留你自己的 {len(kept)} 個節點" + (f"（分頁：{', '.join(tabs)}）" if tabs else ""))
    print(f"  換掉本專案的 {dropped} 個節點")


if __name__ == "__main__":
    main()

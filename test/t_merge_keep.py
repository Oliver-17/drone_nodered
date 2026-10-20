#!/usr/bin/env python3
# =============================================================================
#  t_merge_keep.py —— 驗「更新流程不會動到使用者自己的分頁」
#
#  跑法（不需要 ROS、不需要 Node-RED 在跑）：
#      python3 ~/ros2_ws/src/drone_nodered/test/t_merge_keep.py
#
#  為什麼要有這個測試：
#      2026-09-28 發現 Oliver 的「自訂任務」分頁被清空 —— 分頁和他自己拉的
#      inject／debug 都在，但中間那串動作積木全不見了。原因是 merge_flows.py
#      有一條「指向本專案積木的實例一律清掉」的規則，本意是清掉匯入時複製出來
#      的副本，卻連使用者正常拉出來的那些一起清了。
#      這種 bug 靠肉眼看 JSON 看不出來，所以補一個測試把兩種情況都釘住：
#          使用者拉的積木  → 要留下
#          匯入複製的副本  → 要清掉
# =============================================================================

import json
import os
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
NODERED = os.path.join(HERE, "..", "nodered")
# ⚠️ 刻意不用（見 main() 裡的說明）：跑產生器會覆寫 repo 的 flow 檔。
#    留著這一行是為了讓「為什麼不用它」有地方可說。
GEN = os.path.join(NODERED, "make_drone_basic.py")   # noqa: F841
MERGE = os.path.join(NODERED, "merge_flows.py")
FLOWS = os.path.join(NODERED, "flows", "drone_basic.json")

USER_TAB = "b8027bbad3b3e4af"      # 用真實情境的 id：Oliver 的「自訂任務」分頁
TAKEOFF = "d0c0ffeeaca10000"       # 起飛積木的定義 id（凍結在產生器裡）
# 2026-10-08 新增的積木。加新積木時這個測試要跟著加一筆 ——
# 「新增一個積木定義會不會波及使用者已經拉出來的實例」是獨立的風險，
# 不會因為起飛那一筆通過就自動成立。
GOTO = "d0c0ffeeaca80000"


def fail(msg):
    print(f"✗ {msg}")
    sys.exit(1)


def make_current(generated):
    """組一份「使用者現在的 flows.json」：我們產生的東西 + 他自己的分頁。"""
    user = [
        {"id": USER_TAB, "type": "tab", "label": "自訂任務", "disabled": False, "info": ""},
        {"id": "u0001", "type": "inject", "z": USER_TAB, "name": "▶ 開始任務",
         "x": 150, "y": 100, "wires": [["u0002"]]},
        # 他從左側面板拖出來的「起飛」積木，還改了預設高度
        {"id": "u0002", "type": f"subflow:{TAKEOFF}", "z": USER_TAB, "name": "起飛",
         "env": [{"name": "ALTITUDE", "type": "num", "value": "2.5"}],
         "x": 350, "y": 100, "wires": [["u0003"], ["u0004"]]},
        {"id": "u0003", "type": "debug", "z": USER_TAB, "name": "🏁 任務完成",
         "x": 600, "y": 80, "wires": []},
        {"id": "u0004", "type": "debug", "z": USER_TAB, "name": "失敗 → 已降落",
         "x": 600, "y": 140, "wires": []},
        # 他也拉了一個「導航到座標」並填了走道的座標（2026-10-08 新增的積木）
        {"id": "u0006", "type": f"subflow:{GOTO}", "z": USER_TAB, "name": "導航到座標",
         "env": [{"name": "MAP_X", "type": "num", "value": "0.0"},
                 {"name": "MAP_Y", "type": "num", "value": "-9.0"}],
         "x": 350, "y": 200, "wires": [["u0003"], ["u0004"]]},
        # 用編輯器「匯入」時 Node-RED 複製出來的副本：新的 id、名稱後面多了 (1)
        {"id": "dup00000000dup1", "type": "subflow", "name": "降落 (1)",
         "info": "", "category": "無人機", "in": [], "out": []},
        {"id": "u0005", "type": "subflow:dup00000000dup1", "z": USER_TAB,
         "name": "降落 (1)", "env": [], "x": 350, "y": 300, "wires": [[], []]},
    ]
    return generated + user


def main():
    tmp = tempfile.mkdtemp(prefix="t_merge_")
    # 🔴 這裡**不可以**跑產生器（2026-10-20 修）。
    #
    #    原本是 subprocess.run([sys.executable, GEN]) —— 想確保素材是最新的，
    #    結果變成「跑一次測試就把 flows/drone_basic.json 重新產生一遍」，
    #    手改的東西全部消失。那天就是這樣吃掉了儀表板的地圖 widget：
    #    repo 的檔案在跑測試後安靜地退回 133 個節點，而測試照樣通過，
    #    下一次 install_flows.sh 就把舊版裝進 Node-RED。
    #
    #    產生器是**初始骨架**，不是唯一來源 —— 它自己的檔頭就寫著
    #    「在 Node-RED 裡改了流程就匯出蓋掉 drone_basic.json」。
    #    所以這支測試該用 repo 現在的 flow 當素材，那也才是真正會裝進
    #    Node-RED 的那一份。
    #
    #    ⚠️ 推論：任何測試都不該寫 repo 裡的檔案。要寫就寫 tempfile。
    generated = json.load(open(FLOWS, encoding="utf-8"))

    cur_path = os.path.join(tmp, "current.json")
    out_path = os.path.join(tmp, "merged.json")
    json.dump(make_current(generated), open(cur_path, "w", encoding="utf-8"),
              ensure_ascii=False)

    subprocess.run([sys.executable, MERGE, FLOWS, cur_path, out_path],
                   check=True, capture_output=True)
    merged = json.load(open(out_path, encoding="utf-8"))
    by_id = {n["id"]: n for n in merged}

    # --- 使用者自己的東西：全部要在 -------------------------------------------
    for i, what in (("b8027bbad3b3e4af", "自訂任務分頁"), ("u0001", "他的 inject"),
                    ("u0002", "他拉的起飛積木"), ("u0003", "他的 debug（完成）"),
                    ("u0004", "他的 debug（失敗）"),
                    ("u0006", "他拉的導航到座標積木")):
        if i not in by_id:
            fail(f"{what} 被清掉了")

    # 積木上填的參數也要留著 —— 只換定義，不動實例
    env = by_id["u0002"].get("env") or []
    if not any(e["name"] == "ALTITUDE" and e["value"] == "2.5" for e in env):
        fail(f"他在積木上填的高度不見了：{env}")

    # 接線要完好：積木還指著他的兩個 debug
    if by_id["u0002"]["wires"] != [["u0003"], ["u0004"]]:
        fail(f"接線變了：{by_id['u0002']['wires']}")

    # 新積木上填的座標也要留著
    genv = by_id["u0006"].get("env") or []
    if not any(e["name"] == "MAP_Y" and e["value"] == "-9.0" for e in genv):
        fail(f"他在導航積木上填的座標不見了：{genv}")

    # 積木的定義要有被換成新版（同一個 id 在合併結果裡只能有一份）
    for bid, what in ((TAKEOFF, "起飛"), (GOTO, "導航到座標")):
        if sum(1 for n in merged if n["id"] == bid) != 1:
            fail(f"{what}積木的定義重複或消失")

    # --- 匯入複製出來的副本：要清掉 -------------------------------------------
    if "dup00000000dup1" in by_id:
        fail("匯入複製的積木定義沒有被清掉")
    if "u0005" in by_id:
        fail("指向那份副本的實例沒有被清掉（會變成壞節點）")

    # --- 我們自己的分頁：要被換成新版，不能留下舊的重複 --------------------------
    ids = [n["id"] for n in merged]
    dup = {i for i in ids if ids.count(i) > 1}
    if dup:
        fail(f"合併後有重複 id：{dup}")

    print(f"✓ 使用者的分頁、積木（含新增的導航積木）、參數、接線都留下來了"
          f"（{len(merged)} 個節點）")
    print("✓ 匯入複製的副本和它的實例都清掉了")
    print("✓ 沒有重複 id")


if __name__ == "__main__":
    main()

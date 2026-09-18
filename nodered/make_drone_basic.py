#!/usr/bin/env python3
# =============================================================================
#  make_drone_basic.py — 產生 flows/drone_basic.json（無人機的 Node-RED 操作流程）
#
#  用法：
#      python3 ~/ros2_ws/src/drone_nodered/nodered/make_drone_basic.py
#
#  為什麼要用產生的，不直接手寫 JSON：
#      drone_basic.json 會「把 nodered_ros2 的兩個積木一起包進去」，這樣匯入時只要一個檔，
#      不會因為忘了先匯入積木而出現「未知節點」。
#      但包進去的那份如果手動複製，改了 nodered_ros2 的積木之後兩邊就不一樣了。
#      所以積木一律從 nodered_ros2 讀進來。改了積木 → 重跑這支。
#
#  在 Node-RED 裡改了流程想存回來：選單 → 匯出 → 蓋掉 drone_basic.json。
#  （那樣會連同積木一起匯出；之後要改積木本身，請改 nodered_ros2 的檔再重跑這支。）
# =============================================================================
import json
import os

HERE = os.path.dirname(os.path.abspath(__file__))
SUBFLOWS = os.path.join(HERE, "..", "..", "nodered_ros2", "nodered", "subflows")
OUT = os.path.join(HERE, "flows", "drone_basic.json")

CALL = "f1a0c0de00000001"   # ros2_call_service.json 裡的 subflow id
SUB = "f1a0c0de00000002"    # ros2_subscribe.json 裡的 subflow id
TAB = "d0c0ffee00000000"
# 第二個分頁：開關模擬。
# ⚠️ 不能用 nid() 產得出來的號碼 —— 2026-09-18 這裡本來寫 d0c0ffee00000001，
#    和說明註解 nid(1) 完全一樣，Node-RED 看到重複 id 就把「系統」分頁吃掉，
#    兩頁的節點全疊在同一頁上。改用 nid() 不會用到的尾碼。
SYS = "d0c0ffeeffff0001"
NS = "MAV1"


def nid(i):
    return f"d0c0ffee{i:08x}"


def load_subflows():
    nodes = []
    for name in ("ros2_call_service.json", "ros2_subscribe.json"):
        with open(os.path.join(SUBFLOWS, name), encoding="utf-8") as f:
            nodes += json.load(f)
    ids = {n["id"] for n in nodes if n["type"] == "subflow"}
    assert {CALL, SUB} <= ids, f"nodered_ros2 的 subflow id 變了：{ids}"
    return nodes


def inject(i, name, payload, target, y):
    return {"id": nid(i), "type": "inject", "z": TAB, "name": name,
            "props": [{"p": "payload"}], "repeat": "", "crontab": "", "once": False,
            "onceDelay": 0.1, "topic": "", "payload": json.dumps(payload, ensure_ascii=False),
            "payloadType": "json", "x": 170, "y": y, "wires": [[target]]}


def call(i, service, target, y):
    return {"id": nid(i), "type": f"subflow:{CALL}", "z": TAB, "name": service,
            "env": [{"name": "SERVICE", "type": "str", "value": service},
                    {"name": "TIMEOUT", "type": "num", "value": "10"}],
            "x": 480, "y": y, "wires": [[target]]}


def sys_inject(i, name, target, y):
    """「系統」分頁的按鈕。

    payload 留空、topic 放按鈕名稱 —— exec 節點的 addpay 設成空字串（不把 payload
    接到指令後面），所以按鈕只是「觸發」，真正要跑什麼寫在 exec 裡。
    """
    return {"id": nid(i), "type": "inject", "z": SYS, "name": name,
            "props": [{"p": "topic", "vt": "str"}], "repeat": "", "crontab": "", "once": False,
            "onceDelay": 0.1, "topic": name, "payload": "", "payloadType": "str",
            "x": 160, "y": y, "wires": [[target]]}


def sys_exec(i, cmd, target, y, name=""):
    """執行一行指令。

    為什麼交給 systemd 而不是直接在這裡開 PX4：exec 節點只會對「自己直接開的那個程序」
    送停止訊號（90-exec.js:45），重新 Deploy 或重啟 Node-RED 就可能把模擬砍一半、
    或留下收不掉的殘留。改成叫 systemctl，指令一秒就結束，真正看管模擬的是 systemd。
    """
    return {"id": nid(i), "type": "exec", "z": SYS, "command": cmd,
            # addpay 空字串 = 不要把 msg.payload 接到指令尾巴（不然按鈕的內容會變成參數）
            "addpay": "", "append": "", "useSpawn": "false", "timer": "20",
            "winHide": False, "oldrc": False, "name": name,
            "x": 520, "y": y,
            # 三個輸出：stdout、stderr、回傳碼。前兩個都送去整理，回傳碼不用
            "wires": [[target], [target], []]}


def debug(i, name, y, x=860, z=TAB):
    return {"id": nid(i), "type": "debug", "z": z, "name": name, "active": True,
            "tosidebar": True, "console": False, "tostatus": False, "complete": "payload",
            "targetType": "msg", "statusVal": "", "statusType": "auto", "x": x, "y": y, "wires": []}


def function(i, name, code, target, y, x=450, z=TAB):
    return {"id": nid(i), "type": "function", "z": z, "name": name, "func": code,
            "outputs": 1, "timeout": 0, "noerr": 0, "initialize": "", "finalize": "",
            "libs": [], "x": x, "y": y, "wires": [[target]]}


STATUS_CODE = r'''// 把 drone_api_node 的狀態 JSON 整理成好讀的樣子
let s;
try {
    s = JSON.parse(msg.payload.data);   // std_msgs/String 的內容在 data 欄位
} catch (e) {
    return null;
}
if (!s.connected) {
    msg.payload = { 連線: false, 說明: "drone_api_node 沒收到 PX4 的狀態" };
    msg.key = "未連線";
    return msg;
}
msg.payload = { 模式: s.mode, 解鎖: s.armed, 落地: s.landed, 高度_m: s.altitude_m };
// 巡航中才多顯示進度（第幾個點／共幾個點）
const pt = s.patrol || {};
if (pt.active) {
    msg.payload.巡航 = `${pt.index}/${pt.total} 點`;
}
// 下一個 filter 節點「只比對 msg.key」：模式、解鎖、落地有變才輸出。
// 高度不放進 key —— 它每 0.5 秒都在變，放進去的話 debug 側欄會被洗版。
// 巡航進度也放進 key：飛到下一個點時要看得到，不然中間十幾秒完全沒訊息
msg.key = `${s.mode}|${s.armed}|${s.landed}|${(s.patrol || {}).index}`;
return msg;
'''

LOG_CODE = r'''// 只留 drone_api_node 的 log，格式簡化成「[等級] 內容」
// /rosout 會收到「所有」節點的 log（rosbridge 自己也一直在印），所以要過濾。
// 多機時把 MAV1 改成對應的 namespace。
const NAME = "MAV1.drone_api_node";
const log = msg.payload;                 // rcl_interfaces/msg/Log
if (!log || log.name !== NAME) return null;
const LEVEL = { 10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL" };
msg.topic = log.name;
msg.payload = `[${LEVEL[log.level] || log.level}] ${log.msg}`;
return msg;
'''

SYS_CODE = r'''// 整理 systemctl / journalctl 的輸出，讓 Debug 側欄看得懂
// exec 節點的第 1 個輸出是 stdout、第 2 個是 stderr，兩個都接到這裡。
const raw = (msg.payload === undefined || msg.payload === null ? "" : msg.payload)
    .toString().trim();
if (!raw) return null;              // 指令成功但沒輸出的情況，不用顯示

// systemctl is-active 只會印一個字，翻譯成看得懂的句子
const STATE = {
    "active": "模擬正在跑",
    "inactive": "模擬沒在跑",
    "failed": "模擬啟動失敗 —— 按「看模擬 log」查原因",
    "activating": "模擬啟動中…",
    "deactivating": "模擬關閉中…",
};
msg.payload = STATE[raw] || raw;
return msg;
'''

SYS_HELP = """### 這個分頁在做什麼
點按鈕 → 執行一行 `systemctl` → 由 **systemd** 去開關模擬。

Node-RED 只負責「下指令」，不負責「看管模擬」。所以你按 Deploy 或重啟 Node-RED，
Gazebo 都不會被誤殺。

### 按鈕
| 按鈕 | 說明 |
| --- | --- |
| 開模擬 | Gazebo 視窗會跳出來。**PX4 約需 15 秒**才連上，狀態才會出現 |
| 關模擬 | 只關模擬，控制台留著 |
| 模擬狀態 | 現在有沒有在跑 |
| 看模擬 log | 最後 40 行。開不起來時看這個 |
| 全部關閉 | 模擬和控制台一起關（**Node-RED 自己也會關掉**，等於桌面的「停」圖示） |

### 看不到無人機？
空白世界沒有參照物，預設鏡頭在 6 公尺外，x500 只有 0.5 公尺，看起來像一個點。
滑鼠滾輪推近，或在右側 Entity Tree 的 `x500_0` 上按右鍵 → Move to。
"""

HELP = """### 怎麼開始
桌面點「無人機控制台（開）」→ 這頁的按鈕就能用了。

### 這個流程更新了怎麼辦
**不要用編輯器的「匯入」**（會問你重複 id，選錯就變成兩份，很難清）。跑這行：
```
~/ros2_ws/src/drone_nodered/scripts/install_flows.sh
```
它會先備份你現在的流程、換成新版、把控制台重開，然後 F5 重新整理就好。
Gazebo 要另外在 **「系統」分頁**按「開模擬」，開完等約 15 秒。

### 怎麼用
- 點「起飛 1.5 m」「盤旋」「降落」左邊的小方塊
- 結果、狀態、log 都在右側的 **Debug** 分頁（蟲子圖示）
- 積木下方顯示「已連線」才能用；「未連線」代表 rosbridge 沒開

### 改高度
- **起飛時的高度**：雙擊「起飛 1.5 m」→ 改 payload 的 `1.5` → 完成 → **Deploy**。
- **飛到一半要改**：按「調整高度 3 m」（同樣可以雙擊改數字）。水平位置不會跑掉，
  改完就在新高度盤旋。**要先起飛**，在地上按會被拒絕。

兩個都限制 0.5–5 m（drone_api_node 的參數），超出會被拒絕。

### 狀態查詢
按「狀態查詢」會回報**按下去那一刻**的位置、高度、速度、朝向、模式。
每一項都先檢查有效旗標，無效會寫「無效」而不是給你一個看似正常的數字。

### 巡航
「巡航（10 m 正方形）」會依序飛 (10,0) → (10,10) → (0,10) → (0,0)，飛完停在原點盤旋。
- **座標**：離起飛點的公尺數，x 北為正、y 東為正。雙擊按鈕可以改成自己的路線
  （最多 20 點、±50 m、速度 0.5–5 m/s）。
- **中途要停**：按「盤旋」會就地煞停；按「降落」會停下來並降落。
- **進度**：狀態那一欄會顯示「巡航 2/4 點」。
- **要先起飛**，而且室內用動捕（沒有有效經緯度）時不能用 —— 會回訊息說明。

### 狀態為什麼不是每 0.5 秒一筆
只有「模式、解鎖、落地」改變時才顯示，避免洗版。顯示時會附上當下的高度。
"""


def main():
    nodes = load_subflows()
    nodes.append({"id": TAB, "type": "tab", "label": f"無人機控制（{NS}）", "disabled": False,
                  "info": "起飛／盤旋／降落 + 狀態 + log。由 drone_nodered/nodered/make_drone_basic.py 產生。"})
    nodes.append({"id": nid(1), "type": "comment", "z": TAB, "name": "使用說明（點我，看右側「資訊」分頁）",
                  "info": HELP, "x": 220, "y": 40, "wires": []})

    # 三個指令：按鈕 → 呼叫服務 → debug
    for k, (label, payload, action, y) in enumerate([
            # 狀態查詢放第一個：最常按，而且它不會動到飛機
            ("狀態查詢", {}, "state", 120),
            ("起飛 1.5 m", {"altitude": 1.5}, "takeoff", 210),
            ("盤旋", {}, "hover", 300),
            # 空中改高度。水平位置不動，改完 PX4 自己轉成盤旋（DO_CHANGE_ALTITUDE）。
            # 要先起飛，在地面上按會被拒絕（PX4 沒解鎖時不會執行這個指令）。
            ("調整高度 3 m", {"altitude": 3.0}, "set_altitude", 390),
            # 巡航：正方形 10 m。x 北為正、y 東為正，座標是「離起飛點」的公尺數。
            # 服務只負責「開始」，飛行在背景進行，進度看狀態裡的 patrol 欄位。
            ("巡航（10 m 正方形）",
             {"x": [10, 10, 0, 0], "y": [0, 10, 10, 0],
              "altitude": 3.0, "speed": 2.0, "loops": 1}, "patrol", 480),
            ("降落", {}, "land", 570)]):
        b = 10 + k * 10
        nodes += [inject(b, label, payload, nid(b + 1), y),
                  call(b + 1, f"/{NS}/api/{action}", nid(b + 2), y),
                  debug(b + 2, f"{label}：結果", y)]

    # 狀態和 log 的編號從 200 開始：上面的按鈕用 10, 20, 30… 每加一顆就往後長，
    # 以前放在 50/60 會被第五顆按鈕撞到（2026-09-18 踩過，下面的 assert 就是為此加的）。
    # 狀態：訂閱 → 整理 → 只在變化時 → debug
    nodes += [
        {"id": nid(200), "type": f"subflow:{SUB}", "z": TAB, "name": f"/{NS}/api/status",
         "env": [{"name": "TOPIC", "type": "str", "value": f"/{NS}/api/status"},
                 {"name": "TYPE", "type": "str", "value": "std_msgs/msg/String"}],
         "x": 170, "y": 670, "wires": [[nid(201)]]},
        function(201, "整理狀態", STATUS_CODE, nid(202), 670, x=460),
        {"id": nid(202), "type": "rbe", "z": TAB, "name": "只在模式／解鎖／落地變化時",
         "func": "rbe", "gap": "", "start": "", "inout": "out", "septopics": False,
         "property": "key", "topi": "topic", "x": 700, "y": 670, "wires": [[nid(203)]]},
        debug(203, "狀態", 670, x=1000),
    ]

    # log：訂閱 /rosout → 只留 drone_api_node → debug
    nodes += [
        {"id": nid(210), "type": f"subflow:{SUB}", "z": TAB, "name": "/rosout",
         "env": [{"name": "TOPIC", "type": "str", "value": "/rosout"},
                 {"name": "TYPE", "type": "str", "value": "rcl_interfaces/msg/Log"}],
         "x": 160, "y": 760, "wires": [[nid(211)]]},
        function(211, "只留 drone_api_node", LOG_CODE, nid(212), 760, x=460),
        debug(212, "log", 760),
    ]

    # --- 「系統」分頁：開關模擬 ------------------------------------------------
    nodes.append({"id": SYS, "type": "tab", "label": "系統", "disabled": False,
                  "info": "用 systemd 開關模擬。由 make_drone_basic.py 產生。"})
    nodes.append({"id": nid(100), "type": "comment", "z": SYS,
                  "name": "說明（點我，看右側「資訊」分頁）",
                  "info": SYS_HELP, "x": 220, "y": 40, "wires": []})

    # 五個按鈕共用一個 debug，訊息才不會散在好幾個地方
    sys_debug = nid(101)
    sys_fn = nid(102)

    # 指令都自己 echo 一句話：systemctl 成功時是靜默的，沒有回饋使用者會以為沒反應。
    # 「全部關閉」用 --no-block：它會關掉包含 Node-RED 在內的底座，
    # 若等 systemd 完成，systemctl 自己會先被收掉。--no-block 是送出就返回。
    buttons = [
        ("開模擬", "systemctl --user start drone-sim.service "
                   "&& echo '已送出啟動。Gazebo 視窗會跳出來，PX4 約 15 秒後才連上' "
                   "|| echo '啟動失敗，按「看模擬 log」看原因'", 120),
        ("關模擬", "systemctl --user stop drone-sim.service && echo '模擬已關閉' "
                   "|| echo '關閉失敗'", 220),
        ("模擬狀態", "systemctl --user is-active drone-sim.service", 320),
        ("看模擬 log", "journalctl --user -u drone-sim.service -n 40 --no-pager", 420),
        ("全部關閉", "systemctl --user --no-block stop drone-nodered.service "
                     "&& echo '已送出關閉：模擬和控制台都會停，這個畫面也會失去連線'", 520),
    ]
    for k, (label, cmd, y) in enumerate(buttons):
        b = 110 + k * 10
        nodes += [sys_inject(b, label, nid(b + 1), y),
                  sys_exec(b + 1, cmd, sys_fn, y, name=label)]
    nodes += [
        function(102, "整理輸出", SYS_CODE, sys_debug, 320, x=820, z=SYS),
        debug(101, "系統訊息", 320, x=1020, z=SYS),
    ]

    # 自動檢查 id 有沒有重複 —— 上面那個坑就是這樣來的，靠眼睛看不出來
    ids = [n["id"] for n in nodes if "id" in n]
    dup = {i for i in ids if ids.count(i) > 1}
    assert not dup, f"節點 id 重複：{dup}"

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w", encoding="utf-8") as f:
        json.dump(nodes, f, ensure_ascii=False, indent=2)
    print(f"寫出 {os.path.normpath(OUT)}（{len(nodes)} 個節點，含兩個積木的定義）")


if __name__ == "__main__":
    main()

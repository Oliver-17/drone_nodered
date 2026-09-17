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
            "payloadType": "json", "x": 150, "y": y, "wires": [[target]]}


def call(i, service, target, y):
    return {"id": nid(i), "type": f"subflow:{CALL}", "z": TAB, "name": service,
            "env": [{"name": "SERVICE", "type": "str", "value": service},
                    {"name": "TIMEOUT", "type": "num", "value": "10"}],
            "x": 420, "y": y, "wires": [[target]]}


def debug(i, name, y, x=700):
    return {"id": nid(i), "type": "debug", "z": TAB, "name": name, "active": True,
            "tosidebar": True, "console": False, "tostatus": False, "complete": "payload",
            "targetType": "msg", "statusVal": "", "statusType": "auto", "x": x, "y": y, "wires": []}


def function(i, name, code, target, y, x=450):
    return {"id": nid(i), "type": "function", "z": TAB, "name": name, "func": code,
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
// 下一個 filter 節點「只比對 msg.key」：模式、解鎖、落地有變才輸出。
// 高度不放進 key —— 它每 0.5 秒都在變，放進去的話 debug 側欄會被洗版。
msg.key = `${s.mode}|${s.armed}|${s.landed}`;
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

HELP = """### 用之前要先開這些（各開一個終端）
```
~/ros2_ws/src/drone_nodered/scripts/start_empty_sitl.sh
MicroXRCEAgent udp4 -p 8888
ros2 launch drone_nodered drone_api.launch.py
ros2 launch nodered_ros2 rosbridge.launch.py
```

### 怎麼用
- 點「起飛 1.5 m」「盤旋」「降落」左邊的小方塊
- 結果、狀態、log 都在右側的 **Debug** 分頁（蟲子圖示）
- 積木下方顯示「已連線」才能用；「未連線」代表 rosbridge 沒開

### 改起飛高度
雙擊「起飛 1.5 m」→ 把 payload 的 `1.5` 改掉 → 完成 → 右上角 **Deploy**。
允許範圍 0.5–5 m（drone_api_node 的參數），超出會被拒絕。

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
            ("起飛 1.5 m", {"altitude": 1.5}, "takeoff", 100),
            ("盤旋", {}, "hover", 160),
            ("降落", {}, "land", 220)]):
        b = 10 + k * 10
        nodes += [inject(b, label, payload, nid(b + 1), y),
                  call(b + 1, f"/{NS}/api/{action}", nid(b + 2), y),
                  debug(b + 2, f"{label}：結果", y)]

    # 狀態：訂閱 → 整理 → 只在變化時 → debug
    nodes += [
        {"id": nid(50), "type": f"subflow:{SUB}", "z": TAB, "name": f"/{NS}/api/status",
         "env": [{"name": "TOPIC", "type": "str", "value": f"/{NS}/api/status"},
                 {"name": "TYPE", "type": "str", "value": "std_msgs/msg/String"}],
         "x": 170, "y": 300, "wires": [[nid(51)]]},
        function(51, "整理狀態", STATUS_CODE, nid(52), 300, x=400),
        {"id": nid(52), "type": "rbe", "z": TAB, "name": "只在模式／解鎖／落地變化時",
         "func": "rbe", "gap": "", "start": "", "inout": "out", "septopics": False,
         "property": "key", "topi": "topic", "x": 640, "y": 300, "wires": [[nid(53)]]},
        debug(53, "狀態", 300, x=880),
    ]

    # log：訂閱 /rosout → 只留 drone_api_node → debug
    nodes += [
        {"id": nid(60), "type": f"subflow:{SUB}", "z": TAB, "name": "/rosout",
         "env": [{"name": "TOPIC", "type": "str", "value": "/rosout"},
                 {"name": "TYPE", "type": "str", "value": "rcl_interfaces/msg/Log"}],
         "x": 150, "y": 360, "wires": [[nid(61)]]},
        function(61, "只留 drone_api_node", LOG_CODE, nid(62), 360, x=400),
        debug(62, "log", 360),
    ]

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w", encoding="utf-8") as f:
        json.dump(nodes, f, ensure_ascii=False, indent=2)
    print(f"寫出 {os.path.normpath(OUT)}（{len(nodes)} 個節點，含兩個積木的定義）")


if __name__ == "__main__":
    main()

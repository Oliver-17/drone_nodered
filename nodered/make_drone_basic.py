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
MISSION = "d0c0ffeeffff0002"   # 第三個分頁：把動作串成任務
NS = "MAV1"


def nid(i):
    return f"d0c0ffee{i:08x}"


def load_subflows():
    nodes = []
    for name in ("ros2_call_service.json", "ros2_subscribe.json"):
        with open(os.path.join(SUBFLOWS, name), encoding="utf-8") as f:
            nodes += json.load(f)
    # 設了分類，積木才會出現在左側面板（可以像內建節點一樣拖出來），
    # 沒設的話只會躲在「子流程」那一區
    for n in nodes:
        if n["type"] == "subflow":
            n["category"] = "ROS 2"
    ids = {n["id"] for n in nodes if n["type"] == "subflow"}
    assert {CALL, SUB} <= ids, f"nodered_ros2 的 subflow id 變了：{ids}"
    return nodes


def inject(i, name, payload, target, y, x=170, z=TAB):
    return {"id": nid(i), "type": "inject", "z": z, "name": name,
            "props": [{"p": "payload"}], "repeat": "", "crontab": "", "once": False,
            "onceDelay": 0.1, "topic": "", "payload": json.dumps(payload, ensure_ascii=False),
            "payloadType": "json", "x": x, "y": y, "wires": [[target]]}


def call(i, service, target, y, x=480, z=TAB):
    return {"id": nid(i), "type": f"subflow:{CALL}", "z": z, "name": service,
            "env": [{"name": "SERVICE", "type": "str", "value": service},
                    {"name": "TIMEOUT", "type": "num", "value": "10"}],
            "x": x, "y": y, "wires": [[target]]}


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
// 寫進共用區：「等待完成」積木從這裡讀，就不用每個積木各開一條連線。
// 用 global 不用 flow —— flow context 是「每個分頁、每個 subflow 各一份」，
// 任務分頁和積木內部都讀不到控制分頁的 flow。
// 加上時間戳：等待積木只認「它開始等之後才發布的」那一筆，才不會讀到上一個動作的結果。
s._t = Date.now();
global.set("drone_status", s);

msg.payload = { 模式: s.mode, 解鎖: s.armed, 落地: s.landed, 高度_m: s.altitude_m };
// 看門狗觸發時要看得到，這是自動降落的唯一提示
if (s.watchdog && s.watchdog.tripped) {
    msg.payload.看門狗 = s.watchdog.reason;
}
if (s.task && s.task.state === "failed") {
    msg.payload.任務失敗 = `${s.task.name}：${s.task.message}`;
}
// 巡航中才多顯示進度（第幾個點／共幾個點）
const pt = s.patrol || {};
if (pt.active) {
    msg.payload.巡航 = `${pt.index}/${pt.total} 點`;
}
// 下一個 filter 節點「只比對 msg.key」：模式、解鎖、落地有變才輸出。
// 高度不放進 key —— 它每 0.5 秒都在變，放進去的話 debug 側欄會被洗版。
// 巡航進度也放進 key：飛到下一個點時要看得到，不然中間十幾秒完全沒訊息
msg.key = `${s.mode}|${s.armed}|${s.landed}|${(s.patrol || {}).index}` +
          `|${(s.task || {}).seq}|${(s.task || {}).state}|${(s.watchdog || {}).tripped}`;
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

### 🛑 緊急降落
最上面那顆，直接呼叫降落：取消巡航 → 就地煞停 → 降落。

⚠️ 它要經過瀏覽器 → Node-RED → rosbridge → ROS，任何一層卡住就沒用。
**真正的最後防線是遙控器**，這顆是方便，不是保命。

### 狀態為什麼不是每 0.5 秒一筆
只有「模式、解鎖、落地」改變時才顯示，避免洗版。顯示時會附上當下的高度。
"""


WAIT_ID = "d0c0ffeefff10001"     # 「等待完成」積木（subflow 定義）的 id

WAIT_CODE = r'''// 等 drone_api_node 回報「上一個動作做完了」
//
// 為什麼不是等服務回應就好：服務是「下單」不是「送達」。按下起飛，服務 0.05 秒就回
// 「已開始起飛」，但飛機要 20 秒才爬到。真正的完成寫在狀態的 task 欄位裡。
//
// 為什麼要看時間戳：狀態每 0.5 秒才發一次，剛進來時讀到的可能還是「上一個動作」的
// 結果（已完成），那會讓這個積木立刻放行。所以只認「開始等之後才發布」的那一筆。
// 服務當場就被拒絕（參數錯、飛機在地上…），不用等，直接走失敗
if (msg.payload && msg.payload.success === false) {
    node.status({ fill: "red", shape: "ring", text: "被拒絕" });
    return { payload: { 失敗: msg.payload.message || "服務呼叫被拒絕" } };
}

const t0 = Date.now();
const limit = Number(env.get("TIMEOUT") || 120) * 1000;
// 送指令前的任務編號。要等到「編號和它不一樣」才算是這次的任務 ——
// 否則指令沒送成功時，會讀到上一個已完成的任務就放行（2026-09-24 踩到）。
const before = msg._seq_before;
let seq = null;

const timer = setInterval(() => {
    const s = global.get("drone_status");
    if (!s || !s._t || s._t < t0) {                 // 還沒有新鮮的狀態
        if (Date.now() - t0 > limit) { stop(); fail("等不到狀態更新"); }
        return;
    }
    const task = s.task || {};
    if (seq === null) {
        if (task.seq === before) { return; }        // 還是上一個任務，繼續等
        seq = task.seq;                             // 鎖定這一次的任務
    }
    if (task.seq !== seq) {
        // 有人按了別的按鈕 —— 這不是「失敗」，是使用者接手了。
        // 什麼都不輸出，流程安靜停在這裡，不要自作主張去降落。
        stop();
        node.status({ fill: "grey", shape: "ring", text: "已被手動指令接手" });
        node.warn("任務已被手動指令接手，流程停止");
        return;
    }

    if (task.state === "done") {
        stop();
        msg.payload = { 完成: task.name, 說明: task.message };
        node.send([msg, null]);
    } else if (task.state === "failed") {
        stop();
        fail(`${task.name}：${task.message}`);
    } else if (Date.now() - t0 > limit) {
        stop();
        fail(`等待逾時（${limit / 1000} 秒）`);
    } else {
        node.status({ fill: "blue", shape: "dot", text: `等 ${task.name}…` });
    }
}, 200);

function stop() { clearInterval(timer); }
function fail(why) {
    node.status({ fill: "red", shape: "ring", text: why });
    msg.payload = { 失敗: why };
    node.send([null, msg]);      // 第二個出口＝失敗，接你想要的處置
}
'''


def wait_subflow():
    """「等待完成」積木本身的定義。內部只有一個 function 節點，兩個出口。"""
    fn = "d0c0ffeefff10002"
    return [
        {"id": WAIT_ID, "type": "subflow", "name": "等待完成",
         "info": "等上一個動作真的做完。左邊接動作、上面出口接下一步、下面出口接失敗處置。",
         "category": ACT_CATEGORY, "color": "#C0DEED",
         "in": [{"x": 50, "y": 60, "wires": [{"id": fn}]}],
         "out": [{"x": 420, "y": 40, "wires": [{"id": fn, "port": 0}]},
                 {"x": 420, "y": 100, "wires": [{"id": fn, "port": 1}]}],
         "outputLabels": ["完成", "失敗"],
         "env": [{"name": "TIMEOUT", "type": "num", "value": "120",
                  "ui": {"label": {"zh-TW": "逾時（秒）"}, "type": "input",
                         "opts": {"types": ["num"]}}}]},
        {"id": fn, "type": "function", "z": WAIT_ID, "name": "等任務完成",
         "func": WAIT_CODE, "outputs": 2, "timeout": 0, "noerr": 0,
         "initialize": "", "finalize": "", "libs": [],
         "x": 230, "y": 70, "wires": [[], []]},
    ]


def wait(i, y, done_target, fail_target, timeout=120, x=700, z=None):
    return {"id": nid(i), "type": f"subflow:{WAIT_ID}", "z": z, "name": "等待完成",
            "env": [{"name": "TIMEOUT", "type": "num", "value": str(timeout)}],
            "x": x, "y": y,
            # 零件庫裡的積木還沒接線，出口留空
            "wires": [[done_target] if done_target else [],
                      [fail_target] if fail_target else []]}


# --- 動作積木：一個方塊就是「送指令 + 等它做完」---------------------------------
# 內部結構（和使用者原本要自己拉的三件組完全一樣，只是包起來）：
#     輸入 → 組參數 → ROS2 呼叫服務 → 等任務完成 → 完成／失敗兩個出口
# category 設了之後，這些積木會直接出現在左側面板，可以像內建節點一樣拖出來用。
ACT_CATEGORY = "無人機"

PATROL_BUILD = (
    'msg.payload = {\n'
    '    x: env.get("POINTS_X"), y: env.get("POINTS_Y"),\n'
    '    altitude: Number(env.get("ALTITUDE")),\n'
    '    speed: Number(env.get("SPEED")), loops: Number(env.get("LOOPS")),\n'
    '};'
)

# 每個動作：id 尾碼、名稱、服務、預設逾時、可設定的欄位、組參數的程式
ACTIONS = [
    ("a1", "起飛", "takeoff", 90,
     [("ALTITUDE", "num", "1.5", "高度（公尺，0.5～5）")],
     'msg.payload = { altitude: Number(env.get("ALTITUDE")) };'),
    ("a2", "調整高度", "set_altitude", 60,
     [("ALTITUDE", "num", "3.0", "高度（公尺，0.5～5）")],
     'msg.payload = { altitude: Number(env.get("ALTITUDE")) };'),
    ("a3", "巡航", "patrol", 300,
     [("POINTS_X", "json", "[5,5,0,0]", "路線 x（公尺，北為正）"),
      ("POINTS_Y", "json", "[0,5,5,0]", "路線 y（公尺，東為正）"),
      ("ALTITUDE", "num", "3.0", "高度（公尺，0 = 維持現在）"),
      ("SPEED", "num", "2.0", "速度（m/s，0.5～5）"),
      ("LOOPS", "num", "1", "繞幾圈")],
     PATROL_BUILD),
    ("a4", "盤旋", "hover", 30, [], "msg.payload = {};"),
    ("a5", "降落", "land", 150, [], "msg.payload = {};"),
    ("a6", "狀態查詢", "state", 30, [], "msg.payload = {};"),
]


def act_id(tag, n=0):
    return f"d0c0ffeeac{tag}{n:04x}"


def action_subflows():
    """六個動作積木的定義。"""
    out = []
    for tag, label, action, timeout, envs, build in ACTIONS:
        sid, fn_in, inst, fn_wait = act_id(tag), act_id(tag, 1), act_id(tag, 2), act_id(tag, 3)
        env_def = []
        for name, typ, value, label_zh in envs:
            env_def.append({"name": name, "type": typ, "value": value,
                            "ui": {"label": {"zh-TW": label_zh}, "type": "input",
                                   "opts": {"types": [typ]}}})
        env_def.append({"name": "TIMEOUT", "type": "num", "value": str(timeout),
                        "ui": {"label": {"zh-TW": "逾時（秒）"}, "type": "input",
                               "opts": {"types": ["num"]}}})
        info = (f"送出「{label}」並等它真的做完。\n\n"
                "上面出口＝完成、下面出口＝失敗（逾時或被拒絕）。\n"
                "被手動指令接手時兩個出口都不輸出，流程安靜停下。")
        out += [
            {"id": sid, "type": "subflow", "name": label, "info": info,
             "category": ACT_CATEGORY, "color": "#C0DEED",
             "in": [{"x": 40, "y": 80, "wires": [{"id": fn_in}]}],
             "out": [{"x": 560, "y": 40, "wires": [{"id": fn_wait, "port": 0}]},
                     {"x": 560, "y": 120, "wires": [{"id": fn_wait, "port": 1}]}],
             "outputLabels": ["完成", "失敗"], "env": env_def},
            {"id": fn_in, "type": "function", "z": sid, "name": "組參數",
             # 先用積木設定的值組出參數，再讓「傳進來的 payload」覆蓋。
             # 這樣同一個積木：流程裡用固定值、儀表板上可以用畫面上輸入的值。
             "func": ("// 先記下傳進來的內容，下面那行會把 msg.payload 換掉\n"
                      "const incoming = (msg.payload && typeof msg.payload === 'object')\n"
                      "    ? msg.payload : {};\n"
                      "// 欄位值來自積木的設定（雙擊積木就能改）\n"
                      + build +
                      "\n// 傳進來的值可以覆蓋，但**只限這個動作本來就有的欄位**。\n"
                      "// 不能整包 assign —— 上一個積木送來的是 {完成:…,說明:…}，\n"
                      "// 整包塞進去會變成服務不認得的欄位，呼叫直接失敗（2026-09-24 踩到）。\n"
                      "for (const k of Object.keys(msg.payload)) {\n"
                      "    if (k in incoming) { msg.payload[k] = incoming[k]; }\n"
                      "}\n"
                      "// 記下「送指令前」的任務編號，等待時才分得出新舊任務\n"
                      "msg._seq_before = ((global.get(\"drone_status\") || {}).task || {}).seq;\n"
                      "return msg;"),
             "outputs": 1, "timeout": 0, "noerr": 0, "initialize": "", "finalize": "",
             "libs": [], "x": 160, "y": 80, "wires": [[inst]]},
            {"id": inst, "type": f"subflow:{CALL}", "z": sid, "name": f"/{NS}/api/{action}",
             "env": [{"name": "SERVICE", "type": "str", "value": f"/{NS}/api/{action}"},
                     {"name": "TIMEOUT", "type": "num", "value": "10"}],
             "x": 320, "y": 80, "wires": [[fn_wait]]},
            {"id": fn_wait, "type": "function", "z": sid, "name": "等任務完成",
             "func": WAIT_CODE, "outputs": 2, "timeout": 0, "noerr": 0,
             "initialize": "", "finalize": "", "libs": [], "x": 460, "y": 80,
             "wires": [[], []]},
        ]
    return out


def act(tag, i, y, done_target, fail_target, x=380, z=None, env=None):
    """在畫布上放一個動作積木。env 只有要改預設值時才傳。"""
    label = next(a[1] for a in ACTIONS if a[0] == tag)
    return {"id": nid(i), "type": f"subflow:{act_id(tag)}", "z": z, "name": label,
            "env": env or [], "x": x, "y": y,
            "wires": [[done_target] if done_target else [],
                      [fail_target] if fail_target else []]}


# --- 儀表板（Dashboard 2.0）-----------------------------------------------------
# 儀表板是「另一個網址」：編輯器在 :1880，儀表板在 :1880/dashboard。
# 給操作的人用 —— 他看不到流程，只看到按鈕和狀態。
#
# 結構是三層：ui-base（整個網站）→ ui-page（一頁）→ ui-group（一區）→ 元件。
# 這些是「設定節點」，沒有 z，不屬於任何分頁。
DASH = "d0c0ffeeffff0004"          # 儀表板流程所在的分頁（放元件和接線）
LINK_OUT = "d0c0ffeelink0001"   # 儀表板「開始任務」的出口，接到使用者自己的 link in
UI_BASE, UI_THEME, UI_PAGE = "d0c0ffeeui000001", "d0c0ffeeui000002", "d0c0ffeeui000003"


def ui_group(gid, name, order, width=6):
    return {"id": gid, "type": "ui-group", "name": name, "page": UI_PAGE,
            "width": width, "height": 1, "order": order, "showTitle": True,
            "className": "", "visible": True, "disabled": False, "groupType": "default"}


def ui_button(i, gid, label, order, target, y, color="", bg="", x=520):
    """儀表板按鈕。按下去送出一則 msg 給接在後面的動作積木。"""
    return {"id": nid(i), "type": "ui-button", "z": DASH, "group": gid, "name": label,
            "label": label, "order": order, "width": 0, "height": 0,
            "emulateClick": False, "tooltip": "", "color": color, "bgcolor": bg,
            "className": "", "icon": "", "iconPosition": "left",
            "payload": "", "payloadType": "str", "topic": label, "topicType": "str",
            "buttonColor": bg, "textColor": color, "iconColor": "",
            "enableClick": True, "enablePointerdown": False, "pointerdownPayload": "",
            "pointerdownPayloadType": "str", "enablePointerup": False,
            "pointerupPayload": "", "pointerupPayloadType": "str",
            "x": x, "y": y, "wires": [[target] if target else []]}


def ui_text(i, gid, label, order, y, x=900):
    """一行文字，顯示 msg.payload。"""
    return {"id": nid(i), "type": "ui-text", "z": DASH, "group": gid, "order": order,
            "width": 0, "height": 0, "name": label, "label": label,
            "format": "{{msg.payload}}", "layout": "row-spread", "style": False,
            "font": "", "fontSize": 16, "color": "#717171", "wrapText": False,
            "className": "", "x": x, "y": y, "wires": []}


def ui_gauge(i, gid, label, order, y, x=900):
    return {"id": nid(i), "type": "ui-gauge", "z": DASH, "name": label, "group": gid,
            "order": order, "value": "payload", "valueType": "msg", "width": 0, "height": 0,
            "gtype": "gauge-half", "gstyle": "needle", "title": label,
            "alwaysShowTitle": True, "floatingTitlePosition": "inside",
            "units": "m", "icon": "", "prefix": "", "suffix": "",
            "segments": [{"from": 0, "color": "#5cd65c"}, {"from": 4, "color": "#ff8c00"}],
            "min": 0, "max": 6, "sizeThickness": 16, "sizeGap": 0, "sizeKeyThickness": 8,
            "styleRounded": True, "styleGlow": False, "className": "",
            "x": x, "y": y, "wires": []}


DASH_STATUS_CODE = r'''// 把狀態 JSON 拆成儀表板各個欄位要顯示的東西（四個出口）
let s;
try { s = JSON.parse(msg.payload.data); } catch (e) { return null; }

if (!s.connected) {
    return [{ payload: "未連線" }, { payload: 0 }, { payload: "—" }, { payload: "沒收到 PX4 的狀態" }];
}
const 狀態 = s.landed ? "在地面" : "在空中";
const 解鎖 = s.armed ? "已解鎖" : "未解鎖";
let 提示 = "正常";
if (s.watchdog && s.watchdog.tripped) {
    提示 = "⚠ 自動降落：" + s.watchdog.reason;       // 看門狗是最重要的訊息，優先顯示
} else if (s.patrol && s.patrol.active) {
    提示 = `巡航中 ${s.patrol.index}/${s.patrol.total} 點`;
} else if (s.task && s.task.state) {
    提示 = `${s.task.name || "—"}：${s.task.state}`;
}
return [
    { payload: `${s.mode}（${解鎖}、${狀態}）` },
    { payload: Number(s.altitude_m) || 0 },
    { payload: s.last_message || "—" },
    { payload: 提示 },
];
'''

DASH_HELP = """### 這個分頁在做什麼
產生**儀表板**畫面：http://localhost:1880/dashboard

編輯器（:1880）是給你組流程用的；儀表板是給操作的人用的，他看不到流程，只看到按鈕和狀態。

### 畫面上有什麼
| 區 | 內容 |
| --- | --- |
| 緊急 | 🛑 緊急降落（紅色） |
| 狀態 | 模式／解鎖／落地、高度儀表、提示、最近訊息 |
| 飛行 | 高度輸入框、起飛、盤旋、降落 |
| 任務 | ▶ 開始任務 |
| 模擬 | 開模擬、關模擬 |

### 高度輸入怎麼運作
畫面上輸入的高度存進共用區，按「起飛」時把它塞進 payload。動作積木的規則是
**「傳進來的值蓋過積木自己的設定」**，所以同一個積木在流程裡用固定值、在儀表板上用畫面輸入的值。

### 「▶ 開始任務」要跑你自己的任務
這顆按鈕接到一個叫「執行任務」的 **link out**，但它**還沒指向任何流程**。接法：

1. 到你自己的分頁，從左側「共通」分類拖一個 **`link in`** 進去
2. 接到你那條任務的第一個積木
3. 雙擊那個 `link in` → 勾選「執行任務」→ 完成
4. **Deploy**

設定是存在**你的 `link in` 裡**，所以下次 `install_flows.sh` 更新儀表板分頁時**不會被蓋掉**。

### 自己組一條任務
左側面板的 **「無人機」** 分類裡有現成積木（起飛、調整高度、巡航、盤旋、降落、狀態查詢），
每個都是「送指令 → 等它真的做完」，有**完成**和**失敗**兩個出口。
把它們串起來、前面接一個 `link in`（給儀表板用）和一個 inject（自己測試用）就好。

**中途你自己按別的按鈕**不算失敗：積木會安靜停下、印一行「任務已被手動指令接手」，不會自動降落。

### 要改版面
雙擊任何一個元件 → 改 `group`（放哪一區）、`order`（順序）、`width`（寬度）。
分區本身在「設定節點」裡（編輯器右側選單 → 設定節點 → ui-group）。

### ⚠️ 沒有密碼
儀表板網址不用登入就能開。同網段的人知道網址就能按按鈕，測試階段請不要開放到外網。
"""


def dashboard_nodes():
    """儀表板的設定節點、元件和接線。"""
    g_emg, g_state = "d0c0ffeeuig00001", "d0c0ffeeuig00002"
    g_fly, g_mis, g_sim = "d0c0ffeeuig00003", "d0c0ffeeuig00004", "d0c0ffeeuig00005"
    nodes = [
        {"id": UI_BASE, "type": "ui-base", "name": "無人機控制台", "path": "/dashboard",
         "includeClientData": True, "acceptsClientConfig": [
             "ui-notification", "ui-control"],
         "showPathInSidebar": False, "showReconnectNotification": True,
         "notificationDisplayTime": "5", "showDisconnectNotification": True,
         "allowInstall": True},
        {"id": UI_THEME, "type": "ui-theme", "name": "預設",
         "colors": {"surface": "#ffffff", "primary": "#0094ce", "bgPage": "#eeeeee",
                    "groupBg": "#ffffff", "groupOutline": "#cccccc"},
         "sizes": {"density": "default", "pagePadding": "12px", "groupGap": "12px",
                   "groupBorderRadius": "4px", "widgetGap": "12px"}},
        {"id": UI_PAGE, "type": "ui-page", "name": "控制", "ui": UI_BASE, "path": "/control",
         "icon": "drone", "layout": "grid", "theme": UI_THEME,
         "breakpoints": [{"name": "Default", "px": 0, "cols": 3},
                         {"name": "Tablet", "px": 576, "cols": 6},
                         {"name": "Small Desktop", "px": 768, "cols": 9},
                         {"name": "Desktop", "px": 1024, "cols": 12}],
         "order": 1, "className": "", "visible": True, "disabled": False},
        ui_group(g_emg, "緊急", 1, 6),
        ui_group(g_state, "狀態", 2, 6),
        ui_group(g_fly, "飛行", 3, 6),
        ui_group(g_mis, "任務", 4, 6),
        ui_group(g_sim, "模擬", 5, 6),
        {"id": DASH, "type": "tab", "label": "儀表板", "disabled": False,
         "info": "產生儀表板畫面的流程。由 make_drone_basic.py 產生。"},
        {"id": nid(500), "type": "comment", "z": DASH,
         "name": "說明（點我，看右側「資訊」分頁）", "info": DASH_HELP,
         "x": 220, "y": 40, "wires": []},
    ]

    # 緊急降落：直接接降落積木
    nodes += [ui_button(510, g_emg, "🛑 緊急降落", 1, nid(511), 100, color="#ffffff",
                        bg="#d32f2f", x=300),
              act("a5", 511, 100, None, None, x=560, z=DASH)]

    # 狀態：訂閱 → 拆成四個出口 → 顯示
    nodes += [
        {"id": nid(520), "type": f"subflow:{SUB}", "z": DASH, "name": f"/{NS}/api/status",
         "env": [{"name": "TOPIC", "type": "str", "value": f"/{NS}/api/status"},
                 {"name": "TYPE", "type": "str", "value": "std_msgs/msg/String"}],
         "x": 180, "y": 220, "wires": [[nid(521)]]},
        {"id": nid(521), "type": "function", "z": DASH, "name": "拆成畫面欄位",
         "func": DASH_STATUS_CODE, "outputs": 4, "timeout": 0, "noerr": 0,
         "initialize": "", "finalize": "", "libs": [], "x": 420, "y": 220,
         "wires": [[nid(522)], [nid(523)], [nid(524)], [nid(525)]]},
        ui_text(522, g_state, "模式", 1, 160, x=700),
        ui_gauge(523, g_state, "高度", 2, 220, x=700),
        ui_text(524, g_state, "最近訊息", 4, 280, x=700),
        ui_text(525, g_state, "提示", 3, 340, x=700),
    ]

    # 飛行：高度輸入 → 記起來；三顆按鈕
    nodes += [
        {"id": nid(530), "type": "ui-number-input", "z": DASH, "group": g_fly,
         "name": "起飛高度", "label": "高度（公尺）", "order": 1, "width": 0, "height": 0,
         "topic": "altitude", "topicType": "str", "min": 0.5, "max": 5, "step": 0.5,
         "tooltip": "", "delay": 300, "passthru": True, "sendOnDelay": True,
         "sendOnBlur": True, "sendOnEnter": True, "className": "", "clearable": False,
         "icon": "", "iconPosition": "left", "iconInnerPosition": "inside",
         "spinner": True, "x": 180, "y": 420, "wires": [[nid(531)]]},
        {"id": nid(531), "type": "function", "z": DASH, "name": "記住高度",
         "func": ("// 存進共用區，按「起飛」時才拿出來用\n"
                  "global.set(\"ui_altitude\", Number(msg.payload));\nreturn null;"),
         "outputs": 1, "timeout": 0, "noerr": 0, "initialize": "", "finalize": "",
         "libs": [], "x": 420, "y": 420, "wires": [[]]},
        ui_button(540, g_fly, "起飛", 2, nid(541), 480, x=300),
        {"id": nid(541), "type": "function", "z": DASH, "name": "帶入高度",
         "func": ("// 畫面上輸入的高度優先；沒輸入過就用 1.5\n"
                  "msg.payload = { altitude: global.get(\"ui_altitude\") || 1.5 };\n"
                  "return msg;"),
         "outputs": 1, "timeout": 0, "noerr": 0, "initialize": "", "finalize": "",
         "libs": [], "x": 520, "y": 480, "wires": [[nid(542)]]},
        act("a1", 542, 480, None, None, x=720, z=DASH),
        ui_button(550, g_fly, "盤旋", 3, nid(551), 540, x=300),
        act("a4", 551, 540, None, None, x=560, z=DASH),
        ui_button(560, g_fly, "降落", 4, nid(561), 600, x=300),
        act("a5", 561, 600, None, None, x=560, z=DASH),
    ]

    # 任務：不在這裡重接一條，改用 link out 去觸發「任務」分頁上那一條。
    # 這樣任務只有一份定義：你改任務分頁，儀表板按鈕跟著改。
    nodes += [
        ui_button(570, g_mis, "▶ 開始任務", 1, LINK_OUT, 680, x=300),
        # links 故意留空：要接哪一條任務，由「你自己的分頁」上的 link in 決定。
        # Node-RED 執行時會從 link in 那邊補上這條線（runtime/lib/flows/util.js:336-341），
        # 所以設定存在你的節點裡，下次更新這個分頁也不會被蓋掉。
        {"id": LINK_OUT, "type": "link out", "z": DASH, "name": "執行任務",
         "mode": "link", "links": [], "x": 545, "y": 680, "wires": []},
    ]

    # 模擬：和「系統」分頁一樣，按鈕 → systemctl
    nodes += [
        ui_button(580, g_sim, "開模擬", 1, nid(581), 760, x=300),
        sys_exec(581, "systemctl --user start drone-sim.service "
                      "&& echo '已送出啟動，Gazebo 視窗會跳出來（PX4 約 15 秒後連上）' "
                      "|| echo '啟動失敗'", nid(590), 760, name="開模擬"),
        ui_button(585, g_sim, "關模擬", 2, nid(586), 820, x=300),
        sys_exec(586, "systemctl --user stop drone-sim.service && echo '模擬已關閉' "
                      "|| echo '關閉失敗'", nid(590), 820, name="關模擬"),
        {"id": nid(590), "type": "function", "z": DASH, "name": "整理輸出",
         "func": SYS_CODE, "outputs": 1, "timeout": 0, "noerr": 0,
         "initialize": "", "finalize": "", "libs": [], "x": 900, "y": 790,
         "wires": [[nid(591)]]},
        ui_text(591, g_sim, "模擬狀態", 3, 790, x=1080),
    ]
    # sys_exec 產生的節點 z 預設是「系統」分頁，這裡要改成儀表板分頁
    for n in nodes:
        if n.get("z") == SYS:
            n["z"] = DASH
    return nodes


def main():
    nodes = load_subflows()
    nodes.append({"id": TAB, "type": "tab", "label": f"無人機控制（{NS}）", "disabled": False,
                  "info": "起飛／盤旋／降落 + 狀態 + log。由 drone_nodered/nodered/make_drone_basic.py 產生。"})
    nodes.append({"id": nid(1), "type": "comment", "z": TAB, "name": "使用說明（點我，看右側「資訊」分頁）",
                  "info": HELP, "x": 220, "y": 40, "wires": []})

    # 緊急降落單獨放最上面，和一般按鈕隔開，避免誤按旁邊的東西
    # ⚠️ 它會經過瀏覽器→Node-RED→rosbridge→ROS，任一層卡住就沒用。
    #    真正的最後防線是遙控器，這顆是「方便」不是「保命」。
    nodes += [inject(70, "🛑 緊急降落", {}, nid(71), 90),
              call(71, f"/{NS}/api/land", nid(72), 90),
              debug(72, "緊急降落：結果", 90)]

    # 三個指令：按鈕 → 呼叫服務 → debug
    for k, (label, payload, action, y) in enumerate([
            # 狀態查詢放第一個：最常按，而且它不會動到飛機
            ("狀態查詢", {}, "state", 190),
            ("起飛 1.5 m", {"altitude": 1.5}, "takeoff", 260),
            ("盤旋", {}, "hover", 330),
            # 空中改高度。水平位置不動，改完 PX4 自己轉成盤旋（DO_CHANGE_ALTITUDE）。
            # 要先起飛，在地面上按會被拒絕（PX4 沒解鎖時不會執行這個指令）。
            ("調整高度 3 m", {"altitude": 3.0}, "set_altitude", 400),
            # 巡航：正方形 10 m。x 北為正、y 東為正，座標是「離起飛點」的公尺數。
            # 服務只負責「開始」，飛行在背景進行，進度看狀態裡的 patrol 欄位。
            ("巡航（10 m 正方形）",
             {"x": [10, 10, 0, 0], "y": [0, 10, 10, 0],
              "altitude": 3.0, "speed": 2.0, "loops": 1}, "patrol", 470),
            ("降落", {}, "land", 540)]):
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

    # 積木的定義還是要產生（左側面板要用），只是不再附一個示範任務分頁 ——
    # 任務由使用者放在自己的分頁，更新時才不會被換掉。
    nodes += wait_subflow() + action_subflows()

    nodes += dashboard_nodes()

    # 幫每個分頁和積木蓋上標記：更新流程時，merge_flows.py 靠它認出「這是本專案的」
    for n in nodes:
        if n["type"] in ("tab", "subflow"):
            n["info"] = (n.get("info", "") + "\n\ngenerated-by: drone_nodered").strip()

    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, "w", encoding="utf-8") as f:
        json.dump(nodes, f, ensure_ascii=False, indent=2)
    print(f"寫出 {os.path.normpath(OUT)}（{len(nodes)} 個節點，含兩個積木的定義）")


if __name__ == "__main__":
    main()

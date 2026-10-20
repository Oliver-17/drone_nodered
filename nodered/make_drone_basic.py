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
import argparse
import json
import os
import sys

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
# --with-empty-sim 的開關（由 main() 設定）
WITH_EMPTY_SIM = False
# --force 的開關（由 main() 設定）。見 _ok_to_overwrite ——
# 這支產生器抹掉過一次工作成果，現在預設不覆寫手改的內容。
FORCE = False

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
// 追蹤中顯示進度。三種狀態要分清楚：正在找／暫時看不到／正常追蹤 ——
// 車子靠近時會掉出相機視野，那時飛機還在追，不能顯示成「在找目標」。
//
// 2026-10-08 起用 track.phase（searching / following）而不是 track.searching ——
// 搜尋階段現在是「巡航一整條環線」，不是原地轉頭，所以要看得到走到第幾個航點
// 和第幾圈，否則使用者只看到「尋找目標中 90 秒」完全不知道有沒有在動。
//
// 2026-10-12：用詞和儀表板對齊（同一件事不要兩個名字），而且多兩件資訊：
//   ① 「跟丟了回頭找」和「冷啟動在找」分開 —— 前者代表車就在附近
//   ② 跟隨時顯示「實際距離 / 要保持的距離」和「保持了多久」，才看得出來穩了沒
const tr = s.track || {};
if (tr.active) {
    if (tr.phase === "searching") {
        const 頭 = tr.reacquire ? "跟丟了，回頭找" : "巡航找目標";
        if (tr.search_index !== null && tr.search_index !== undefined) {
            msg.payload.追蹤 = `${頭}：第 ${tr.search_index} 個航點` +
                               `（第 ${(tr.search_laps || 0) + 1} 圈，已找 ${tr.search_s} 秒）`;
        } else {
            msg.payload.追蹤 = `${頭}（已找 ${tr.search_s} 秒）`;
        }
    } else {
        const 想要 = (tr.want !== null && tr.want !== undefined) ? ` / ${tr.want}` : "";
        const 保持 = (tr.locked_s !== null && tr.locked_s !== undefined)
            ? `，保持 ${Math.round(tr.locked_s)} 秒` : "";
        msg.payload.追蹤 = `跟著目標 ${tr.distance}${想要} m${保持}` +
                           (tr.lost_s > 0.5 ? `（暫時看不到 ${tr.lost_s} 秒）` : "");
    }
}
// Nav2 導航（api/goto）。
// ⚠️ 進度要以 state 為準，**不要只看 distance_remaining** ——
//    目標不可達時 Nav2 的 feedback 會把 distance_remaining 退化成 0
//    （L2 實測：規劃不出路徑的目標先顯示 remain 0.0，再跳 failed）。
//    只看那個數字的話畫面會先顯示「已到達」然後突然失敗。
const nv = s.nav || {};
if (nv.state === "running") {
    const g = nv.goal || [];
    msg.payload.導航 = `往 (${g[0]}, ${g[1]})` +
        (nv.distance_remaining !== null && nv.distance_remaining !== undefined
            ? `，剩 ${nv.distance_remaining} m` : "");
} else if (nv.state === "failed") {
    msg.payload.導航失敗 = nv.message || "Nav2 回報失敗";
}
// 巡航中才多顯示進度（第幾個點／共幾個點）
const pt = s.patrol || {};
if (pt.active) {
    msg.payload.巡航 = `${pt.index}/${pt.total} 點`;
}
// 下一個 filter 節點「只比對 msg.key」：模式、解鎖、落地有變才輸出。
// 高度不放進 key —— 它每 0.5 秒都在變，放進去的話 debug 側欄會被洗版。
// 巡航進度也放進 key：飛到下一個點時要看得到，不然中間十幾秒完全沒訊息
// 2026-10-08：加 track.phase / search_index 和 nav.state —— 階段或航點變了
// 要輸出一筆，否則搜尋繞一整圈（40 秒以上）畫面上完全沒動靜。
// distance_remaining 刻意**不**放進 key：它每 0.5 秒都在變，會洗版。
msg.key = `${s.mode}|${s.armed}|${s.landed}|${(s.patrol || {}).index}` +
          `|${(s.task || {}).seq}|${(s.task || {}).state}|${(s.watchdog || {}).tripped}` +
          `|${(s.track || {}).active}|${(s.track || {}).distance}` +
          `|${(s.track || {}).phase}|${(s.track || {}).search_index}` +
          `|${(s.nav || {}).state}`;
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
| 開模擬 | 倉庫場景。Gazebo 視窗會跳出來，**PX4 約 15 秒、Nav2 約 45 秒**才就緒 |
| 關模擬 | 只關模擬，控制台留著 |
| 模擬狀態 | 現在有沒有在跑 |
| 看模擬 log | 最後 40 行。開不起來時看這個 |
| 全部關閉 | 模擬和控制台一起關（**Node-RED 自己也會關掉**，等於桌面的「停」圖示） |

### 場景長什麼樣
36 × 24 公尺的倉庫，四排 7 公尺高的貨架（飛行高度 5 公尺，所以飛不過去），
白色半透明屋頂（從上面看得穿）。無人機生成在**西南角**，中間走道有一台洋紅色的車
繞著東邊第二排貨架轉圈。

看不到無人機的話：滑鼠滾輪推近，或在右側 Entity Tree 的 `x500_cam_down_0`
上按右鍵 → Move to。

### 什麼時候才算真的可以用
按「開模擬」之後 Gazebo 很快就跳出來，但**那不代表可以飛**：

| 等什麼 | 大約 |
| --- | --- |
| PX4 連上 agent（狀態才會有東西） | 15 秒 |
| Nav2 就緒（「導航到座標」才能用） | 45 秒 |

太早按起飛會被拒絕（狀態還沒來），太早按導航會說「Nav2 沒有在跑」。

### 空白世界呢
舊的空白世界（`drone-sim.service`）2026-10-12 退休了，東西都還留著。
要叫回來在終端機跑：

```
bash ~/ros2_ws/src/drone_nodered/scripts/install_services.sh --with-empty-sim
bash ~/ros2_ws/src/drone_nodered/scripts/install_flows.sh
```

這個分頁會多出「開空白世界」等四顆按鈕。
"""

HELP = """### 怎麼開始
桌面點「無人機控制台（開）」→ 這頁的按鈕就能用了。

### 這個流程更新了怎麼辦
**不要用編輯器的「匯入」**（會問你重複 id，選錯就變成兩份，很難清）。跑這行：
```
~/ros2_ws/src/drone_nodered/scripts/install_flows.sh
```
它會先備份你現在的流程、換成新版、把控制台重開，然後 F5 重新整理就好。
Gazebo 要另外在 **「系統」分頁**按「開模擬」（倉庫場景）。
**PX4 約 15 秒**才連上（狀態才會有東西）、**Nav2 約 45 秒**才就緒
（「導航到座標」積木才能用）。

### 怎麼用
- 點「起飛 1.5 m」「盤旋」「降落」左邊的小方塊
- 結果、狀態、log 都在右側的 **Debug** 分頁（蟲子圖示）
- 積木下方顯示「已連線」才能用；「未連線」代表 rosbridge 沒開

### 改高度
- **起飛時的高度**：雙擊「起飛 1.5 m」→ 改 payload 的 `1.5` → 完成 → **Deploy**。
- **飛到一半要改**：按「調整高度 3 m」（同樣可以雙擊改數字）。水平位置不會跑掉，
  改完就在新高度盤旋。**要先起飛**，在地上按會被拒絕。

兩個都限制 **0.5–6 m**（`drone_api_node` 的 `max_takeoff_altitude`，
由 `drone-nodered.service` 帶 `6.0`），超出會被拒絕。
倉庫的巡航高度是 5 m（貨架 7 m、屋頂 8 m），所以 6 m 留了 1 m 餘裕。

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
    # ⚠️ 2026-10-08：DISTANCE 預設從 2.0 改成 0（＝自動）。
    #    0 的意思是「算出讓目標落在畫面正中的距離」＝ 高度 / tan(相機俯角)。
    #    67° 相機在 5 m 高 → 2.12 m；3 m 高 → 1.27 m。因為它隨高度變，
    #    填固定值反而會讓目標偏離畫面中心。舊的 2.0 是 45° 相機時代的值
    #    （那時正下方是盲區，非得保持距離）。
    #
    # SEARCH_X / SEARCH_Y 是**map 座標**（和「導航到座標」一樣），不是
    #    api/patrol 的「離起飛點 NED」。預設值是倉庫的 search_loop
    #    （繞 x=+2.8 那排貨架，和車反向跑）。
    #    空陣列 = 不巡航找，只會停在原地轉頭掃描 45 秒 —— 在倉庫幾乎找不到，
    #    因為相機在 5 m 高只看得到前方約 2~6 m，而走道長 12 m。
    ("a7", "追蹤目標", "track", 300,
     [("DISTANCE", "num", "0", "保持距離（公尺，0 = 自動算最佳值）"),
      ("ALTITUDE", "num", "0", "高度（公尺，0 = 維持現在）"),
      ("DURATION", "num", "60", "追多久（秒）"),
      ("SEARCH_X", "json", "[5.6,5.6,0,0]", "搜尋環線 x（map 座標，東為正）"),
      ("SEARCH_Y", "json", "[-9,9,9,-9]", "搜尋環線 y（map 座標，北為正）")],
     'msg.payload = {\n'
     '    distance: Number(env.get("DISTANCE")),\n'
     '    altitude: Number(env.get("ALTITUDE")),\n'
     '    duration: Number(env.get("DURATION")),\n'
     '    search_x: env.get("SEARCH_X"), search_y: env.get("SEARCH_Y"),\n'
     '};'),
    # 導航到座標（2026-10-08 新增）。走 Nav2，會自己繞開地圖上的貨架。
    #
    # ⚠️ 座標是 **map 座標系**，和「巡航」積木的「離起飛點 NED」不同。
    #    這是整個 API 裡唯一的例外，所以欄位名刻意叫 MAP_X / MAP_Y 而不是 X / Y
    #    —— 在積木上就看得出來。倉庫的生成點在 map (-14, -9)，所以
    #    「去 map (0,0)」等於「離起飛點北 +9、東 +14」，兩者不會剛好相等。
    #
    # YAW 用**度**不用弧度（Node-RED 的使用者不該算弧度），積木內部轉。
    #    留 0 就送 NaN＝「不在乎終點朝向」—— 目前的 nav2 參數
    #    yaw_goal_tolerance 是 3.14，本來就不檢查，填了也不會被遵守。
    ("a8", "導航到座標", "goto", 240,
     [("MAP_X", "num", "0.0", "map 座標 x（公尺，東為正）"),
      ("MAP_Y", "num", "0.0", "map 座標 y（公尺，北為正）"),
      ("YAW", "num", "999", "終點朝向（度，999 = 不在乎）")],
     # ⚠️ 不要自己加 TIMEOUT 欄位 —— 動作積木的模板本來就會加一個
     #    （「等待完成」的逾時），同名會變成兩個一樣的欄位。
     #    節點那邊的 goto_timeout_s 預設 120 秒，比這個積木的 240 秒先到，
     #    所以卡住時是節點先把任務標成失敗、積木跟著走「失敗」出口。
     # ⚠️ 不在乎朝向時送 999（度）而不是 NaN。
     #    rosbridge **不能帶 NaN 給浮點欄位** —— 它的型別檢查在
     #    rosbridge_library/internal/message_conversion.py:462-465，
     #    字串 "nan" 會被拒絕（type_map['str'] 裡沒有 float32），
     #    而 JS 的 JSON.stringify(NaN) 會變成 null，null 也一樣被拒絕。
     #    所以 GoTo.srv 另外約定「|yaw| > pi 也代表不在乎」。
     #    預設就填 999：終點朝向目前本來就不被檢查
     #    （nav2 的 yaw_goal_tolerance 是 3.14）。
     'msg.payload = {\n'
     '    map_x: Number(env.get("MAP_X")),\n'
     '    map_y: Number(env.get("MAP_Y")),\n'
     '    yaw: Number(env.get("YAW")) * Math.PI / 180,\n'
     '    timeout: 0,   // 0 = 用節點預設（120 秒）\n'
     '};'),
    ("a4", "盤旋", "hover", 30, [], "msg.payload = {};"),
    ("a5", "降落", "land", 150, [], "msg.payload = {};"),
    ("a6", "狀態查詢", "state", 30, [], "msg.payload = {};"),
]


def act_id(tag, n=0):
    return f"d0c0ffeeac{tag}{n:04x}"


# 積木 id 是「對外介面」，不是內部細節。
# 使用者把積木拖到自己的分頁上之後，那個實例是用 id 指向定義的；id 一改，
# 他分頁上的積木就變成指向不存在定義的壞節點，更新時會被 merge_flows.py 清掉。
# 所以這張表把 tag → 服務名的對應鎖死：改到會在產生流程時就跳錯，而不是等使用者發現東西不見了。
# 真的需要換 id（例如某個積木退役）時，要連同 merge_flows.py 的行為一起想清楚再改這裡。
FROZEN_ACTION_IDS = {
    "d0c0ffeeaca10000": "takeoff",
    "d0c0ffeeaca20000": "set_altitude",
    "d0c0ffeeaca30000": "patrol",
    "d0c0ffeeaca40000": "hover",
    "d0c0ffeeaca50000": "land",
    "d0c0ffeeaca60000": "state",
    "d0c0ffeeaca70000": "track",
    "d0c0ffeeaca80000": "goto",
}


def check_frozen_ids():
    """確認積木的 id 和它對應的服務都沒有被改掉。"""
    got = {act_id(tag): action for tag, _, action, *_ in ACTIONS}
    if got != FROZEN_ACTION_IDS:
        only_new = {k: v for k, v in got.items() if FROZEN_ACTION_IDS.get(k) != v}
        only_old = {k: v for k, v in FROZEN_ACTION_IDS.items() if got.get(k) != v}
        raise AssertionError(
            "積木 id 對應變了，使用者分頁上的積木會壞掉。\n"
            f"  現在產生的：{only_new}\n  凍結表裡的：{only_old}\n"
            "  確定要改的話，請一起更新 FROZEN_ACTION_IDS 並告知使用者要重拉積木。")


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


def ui_text(i, gid, label, order, y, x=900, width=0, name=None,
            layout="row-spread"):
    """一行文字，顯示 msg.payload。

    label=""（沒有標籤）時要把 layout 改成 row-center —— row-spread 的意思是
    「標籤靠左、值靠右」，標籤空的話值就會黏在最右邊、左邊空一大片。
    name 是編輯器裡的節點名稱，label 空的時候要另外給，否則流程圖上會變無名節點。
    """
    return {"id": nid(i), "type": "ui-text", "z": DASH, "group": gid, "order": order,
            "width": width, "height": 0, "name": name or label, "label": label,
            "format": "{{msg.payload}}", "layout": layout, "style": False,
            "font": "", "fontSize": 16, "color": "#717171", "wrapText": False,
            "className": "", "x": x, "y": y, "wires": []}


def ui_gauge(i, gid, label, order, y, x=900, width=0, height=0):
    return {"id": nid(i), "type": "ui-gauge", "z": DASH, "name": label, "group": gid,
            "order": order, "value": "payload", "valueType": "msg",
            "width": width, "height": height,
            "gtype": "gauge-half", "gstyle": "needle", "title": label,
            "alwaysShowTitle": True, "floatingTitlePosition": "inside",
            "units": "m", "icon": "", "prefix": "", "suffix": "",
            "segments": [{"from": 0, "color": "#5cd65c"}, {"from": 4, "color": "#ff8c00"}],
            "min": 0, "max": 6, "sizeThickness": 16, "sizeGap": 0, "sizeKeyThickness": 8,
            "styleRounded": True, "styleGlow": False, "className": "",
            "x": x, "y": y, "wires": []}


DASH_STATUS_CODE = r"""// 把狀態 JSON 拆成儀表板各個欄位要顯示的東西（四個出口）
let s;
try { s = JSON.parse(msg.payload.data); } catch (e) { return null; }

if (!s.connected) {
    return [{ payload: "❌ 未就緒（沒收到 PX4 的狀態）" }, { payload: 0 },
            { payload: "—" }, { payload: "—" }];
}
// --- 第一欄：就緒燈 ---------------------------------------------------------
// 就緒了就只寫「✅ 就緒」—— 不接模式／解鎖／落地（Oliver 2026-10-12：
// 「我只想要就緒而已」）。還沒就緒時才列出缺哪一項，因為那時候你要的是
// 「還要等什麼」。四項的定義在 drone_api_node 的 _ready()
// （Node-RED 看不到 Nav2 和那座橋，所以不能在這裡判斷）。
const r = s.ready;
let 系統狀態;
if (!r) {
    // 節點是舊版（還沒有 ready 欄位）。不要顯示「未就緒」——
    // 那會變成查不出原因的假故障，要講清楚是節點該重開。
    系統狀態 = "❓ 節點是舊版（沒有就緒資訊）";
} else if (!r.all) {
    // 「未就緒」後面附上還缺什麼：開模擬之後那 45 秒就是靠這個知道還要等多久，
    // 只寫「未就緒」的話看不出來是正在啟動還是真的壞了。
    const 缺 = [];
    if (!r.px4) 缺.push("PX4");
    if (!r.position) 缺.push("位置估計");
    if (!r.nav2) 缺.push("Nav2");
    if (!r.bridge) 缺.push("cmd_vel 橋");
    系統狀態 = "⏳ 未就緒（等 " + 缺.join("、") + "）";
} else {
    系統狀態 = "✅ 就緒";
}

// --- 第三欄：無人機狀態（飛機現在在做什麼）----------------------------------
let 動作 = "待命";
if (s.watchdog && s.watchdog.tripped) {
    動作 = "⚠ 自動降落：" + s.watchdog.reason;       // 看門狗是最重要的訊息，優先顯示
} else if (s.patrol && s.patrol.active) {
    動作 = `巡航中 ${s.patrol.index}/${s.patrol.total} 點`;
} else if (s.track && s.track.active) {
    const tr = s.track;
    // 2026-10-08 起看 phase（searching / following）。搜尋階段是「巡航一整條
    // 環線」，所以要顯示第幾個航點 —— 繞一圈 40 秒以上，只顯示「尋找中」
    // 使用者會以為卡住了。
    if (tr.phase === "searching") {
        // 2026-10-12：「冷啟動在找」和「鎖定過又跟丟」要分開。後者代表車就在
        // 附近、飛機正回頭找，看到它才知道不必重按一次追蹤。
        const 頭 = tr.reacquire ? "↩️ 跟丟了，回頭找" : "🔍 巡航找目標";
        動作 = (tr.search_index !== null && tr.search_index !== undefined)
            ? `${頭}：第 ${tr.search_index} 點（第 ${(tr.search_laps || 0) + 1} 圈）`
            : `${頭}… ${tr.search_s} 秒`;
    } else {
        // 追到了就要看得出來「穩了」：實際距離／要保持的距離、已經保持多久。
        // 只寫「跟隨中 1.3 m」看不出來那 1.3 到底算準不算準。
        const 想要 = (tr.want !== null && tr.want !== undefined) ? ` / ${tr.want}` : "";
        const 保持 = (tr.locked_s !== null && tr.locked_s !== undefined)
            ? `，保持 ${Math.round(tr.locked_s)} 秒` : "";
        // 「暫時看不到」還在跟隨（車靠近時會掉出相機視野），不能顯示成在找。
        const 暫失 = tr.lost_s > 0.5 ? `（暫時看不到 ${tr.lost_s} 秒）` : "";
        動作 = `🎯 跟著目標：${tr.distance}${想要} m${保持}${暫失}`;
    }
} else if (s.nav && s.nav.state === "running") {
    // Nav2 導航。⚠️ 以 state 為準 —— distance_remaining 在目標不可達時會
    // 退化成 0（L2 實測），只看那個數字會先顯示「快到了」再突然失敗。
    const g = s.nav.goal || [];
    動作 = `導航中 → (${g[0]}, ${g[1]})` +
           (s.nav.distance_remaining ? `，剩 ${s.nav.distance_remaining} m` : "");
} else if (s.task && s.task.state) {
    動作 = `${s.task.name || "—"}：${s.task.state}`;
}
return [
    { payload: 系統狀態 },
    { payload: Number(s.altitude_m) || 0 },
    { payload: s.last_message || "—" },
    { payload: 動作 },
];
"""

DASH_HELP = """### 這個分頁在做什麼
產生**儀表板**畫面：http://localhost:1880/dashboard

編輯器（:1880）是給你組流程用的；儀表板是給操作的人用的，他看不到流程，只看到按鈕和狀態。

### 畫面上有什麼
| 位置 | 區 | 內容 |
| --- | --- | --- |
| 最上面（整列） | 狀態 | **大高度儀表**（置中），下面三欄：系統狀態、無人機狀態、最近訊息 |
| 中間左 | 飛行 | 高度輸入框、起飛、盤旋、降落 |
| 中間右 | 任務 | ▶ 開始任務、🎯 追蹤目標 |
| 下面左 | 模擬 | 開模擬、關模擬（倉庫場景） |
| **右下角** | 緊急 | 🛑 緊急降落（紅色） |

### 系統狀態：什麼時候可以開始飛
高度儀表下面左邊那一欄。**四項全到才顯示「✅ 就緒」**，
還沒好就寫「⏳ 未就緒」並附上缺哪一項（例：`⏳ 未就緒（等 Nav2、cmd_vel 橋）`）：

| 項目 | 少了它會怎樣 |
| --- | --- |
| PX4 | 收不到飛控狀態，所有指令直接被拒 |
| 位置估計 | 連盤旋都不行（PX4 的 `xy_valid`） |
| Nav2 | 「導航到座標」會被拒 |
| cmd_vel 橋 | **最陰險的一項**：Nav2 規劃成功、畫面顯示「導航中」，但飛機不動、也不報錯 |

就緒之後就只有「✅ 就緒」兩個字，不接模式／解鎖／落地 ——
飛機在幹什麼看中間的**無人機狀態**那一欄。
顯示 `❓ 節點是舊版` 的話是 `drone_api_node` 還沒換成新版（跑一次 `install_flows.sh`）。

⚠️ 就緒燈**不包含視覺偵測**。「偵測節點在跑」和「現在看得到車」是兩回事，
放進來會讓人以為燈亮了就一定追得到。追蹤看不看得到目標，看**無人機狀態**那一欄。

### 無人機狀態那一欄：追蹤的三種狀態
| 顯示 | 意思 |
| --- | --- |
| `🔍 巡航找目標：第 2 點（第 1 圈）` | 還沒看到過目標，正在繞環線找 |
| `↩️ 跟丟了，回頭找：第 2 點（第 1 圈）` | **鎖定過**又跟丟了 —— 車就在附近，不用重按追蹤 |
| `🎯 跟著目標：1.31 / 1.27 m，保持 24 秒` | 追到了。前面是實際距離、後面是要保持的距離 |

兩個距離愈接近就愈穩。「保持 N 秒」在跟丟時歸零，所以它也是「這次鎖定穩不穩」的指標。

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

### 模擬要等多久才能飛
按「開模擬」之後 Gazebo 很快就跳出來，但那不代表可以飛：

| 等什麼 | 大約 |
| --- | --- |
| PX4 連上（狀態才會有東西） | 15 秒 |
| Nav2 就緒（「導航到座標」才能用） | 45 秒 |

太早按起飛會被拒絕，太早按導航會說「Nav2 沒有在跑」。

### 自己組一條任務
左側面板的 **「無人機」** 分類裡有現成積木（起飛、調整高度、巡航、盤旋、降落、
狀態查詢、追蹤目標、導航到座標），
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
        # 版面：頁面是 12 欄，分區照 order 由左到右排，排不下就換行。
        # 想要的樣子 ——
        #     第一列  狀態（整列 12 欄，儀表在正中間、放大）
        #     第二列  飛行（左） | 任務（右）
        #     第三列  模擬（左） | 緊急（右下角）
        # 緊急降落放右下角而不是最上面：它是「出事才按」的按鈕，放在最顯眼的
        # 位置反而容易誤觸，但又必須固定在同一個好找的角落。
        ui_group(g_state, "狀態", 1, 12),
        ui_group(g_fly, "飛行", 2, 6),
        ui_group(g_mis, "任務", 3, 6),
        ui_group(g_sim, "模擬", 4, 6),
        ui_group(g_emg, "緊急", 5, 6),
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
        # 儀表排第一、佔滿整列（12 欄）、高 4 格 —— 元件會把自己畫在格子正中間，
        # 所以「整列寬」就等於「置中且最大」。下面三個文字各佔 4 欄，排成一列。
        ui_gauge(523, g_state, "高度", 1, 220, x=700, width=12, height=4),
        # 三欄：系統狀態／無人機狀態／最近訊息。
        # layout 一律 col-center（標題在上、值在下、置中）——
        # 預設的 row-spread 是「標籤靠左、值靠右」，三欄並排時每一欄的值都會
        # 被推到緊貼下一欄的標題，看起來像是下一欄的內容（2026-10-12 Oliver
        # 看到畫面才發現：「這個提示是幹麻的」）。
        ui_text(522, g_state, "系統狀態", 2, 160, x=700, width=4,
                layout="col-center"),
        ui_text(525, g_state, "無人機狀態", 3, 340, x=700, width=4,
                layout="col-center"),
        ui_text(524, g_state, "最近訊息", 4, 280, x=700, width=4,
                layout="col-center"),
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
        # 追蹤目標用動作積木（不是直接呼叫服務）：這樣才有「等它真的做完」的語意，
        # 失敗時（等 45 秒找不到目標、位置估計失效…）訊息會進狀態區的「無人機狀態」。
        ui_button(575, g_mis, "🎯 追蹤目標", 2, nid(576), 740, x=300),
        act("a7", 576, 740, None, None, x=560, z=DASH),
    ]

    # 模擬：和「系統」分頁一樣，按鈕 → systemctl
    #
    # 2026-10-12：指向 drone-warehouse.service（倉庫）。
    # 空白世界（drone-sim.service）退休了 —— 理由和叫回來的方式見上面
    # buttons 那一段的註解。
    # ⚠️ 儀表板**刻意不加**封存那組按鈕（即使帶了 --with-empty-sim）：
    #    儀表板是給「只要按按鈕飛」的人用的，場景切換屬於維護操作，
    #    放在編輯器的「系統」分頁就夠了。
    nodes += [
        ui_button(580, g_sim, "開模擬", 1, nid(581), 760, x=300),
        sys_exec(581, "systemctl --user start drone-warehouse.service "
                      "&& echo '已送出啟動，Gazebo 視窗會跳出來"
                      "（PX4 約 15 秒、Nav2 約 45 秒後就緒）' "
                      "|| echo '啟動失敗'", nid(590), 760, name="開模擬"),
        ui_button(585, g_sim, "關模擬", 2, nid(586), 820, x=300),
        sys_exec(586, "systemctl --user stop drone-warehouse.service "
                      "&& echo '模擬已關閉' || echo '關閉失敗'", nid(590), 820,
                 name="關模擬"),
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


def _ok_to_overwrite(nodes):
    """現有的 flow 檔裡有手改的東西嗎？有就擋下來，回 False。

    🔴 2026-10-20：這支產生器**抹掉過一次工作成果**。
       儀表板的地圖 widget 是直接改在 flows/drone_basic.json 裡的
       （照這支檔頭寫的「在 Node-RED 裡改了就匯出蓋掉」那個流程），
       而 test/t_merge_keep.py 當時會先跑一次產生器「確保素材最新」——
       於是跑一次測試就把地圖整個產生掉了。repo 的檔案安靜地從 135 個節點
       退回 133 個，測試照樣通過，下一次 install_flows.sh 就把舊版裝進
       Node-RED。那次是靠 git 才找回來的。

    所以產生器不能再「無聲覆寫」。這裡比對「現有檔案」和「這次要產生的」：

        會消失的節點   現有檔案有、產生器不認得 → 手改加上去的
        會變回去的節點 兩邊都有但內容不同        → 手改過內容的

    任何一種存在就擋下來並列出來。真的要重新產生（例如改了
    nodered_ros2 的積木）就加 --force，但先看一眼列表。

    ⚠️ 這個函式只讀檔案、不寫 —— 判斷和寫入分開，才不會出現
       「擋下來了但檔案已經被 open(..., 'w') 截斷」。
    """
    if not os.path.exists(OUT):
        return True
    try:
        with open(OUT, encoding="utf-8") as f:
            old = json.load(f)
    except (OSError, ValueError) as e:
        print(f"（讀不到現有的 {os.path.basename(OUT)}：{e} → 當成新檔案處理）")
        return True

    new_by_id = {n["id"]: n for n in nodes if "id" in n}
    gone, changed = [], []
    for n in old:
        i = n.get("id")
        if i is None:
            continue
        if i not in new_by_id:
            gone.append(n)
        elif (json.dumps(n, sort_keys=True, ensure_ascii=False)
              != json.dumps(new_by_id[i], sort_keys=True, ensure_ascii=False)):
            changed.append(n)

    if not gone and not changed:
        return True

    def _label(n):
        return (f"{n.get('name') or n.get('label') or '(沒有名字)'}"
                f"  [{n.get('type')}  id={n.get('id')}]")

    def _list(title, items):
        print(f"\n  {title}（{len(items)} 個）：")
        for n in items[:20]:
            print(f"      - {_label(n)}")
        if len(items) > 20:
            print(f"      … 還有 {len(items) - 20} 個")

    print(f"✗ 沒有寫出 —— 現有的 {os.path.basename(OUT)} 裡有產生器做不出來的內容")
    if gone:
        _list("會**消失**的節點", gone)
    if changed:
        _list("會**變回產生器版本**的節點", changed)
    print("""
  這些大多是「在 Node-RED 裡改好之後匯出蓋回來」的成果（儀表板就是這樣做的）。
  產生器是初始骨架，不是唯一來源 —— 它做不出上面那些東西。

  真的要重新產生（例如改了 nodered_ros2 的積木定義）：
      python3 nodered/make_drone_basic.py --force
  但先確認上面的列表裡沒有你要留的東西。想保留的話，正確做法是把改動補進
  這支產生器，而不是 --force 之後再手改一次。""")
    return False


def main():
    # 封存的空白世界（drone-sim.service）2026-10-12 退休。
    # 帶 --with-empty-sim 重跑這支，「系統」分頁就會多出那四顆按鈕。
    # ⚠️ 服務本身要另外裝：install_services.sh --with-empty-sim
    global WITH_EMPTY_SIM, FORCE
    ap = argparse.ArgumentParser(description="產生 Node-RED 流程（drone_basic.json）")
    ap.add_argument("--with-empty-sim", action="store_true",
                    help="連封存的空白世界按鈕一起產生"
                         "（服務要另外裝：install_services.sh --with-empty-sim）")
    ap.add_argument("--force", action="store_true",
                    help="即使會蓋掉手改的內容也照樣寫出（見 _ok_to_overwrite）")
    args = ap.parse_args()
    WITH_EMPTY_SIM = args.with_empty_sim
    FORCE = args.force
    check_frozen_ids()
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
            # 追蹤目標：視覺算出目標位置，飛機保持 2 m 跟著。要先開視覺
            # （sim_all.launch.py 預設會一起開）。看不到目標會先原地轉頭找。
            ("追蹤目標（2 m、30 秒）",
             {"distance": 2.0, "altitude": 0, "duration": 30.0}, "track", 610),
            ("降落", {}, "land", 680)]):
        # 從 300 起跳：60 後面接不下去（70~72 是緊急降落、100~151 是「系統」分頁、
        # 200~212 是狀態和 log），300 這一段目前沒人用
        b = 300 + k * 10
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
    # 2026-10-12：空白世界（drone-sim.service）退休了，只留倉庫。
    #
    # 為什麼：兩個場景各有「開／關／狀態／log」四顆，再加第三個場景就是 12 顆 ——
    # 不是「按鈕多」而是同一組操作被複製 N 份。Oliver 決定舊地圖不用了，
    # 所以直接只留一個場景，按鈕名稱就用最直覺的「開模擬」。
    #
    # 舊的東西都還在倉庫裡（systemd/drone-sim.service.in、gz/worlds/empty*.sdf、
    # sim_all.launch.py 的 world 參數），要叫回來三行：
    #     bash scripts/install_services.sh --with-empty-sim
    #     python3 nodered/make_drone_basic.py --with-empty-sim   ← 不能省
    #     bash scripts/install_flows.sh
    # 那時下面的 WITH_EMPTY_SIM 會是 True，多出「開空白世界」等四顆
    # （明確命名的是封存的那個，因為「模擬」已經是倉庫的名字了）。
    #
    # ⚠️ drone-warehouse.service 帶 api:=false —— 底座已經有一個 drone_api_node，
    #    兩個同時跑的話指令會落到不確定的那一個（2026-10-08 踩過）。
    buttons = [
        ("開模擬", "systemctl --user start drone-warehouse.service "
                   "&& echo '已送出啟動。Gazebo 視窗會跳出來，"
                   "PX4 約 15 秒、Nav2 約 45 秒後才就緒' "
                   "|| echo '啟動失敗，按「看模擬 log」看原因'", 120),
        ("關模擬", "systemctl --user stop drone-warehouse.service && echo '模擬已關閉' "
                   "|| echo '關閉失敗'", 220),
        ("模擬狀態", "systemctl --user is-active drone-warehouse.service", 320),
        ("看模擬 log", "journalctl --user -u drone-warehouse.service -n 40 --no-pager", 420),
        ("全部關閉", "systemctl --user --no-block stop drone-nodered.service "
                     "&& echo '已送出關閉：模擬和控制台都會停，這個畫面也會失去連線'", 520),
    ]
    # 封存的空白世界（--with-empty-sim 才加）。
    # 名稱刻意寫明「空白世界」—— 因為「模擬」已經是倉庫的名字了。
    if WITH_EMPTY_SIM:
        buttons += [
            ("開空白世界", "systemctl --user start drone-sim.service "
                           "&& echo '已送出啟動（倉庫會自動停掉）。"
                           "Gazebo 視窗會跳出來，PX4 約 15 秒後才連上' "
                           "|| echo '啟動失敗，按「看空白世界 log」看原因'", 640),
            ("關空白世界", "systemctl --user stop drone-sim.service "
                           "&& echo '空白世界已關閉' || echo '關閉失敗'", 740),
            ("空白世界狀態", "systemctl --user is-active drone-sim.service", 840),
            ("看空白世界 log",
             "journalctl --user -u drone-sim.service -n 40 --no-pager", 940),
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
    # 擋住「無聲覆寫手改的內容」—— 見 _ok_to_overwrite 的說明
    if not FORCE and not _ok_to_overwrite(nodes):
        return 1
    with open(OUT, "w", encoding="utf-8") as f:
        json.dump(nodes, f, ensure_ascii=False, indent=2)
    print(f"寫出 {os.path.normpath(OUT)}（{len(nodes)} 個節點，含兩個積木的定義）")
    return 0


if __name__ == "__main__":
    # 回傳碼要傳出去 —— 被擋下來時呼叫者（腳本、CI）才知道沒有寫出
    sys.exit(main() or 0)

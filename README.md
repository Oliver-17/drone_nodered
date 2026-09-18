# drone_nodered — 用 Node-RED 點按鈕控制無人機

ROS 2 Humble + PX4 的**無人機控制套件**。把「起飛、盤旋、調整高度、巡航、降落、狀態查詢」
做成六個獨立的 ROS 2 服務，再用 Node-RED 的按鈕去呼叫，**全程不用開終端機**。

> 這個 repo **本身就是一個 ROS 2 package**，不是 workspace。
> clone 進你既有 workspace 的 `src/` 底下即可。

---

## ⚠️ 這是三個套件中的一個，三個都要裝

| 套件 | 角色 | GitHub |
|---|---|---|
| **drone_nodered**（本包） | 無人機專用：控制節點、Node-RED 流程、模擬世界、systemd 服務、桌面圖示 | https://github.com/Oliver-17/drone_nodered |
| nodered_ros2 | 通用橋接：rosbridge 啟動檔 + 兩個 Node-RED 積木。**不認得無人機**，接地面車也能用 | https://github.com/Oliver-17/nodered_ros2 |
| drone_nodered_interfaces | 只有服務格式（`.srv`）。獨立是因為跑 rosbridge 的電腦必須認得這些格式 | https://github.com/Oliver-17/drone_nodered_interfaces |

**相依方向**：`drone_nodered` → 需要另外兩個；`nodered_ros2` 和 `drone_nodered_interfaces` 彼此獨立、
也不依賴 `drone_nodered`。所以**三個都要 clone 才跑得起來**，但另外兩個可以單獨拿去接別的機器人。

```
drone_nodered ──需要──> nodered_ros2            （積木、rosbridge）
      └───────需要──> drone_nodered_interfaces  （Takeoff / SetAltitude / Patrol 格式）
```

---

## 快速導覽

| 你想知道 | 看哪一節 |
|---|---|
| **我只是忘記指令怎麼打** | [0 指令速查](#0-指令速查) |
| 這包在做什麼、為什麼要拆三包 | [1 這包在做什麼](#1-這包在做什麼) |
| **全新電腦要怎麼裝** | [2 安裝](#2-安裝) |
| 平常怎麼操作 | [3 日常使用](#3-日常使用) |
| 有哪些動作、參數與限制 | [4 六個服務](#4-六個服務) |
| Node-RED 怎麼跟 PX4 講到話 | [5 架構](#5-架構) |
| **流程改了要怎麼更新** | [6 更新 Node-RED 流程](#6-更新-node-red-流程) |
| 檔案在哪、改東西動哪個檔 | [7 檔案結構](#7-檔案結構) |
| 實機（樹莓派）怎麼跑 | [8 實機](#8-實機) |
| 有什麼坑 | [9 踩過的雷](#9-踩過的雷) |

---

## 0 指令速查

### 0.1 全新電腦：一次性安裝

```bash
# 1) 三個套件都要
cd ~/ros2_ws/src
git clone git@github.com:Oliver-17/drone_nodered.git
git clone git@github.com:Oliver-17/nodered_ros2.git
git clone git@github.com:Oliver-17/drone_nodered_interfaces.git

# 2) 編譯（一定要在 workspace 根目錄）
cd ~/ros2_ws
colcon build --packages-select drone_nodered nodered_ros2 drone_nodered_interfaces
source ~/ros2_ws/install/setup.bash

# 3) 裝 systemd 服務 + 桌面圖示（不會設成開機自動啟動）
~/ros2_ws/src/drone_nodered/scripts/install_services.sh

# 4) 裝 Node-RED 流程
~/ros2_ws/src/drone_nodered/scripts/install_flows.sh
```

### 0.2 每天用

| 要做的事 | 怎麼做 |
|---|---|
| 開控制台 | 桌面點 **「無人機控制台（開）」**（瀏覽器會自動開 `localhost:1880`） |
| 開模擬（Gazebo） | Node-RED **「系統」分頁 → 開模擬**，等約 15 秒 |
| 飛 | **「無人機控制（MAV1）」分頁** → 狀態查詢／起飛／盤旋／調整高度／巡航／降落 |
| 關掉全部 | 桌面點 **「無人機控制台（停）」**，或系統分頁按 **全部關閉** |

### 0.3 不想用圖示時的指令版

```bash
systemctl --user start drone-nodered     # 控制台（rosbridge + drone_api_node + Node-RED）
systemctl --user start drone-sim         # 模擬（agent + PX4 SITL + Gazebo）
systemctl --user stop  drone-nodered     # 全關（模擬會跟著停）
systemctl --user status drone-sim        # 看狀態
journalctl --user -u drone-sim -f        # 看模擬的 log
```

### 0.4 用 ros2 指令直接呼叫（不透過 Node-RED）

```bash
ros2 service call /MAV1/api/state        std_srvs/srv/Trigger
ros2 service call /MAV1/api/takeoff      drone_nodered_interfaces/srv/Takeoff "{altitude: 1.5}"
ros2 service call /MAV1/api/set_altitude drone_nodered_interfaces/srv/SetAltitude "{altitude: 3.0}"
ros2 service call /MAV1/api/patrol       drone_nodered_interfaces/srv/Patrol \
    "{x: [10,10,0,0], y: [0,10,10,0], altitude: 3.0, speed: 2.0, loops: 1}"
ros2 service call /MAV1/api/hover        std_srvs/srv/Trigger
ros2 service call /MAV1/api/land         std_srvs/srv/Trigger
ros2 topic echo   /MAV1/api/status
```

### 0.5 改了程式之後

```bash
cd ~/ros2_ws && colcon build --packages-select drone_nodered && \
systemctl --user restart drone-nodered

# 改了 Node-RED 流程的產生器
python3 ~/ros2_ws/src/drone_nodered/nodered/make_drone_basic.py
~/ros2_ws/src/drone_nodered/scripts/install_flows.sh
```

---

## 1 這包在做什麼

一句話：**讓不會寫程式的人也能用滑鼠控制無人機。**

Node-RED 是一個用瀏覽器操作的流程編輯器（把方塊連起來就是程式）。它本身不懂 ROS 2，
所以中間要一層轉接。整條鏈是：

```
你按按鈕（瀏覽器）
   → Node-RED
   → rosbridge（把 JSON 轉成 ROS 2 呼叫）
   → drone_api_node（本包）
   → PX4（VehicleCommand）
   → 飛機動作
```

### 為什麼拆成三個套件

| 如果全部塞在一起 | 拆開之後 |
|---|---|
| 別人想用 Node-RED 接地面車，卻被迫安裝 `px4_msgs` | `nodered_ros2` 完全不認得無人機，誰都能用 |
| 跑 rosbridge 的電腦要裝整包無人機程式才認得服務格式 | `drone_nodered_interfaces` 只有格式，很小 |

三包的邊界就是這樣定的：**通用的東西不准依賴特定載具**。

### 為什麼用 PX4 內建模式，不用 offboard

offboard 要**每秒送 20 次** setpoint，中間只要 Node-RED、瀏覽器或網路卡一下超過
`COM_OF_LOSS_T`，PX4 就會觸發失效保護。起飛、盤旋、降落這種「送一次、PX4 自己做完」的動作
不需要冒這個風險。

代價：巡航靠 `DO_REPOSITION`，需要有效的經緯度參考（見 [9 踩過的雷](#9-踩過的雷)）。

---

## 2 安裝

### 2.1 前置（這些不在 repo 裡，要自己裝）

| 軟體 | 版本（開發時用的） | 用途 |
|---|---|---|
| Ubuntu | 22.04 | |
| ROS 2 | Humble | |
| PX4-Autopilot | v1.17.0，且要 `make px4_sitl_default` 編過 | 模擬用的飛控 |
| Gazebo | Harmonic（gz sim 8.15） | 模擬環境 |
| micro-XRCE-DDS Agent | — | PX4 ↔ ROS 2 的橋 |
| px4_msgs | 與 PX4 版本相符 | ROS 2 這邊的訊息定義 |
| rosbridge_suite | `sudo apt install ros-humble-rosbridge-suite` | Node-RED ↔ ROS 2 |
| Node.js | 24.x | Node-RED 的執行環境 |
| Node-RED | 5.0.7（[官方安裝腳本](https://nodered.org/docs/getting-started/local)） | 流程編輯器 |

> 安裝 Node-RED 時它會問要不要設成開機自動啟動 —— **選「否」**。
> 我們用自己的 systemd 服務（見下），兩套一起跑會搶 1880 埠。

### 2.2 安裝本專案

見 [0.1 全新電腦：一次性安裝](#01-全新電腦一次性安裝)。兩支安裝腳本都支援 `--dry-run`，
可以先看它要做什麼再決定：

```bash
~/ros2_ws/src/drone_nodered/scripts/install_services.sh --dry-run
~/ros2_ws/src/drone_nodered/scripts/install_flows.sh    --dry-run
```

`install_services.sh` 會做的事：
1. 檢查 ROS 環境和 `install/setup.bash` 在不在（路徑錯就停下來，不寫壞東西）
2. 把服務範本裡的路徑換成你這台電腦的，放進 `~/.config/systemd/user/`
3. 桌面與應用程式選單各放兩個圖示
4. `systemctl --user daemon-reload`

它**不會**把服務設成開機自動啟動，也**不會覆蓋**不是它產生的同名檔案
（產生的檔案第一行有 `# generated-by:` 標記）。

要移除：`~/ros2_ws/src/drone_nodered/scripts/uninstall_services.sh`

---

## 3 日常使用

### 3.1 兩個服務、兩個分頁

| systemd 服務 | 內容 | 什麼時候開 |
|---|---|---|
| `drone-nodered`（控制台） | rosbridge + drone_api_node + Node-RED | 點桌面圖示 |
| `drone-sim`（模擬） | XRCE agent + PX4 SITL + Gazebo | Node-RED「系統」分頁按「開模擬」 |

`drone-sim` 設了 `PartOf=drone-nodered.service`，所以**關控制台時模擬會自動跟著關**，
不會留下 Gazebo 在背景吃顯卡。

### 3.2 一次完整的飛行

1. 桌面點「無人機控制台（開）」→ 瀏覽器自動開 `localhost:1880`
2. 「系統」分頁 → **開模擬** → Gazebo 視窗跳出來
3. **等約 15 秒**（PX4 要這麼久才連上 agent，狀態才會出現）
4. 「無人機控制（MAV1）」分頁 → **狀態查詢**（確認「在地面、未解鎖」）
5. **起飛 1.5 m** → 等它爬升 → **調整高度 3 m** → **巡航** → **降落**
6. 用完點桌面「無人機控制台（停）」

> 結果、狀態、log 都在右側的 **Debug** 分頁（蟲子圖示）。

### 3.3 空白世界裡看不到無人機？

正常。空白世界沒有參照物，預設鏡頭在 6 公尺外，而 x500 只有 0.5 公尺，看起來像一個小點。
滑鼠滾輪推近，或在右側 Entity Tree 的 `x500_0` 按右鍵 → **Move to**。

---

## 4 六個服務

全部都在 `/<namespace>/api/` 底下（預設 namespace 是 `MAV1`）。

| 服務 | 型別 | 做什麼 | PX4 指令 |
|---|---|---|---|
| `state` | `std_srvs/Trigger` | **狀態查詢**：回報按下那一刻的位置、高度、速度、朝向 | 不送指令 |
| `takeoff` | `Takeoff` | 解鎖並起飛到指定高度 | `NAV_TAKEOFF`(22) |
| `hover` | `std_srvs/Trigger` | 定點定高盤旋；巡航中按會**就地煞停** | `DO_SET_MODE` / `DO_REPOSITION` |
| `land` | `std_srvs/Trigger` | 在目前位置降落 | `NAV_LAND`(21) |
| `set_altitude` | `SetAltitude` | **空中改高度**，水平位置不動 | `DO_CHANGE_ALTITUDE`(186) |
| `patrol` | `Patrol` | **巡航**：依序飛過一串點 | `DO_REPOSITION`(192) |

### 4.1 參數與限制

| 項目 | 預設 | 說明 |
|---|---|---|
| 起飛／調整高度範圍 | 0.5–5 m | launch 參數 `min_takeoff_altitude`／`max_takeoff_altitude`。**室內實機建議改 1.5** |
| 巡航點數 | ≤ 20 | |
| 巡航座標 | ±50 m | 防止打錯字（10 打成 100）飛出視線 |
| 巡航速度 | 0.5–5 m/s，0 = PX4 預設 | |
| 巡航到點判定 | 水平距離 < 0.5 m | |

座標定義：**x 北為正、y 東為正**，單位公尺，原點是**起飛點**。

### 4.2 每個服務都會先擋掉不合理的情況

- 沒收到 PX4 狀態 →「沒有收到 PX4 的狀態，確認 agent 有連上」
- 在地上按盤旋／降落／調整高度／巡航 →「飛機在地上，請先起飛」
  （PX4 在沒解鎖時會把這些指令默默吃掉，所以我們自己先擋，不讓你以為壞了）
- 沒有有效的高度／經緯度參考 → 直接拒絕，不用目前值去猜

### 4.3 狀態 topic

`/<namespace>/api/status`（`std_msgs/String`，內容是 JSON），每 0.5 秒一筆：

```json
{"connected": true, "armed": true, "landed": false, "mode": "AUTO_LOITER",
 "altitude_m": 2.99, "xy_valid": true,
 "patrol": {"active": true, "index": 2, "total": 4},
 "last_command": "patrol", "last_ok": true, "last_message": "..."}
```

Node-RED 上只有「模式／解鎖／落地／巡航進度」變化時才顯示，避免洗版。

---

## 5 架構

```
瀏覽器（你）
   │  http://localhost:1880
Node-RED ── 流程 drone_basic.json（本包提供）
   │        積木 ROS2 呼叫服務 / ROS2 訂閱（nodered_ros2 提供）
   │  WebSocket ws://127.0.0.1:9090
rosbridge ── nodered_ros2/launch/rosbridge.launch.py
   │  ROS 2 服務呼叫（格式來自 drone_nodered_interfaces）
drone_api_node（本包 scripts/drone_api_node.py）
   │  /MAV1/fmu/in/vehicle_command
micro-XRCE-DDS Agent
   │  UDP 8888（模擬）／序列埠（實機）
PX4
```

**一個容易誤會的地方**：Node-RED 不「看管」模擬。系統分頁的按鈕只是執行一行
`systemctl`，真正開關 Gazebo 的是 systemd。所以你按 Deploy 或重啟 Node-RED，
模擬都不會被誤殺。

---

## 6 更新 Node-RED 流程

**不要用編輯器的「匯入」來更新。** 匯入只適合第一次；之後再匯入，Node-RED 會發現節點 id
已存在，問你要「取代」還是「導入副本」，選錯就變成兩份重複的流程和積木，而且很難清掉。

正確做法一行：

```bash
~/ros2_ws/src/drone_nodered/scripts/install_flows.sh
```

它會：偵測控制台在跑就先停 → 把現有流程備份成 `flows.json.bak.<日期時間>` →
換成新版 → 再開回來。內容一樣時會直接說「不需要更新」。

> **為什麼一定要先停 Node-RED**：它關閉時會把記憶體裡的流程寫回 `flows.json`，
> 開著改檔案會被它蓋回去。

改流程內容請改產生器 `nodered/make_drone_basic.py`，再重跑它產生 JSON。
產生器結尾有一道 `assert` 會檢查節點 id 有沒有重複（撞號會讓整個分頁消失，見
[9 踩過的雷](#9-踩過的雷)）。

---

## 7 檔案結構

```
drone_nodered/
├── scripts/
│   ├── drone_api_node.py        六個服務的實作（核心）
│   ├── start_empty_sitl.sh      啟動 PX4 SITL + Gazebo（FOREGROUND=1 才會自己收尾）
│   ├── install_services.sh      裝 systemd 服務 + 桌面圖示
│   ├── uninstall_services.sh    移除上面那些
│   ├── install_flows.sh         裝／更新 Node-RED 流程
│   ├── console_start.sh         桌面「開」圖示執行的東西
│   └── console_stop.sh          桌面「停」圖示執行的東西
├── launch/
│   ├── drone_api.launch.py      只開 drone_api_node
│   ├── nodered_all.launch.py    控制台三件套（rosbridge / drone_api / Node-RED）
│   └── sim_all.launch.py        模擬（agent + SITL）
├── nodered/
│   ├── make_drone_basic.py      ★ 流程的產生器，要改流程改這支
│   └── flows/drone_basic.json   產生出來的流程（不要手改）
├── systemd/*.service.in         服務範本（安裝時才換成實際路徑）
├── desktop/*.desktop.in         桌面圖示範本
└── gz/worlds/empty.sdf          空白世界
```

---

## 8 實機

實機時**不會**用到 `drone-sim` 服務：PX4 是飛控板上的韌體，本來就在跑；agent 跑在樹莓派上
走序列埠，跟模擬的 UDP 不一樣。

| 東西 | 跑在哪 |
|---|---|
| Node-RED | **筆電**（操作介面就是要在人這邊） |
| rosbridge、drone_api_node | 樹莓派 |
| XRCE agent | 樹莓派（序列埠） |
| PX4 | 飛控板韌體 |

launch 檔已經留了開關，同一份設定兩台電腦各開一半：

```bash
# 樹莓派
ros2 launch drone_nodered nodered_all.launch.py nodered:=false rosbridge_address:=0.0.0.0

# 筆電
ros2 launch drone_nodered nodered_all.launch.py rosbridge:=false drone_api:=false
```

筆電的 Node-RED 積木要把 `ROSBRIDGE_URL` 改成 `ws://<樹莓派IP>:9090`。

> **室內用動捕（OptiTrack）時，巡航不能用**：`DO_REPOSITION` 需要有效的經緯度參考
> （`xy_global`），動捕通常沒有。屆時要改走 offboard，那是另一個題目。

---

## 9 踩過的雷

| 現象 | 原因 | 解法 |
|---|---|---|
| 服務回「Unable to import」 | 跑 rosbridge 的環境不認得自訂服務格式 | rosbridge 必須在 source 過 workspace 的環境下啟動。systemd 服務檔已經自己 source，不靠 `~/.bashrc` |
| 起飛後節點直接當掉 | rclpy 的 logger **同一行不能換等級**（`rcutils_logger.py:286`） | `info` 和 `warn` 寫在不同行 |
| 飛機飛去經緯度 0,0 | `VehicleCommand` 沒用到的參數留 0.0 會被當成有效值 | 一律填 `NaN` |
| `NAV_TAKEOFF` 沒反應 | 它只切模式、不解鎖（`Commander.cpp:1064`） | 先送 `COMPONENT_ARM_DISARM` |
| 模式變 LOITER 但還在爬升 | `NAV_MC_ALT_RAD` 預設 0.8 m，差 0.8 m 就算「起飛完成」 | 別用「模式變 LOITER」當到達高度的判斷 |
| 按盤旋，飛機還是飛完那一段才停 | 切 LOITER **不會清掉** PX4 手上的目標點 | 送「經緯度和高度全 NaN 的 `DO_REPOSITION`」＝原地煞停 |
| 巡航指令完全沒作用 | `DO_REPOSITION` 的 `param2` 第 0 位元沒設 1，PX4 直接回 `UNSUPPORTED`（`Commander.cpp:765`） | 帶 `param2 = 1` |
| Node-RED 兩個分頁的節點疊在一起 | 節點 id 撞號（分頁 id 和某個節點相同） | 產生器結尾的 `assert` 會擋下來 |
| 停模擬後 Gazebo 還在 | `start_empty_sitl.sh` 預設把 PX4 丟背景就結束 | 用 `FOREGROUND=1`（服務已經這樣設） |
| 剛開模擬，Node-RED 沒有狀態 | PX4 要約 11–15 秒才連上 agent | 等一下再看 |
| 已開著的終端機找不到新的服務格式 | 環境變數只往下傳，不會回頭影響已開的終端 | 關掉重開，或 `source ~/ros2_ws/install/setup.bash` |

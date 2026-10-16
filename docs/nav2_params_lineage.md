# 三份 Nav2 參數檔的關係

專案裡有三份 `nav2_params`，它們**不是複製貼上的分支，而是有方向性的**：
調校結果從實機往模擬搬，場地幾何從模擬往實機搬。搞反會把驗過的東西弄壞。

```
  drone_nav2_apriltag/config/nav2_params.yaml      arena（最早的模擬場地）
                    │
                    │ 複製 → 換場地幾何
                    ▼
  drone_nodered/config/warehouse_nav2.yaml         倉庫（Node-RED 用）
                    ▲
                    │ 搬「調校結果」（2026-10-07，L0）
                    │
  drone_bringup/config/real/nav2_params.yaml       實機（室內動捕場）
```

## 哪些該跟、哪些不該跟

| 類別 | 例子 | 方向 | 理由 |
|---|---|---|---|
| **bug fix** | `default_server_timeout` | 實機 → 全部 | 20 ms 在任何平台都會卡死，和 CPU 無關 |
| **行為修正** | 拿掉 `PathAlign`/`GoalAlign`、加 `Twirling` | 實機 → 全部 | 前方投影點對全向機體是錯的，和平台無關 |
| **CPU 預算** | `controller_frequency`、`v*_samples`、`bt_loop_duration` | 實機 → 模擬 | 模擬機 CPU 夠，但**模擬的用途是排練實機**，頻率不同手感就搬不過去 |
| **場地尺寸** | `max_vel_*`、`xy_goal_tolerance`、`inflation_radius`、costmap 大小 | **不要互搬** | 倉庫 36×24 m vs 室內小場地，這是物理差異不是調校 |
| **感測器** | `local_costmap` 的 `plugins` | **不要互搬** | arena 的 x500_nav2 有光達，倉庫的 x500 沒有 |
| **時鐘** | `use_sim_time` | **不要互搬** | 只有 arena 有橋 `/clock` |

## 2026-10-07（L0）搬了什麼

從 `drone_bringup/config/real/nav2_params.yaml` → `drone_nodered/config/warehouse_nav2.yaml`：

| 項目 | 原本 | 現在 |
|---|---|---|
| `default_server_timeout` | 20 | **200** |
| `bt_loop_duration` | 10 | 50 |
| `controller_frequency` | 20.0 | 10.0 |
| `vx/vy/vtheta_samples` | 20/10/10 | 10/10/5 |
| `critics` | 含 `GoalAlign`/`PathAlign` | 改 `Twirling.scale: 10.0` |

**刻意沒搬**：`max_vel_*`（0.8 vs 0.3）、`xy_goal_tolerance`（0.4 vs 0.2）。

### 搬之前查過的一件事

實機那份的註解警告：`max_vel_theta` 必須 ≤ bridge 的 `max_yawspeed`，
否則超出的部分會被 `cmd_vel_to_px4_node.cpp:348` 的 `clampAbs` 夾掉，
DWB 的預測模型就和現實不符，反而更亂。

- 實機：`max_yawspeed: 0.5`（`drone_bringup/config/real/px4_bridge.yaml`）＝ `max_vel_theta: 0.5` ← 剛好相等，要調必須兩邊一起
- 模擬：`max_yawspeed: 1.0`（`drone_control/config/px4_bridge.yaml`）> `max_vel_theta: 0.8` ← 有餘裕

所以倉庫這份的 `max_vel_theta` 不用動。

## `arena` 那份還沒跟

`drone_nav2_apriltag/config/nav2_params.yaml` **還是舊的**（`default_server_timeout: 20`、
含 `PathAlign`）。那套目前沒有人在用，等要用再搬 —— 搬的時候照上面那張表，
只搬前兩類（bug fix 和行為修正）加 CPU 預算，不要動它的場地尺寸。

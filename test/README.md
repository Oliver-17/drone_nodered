# 測試

## 不用飛的（改完程式先跑這個）

```bash
bash ~/ros2_ws/src/drone_nodered/test/run_all.sh     # 全部，約 15 秒
```

沒有一支需要 ROS 在跑、Gazebo、或 source 過工作區 —— 它們用正規表示式從
`scripts/drone_api_node.py` 挖出待測函式再 `exec`，所以在任何終端機都跑得起來。

| 測試 | 驗什麼 | 起因 |
| --- | --- | --- |
| `t_no_fly.py` | 跟隨段的禁飛方框擋得住、又不會把正常跟隨夾死 | 2026-10-19 撞貨架摔機（含用真實座標重演那一次） |
| `t_standoff_dir.py` | 站位方向的連續混合與一階低通 | 跟隨時畫面左右搖擺、鎖定瞬間抽動 |
| `t_coords.py` | map ↔ PX4 本地 NED 三個轉換永遠互相一致 | 2026-10-20 把原點解析提取成一份（本來有三份） |
| `t_search_waypoint.py` | 搜尋航點「ACCEPTED 但被搶走」會被抓到並重送 | 2026-10-20「它沒有去巡航找車」（白站 10.1 秒） |
| `t_merge_keep.py` | `install_flows.sh` 不會動到使用者自己的分頁 | 2026-09-28「自訂任務」分頁的積木被清空 |
| `t_map_widget.js` | 儀表板地圖的 JS 真的跑得起來、座標顏色環線都對 | ui-template 壞掉時**終端機沒有任何錯誤**，一天踩三次 |
| `t_launch_order.sh` | `launch_arguments` 的求值順序 | 咬過兩次，都是「有設定但值是錯的，而且不報錯」 |

每一支的檔頭都寫著「為什麼要有這個測試」和當初的症狀 —— 那比測試本身重要，
因為那些 bug 的共同點是**不會報錯**。

## 要飛的

在 `test/tools/`。**一定要先 `source test/tools/guard.sh` 並呼叫
`guard_or_die`** —— 2026-10-08 測試腳本把 Oliver 正在跑的堆疊殺掉過。

| 腳本 | 做什麼 |
| --- | --- |
| `run_track_test.sh` | 端到端：車在跑，飛機用 YOLO 追 |
| `run_target_accuracy.sh` | 量目標世界座標的誤差 |
| `run_goto_bench.sh` | 量 Nav2 導航的時間與落點 |
| `run_hover_grab.sh` / `run_grab.sh` | 抓影像素材 |
| `check_yolo_node.py` | 只驗偵測，不用飛 |

# 這個目錄裡的兩種檔案不一樣，不要搞混

| 檔案 | 是什麼 | 誰會讀它 |
|---|---|---|
| `drone_basic.json` | **產生器的輸出**，是要「裝進 Node-RED」的那一份 | `merge_flows.py`（更新流程時的來源） |
| `deployed_20261106.json` | **部署現場的快照**，含 Oliver 手建的「自訂任務」分頁（157 個節點） | 人（備查、對照），**不要**餵給 merge |

## 為什麼要分開（2026-11-07 踩過）

一度把部署快照直接存成 `drone_basic.json`。結果 `merge_flows.py` 把快照裡
「自訂任務」那個分頁當成**專案的**分頁，更新流程時就會清掉使用者自己在裡面
拉的節點 —— 這正是 2026-09-28 那個 bug 的重演，`test/t_merge_keep.py` 當場抓到。

規則：`drone_basic.json` 只能由 `make_drone_basic.py` 產生。要保存部署現場，
另存一份帶日期的快照。

## 產生 / 更新

```bash
python3 nodered/make_drone_basic.py            # 產生（有覆寫護欄）
python3 nodered/make_drone_basic.py --force    # 確定要蓋掉手改內容時
python3 nodered/merge_flows.py                 # 併進正在跑的 Node-RED
```

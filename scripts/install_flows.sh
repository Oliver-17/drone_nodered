#!/usr/bin/env bash
# =============================================================================
#  install_flows.sh —— 把套件裡的 Node-RED 流程裝進 Node-RED（第一次安裝／之後更新都用這支）
#
#  用法：
#      ~/ros2_ws/src/drone_nodered/scripts/install_flows.sh
#      ~/ros2_ws/src/drone_nodered/scripts/install_flows.sh --dry-run          # 只看會做什麼
#      ~/ros2_ws/src/drone_nodered/scripts/install_flows.sh --user-dir /tmp/nr # 裝到別的地方
#
#  為什麼要有這支，不在編輯器裡「匯入」就好：
#      匯入只適合「第一次」。之後版本更新再匯入，Node-RED 會發現節點 id 已經存在，
#      問你要取代還是匯入副本 —— 選錯就變成兩份重複的流程和積木，
#      而且要在編輯器裡一個一個刪，非常痛苦（2026-09-18 實際踩到）。
#      直接換掉流程檔就沒有這個問題，而且別人拿到這個專案也能一行指令搞定。
#
#  只換掉「本專案產生的四個分頁」（無人機控制、系統、任務、零件庫）和三個積木，
#  **你自己新增的分頁會保留**（合併邏輯見 nodered/merge_flows.py）。
#
#  ⚠️ 但東西如果是加在「我們的分頁裡面」，會跟著那一頁被換掉。
#     所以自己的任務請放在自己新增的分頁。每次更新前都會先備份成 flows.json.bak.<日期時間>。
# =============================================================================
set -e

SERVICE="drone-nodered.service"
PKG_SRC="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
USER_DIR="${NODE_RED_USER_DIR:-$HOME/.node-red}"
DRY_RUN=0

EXTRA=()
while [ $# -gt 0 ]; do
    case "$1" in
        --dry-run) DRY_RUN=1; shift ;;
        --user-dir) USER_DIR="$2"; shift 2 ;;
        *.json)
            # 額外要一起裝的流程檔（例如地面車的 husky_basic.json）。
            # 見下面 EXTRA 的註解：為什麼一定要和無人機那份「一起」裝。
            [ -f "$1" ] || { echo "找不到流程檔：$1"; exit 1; }
            EXTRA+=("$1"); shift ;;
        *) echo "不認得的參數：$1"; exit 1 ;;
    esac
done

# 流程檔：從原始碼目錄執行時在 <pkg>/nodered/flows；從安裝目錄執行時問 ROS 2 要路徑
SRC="$PKG_SRC/nodered/flows/drone_basic.json"
# 額外要一起裝的流程檔（例如地面車）。用法：
#     install_flows.sh                                    只裝無人機
#     install_flows.sh ~/ros2_ws/src/husky_nodered/nodered/flows/husky_basic.json
#
# ⚠️ 為什麼一定要「一起」裝，不能分兩次：
#    merge_flows.py 是用 info 裡的 generated-by 標記判斷「哪些要換掉」。
#    只拿地面車那份去合併的話，無人機的分頁會被當成本專案的舊東西整批刪掉。
#    所以這裡把所有產生的流程併成一份再交給 merge_flows.py。
# （在參數解析時填入，見上面的 *.json 那一條）
if [ ! -f "$SRC" ]; then
    SRC="$(ros2 pkg prefix drone_nodered 2>/dev/null)/share/drone_nodered/nodered/flows/drone_basic.json"
fi
if [ ! -f "$SRC" ]; then
    echo "✗ 找不到 drone_basic.json（試過 $PKG_SRC/nodered/flows 和安裝目錄）"
    exit 1
fi

DST="$USER_DIR/flows.json"
if [ ${#EXTRA[@]} -gt 0 ]; then
    COMBINED="$(mktemp /tmp/flows_combined_XXXX.json)"
    python3 - "$COMBINED" "$SRC" "${EXTRA[@]}" <<'PYCOMB'
import json, sys
out_path, *srcs = sys.argv[1:]
seen, merged = {}, []
for p in srcs:
    for n in json.load(open(p, encoding="utf-8")):
        if n["id"] in seen:
            # nodered_ros2 的兩個通用積木是刻意共用同一份定義，重複是正常的。
            # 其他 id 重複代表兩份流程撞號 —— 那會讓後載入的蓋掉前一個，必須擋下。
            if json.dumps(n, sort_keys=True) != json.dumps(seen[n["id"]], sort_keys=True):
                sys.exit(f"✗ id {n['id']} 在兩份流程裡內容不同（{p}）—— 撞號了")
            continue
        seen[n["id"]] = n
        merged.append(n)
print(f"  併了 {len(srcs)} 份流程，共 {len(merged)} 個節點")
json.dump(merged, open(out_path, "w", encoding="utf-8"), ensure_ascii=False, indent=1)
PYCOMB
    SRC="$COMBINED"
fi

MERGE="$PKG_SRC/nodered/merge_flows.py"
[ -f "$MERGE" ] || MERGE="$(ros2 pkg prefix drone_nodered 2>/dev/null)/share/drone_nodered/nodered/merge_flows.py"
echo "來源： $SRC"
echo "目標： $DST"

# 先算出「合併後」的內容放到暫存檔，再比對要不要動作。
# 合併＝我們的分頁用新的，使用者自己的分頁原封不動留著。
NEW="$(mktemp)"
trap 'rm -f "$NEW"' EXIT
if [ -f "$DST" ]; then
    python3 "$MERGE" "$SRC" "$DST" "$NEW" || { echo "✗ 合併失敗，沒有更動任何東西"; exit 1; }
else
    cp "$SRC" "$NEW"
fi

# 內容一樣就不用做事 —— 免得白白多一個備份檔，也免得白重啟服務
if [ -f "$DST" ] && cmp -s "$NEW" "$DST"; then
    echo "✓ 內容已經一樣，不需要更新"
    exit 0
fi

# Node-RED 關閉時會把記憶體裡的流程寫回 flows.json，
# 所以「開著的時候改檔案」會被它蓋回去。一定要先停。
WAS_ACTIVE=0
if systemctl --user is-active --quiet "$SERVICE"; then
    WAS_ACTIVE=1
    echo "控制台正在跑 → 會先停掉、換檔案、再開回來"
fi

if [ "$DRY_RUN" = 1 ]; then
    echo "[dry-run] 會備份 $DST → $DST.bak.<日期時間>"
    echo "[dry-run] 會複製流程檔過去"
    [ "$WAS_ACTIVE" = 1 ] && echo "[dry-run] 會重新啟動 $SERVICE"
    echo "（dry-run，沒有實際更動）"
    exit 0
fi

if [ "$WAS_ACTIVE" = 1 ]; then
    systemctl --user stop "$SERVICE"
    # 等它真的寫完檔再動手
    sleep 2
fi

mkdir -p "$USER_DIR"
if [ -f "$DST" ]; then
    BAK="$DST.bak.$(date +%Y%m%d-%H%M%S)"
    cp -p "$DST" "$BAK"
    echo "  ✓ 舊的流程已備份： $BAK"
fi
cp "$NEW" "$DST"
echo "  ✓ 已更新 $DST"

if [ "$WAS_ACTIVE" = 1 ]; then
    systemctl --user start "$SERVICE"
    echo "  ✓ 控制台已重新啟動"
fi

echo
echo "完成。瀏覽器重新整理 http://localhost:1880 就會看到新的流程。"
echo "（編輯器裡如果還開著舊畫面，按 F5 重新整理才看得到）"

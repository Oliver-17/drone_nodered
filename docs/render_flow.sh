#!/usr/bin/env bash
# =============================================================================
#  render_flow.sh —— 把 message_flow.md 裡的 Mermaid 圖轉成 PNG（貼投影片用）
#
#  用法：
#      bash docs/render_flow.sh              # 輸出到 docs/flow/
#      bash docs/render_flow.sh ~/投影片素材  # 輸出到別的地方
#
#  ⚠️ 第一次跑會用 npx 下載 mermaid-cli（含 Chromium，約 300 MB），要等一下。
#     只是要看圖的話，GitHub 上**直接就會算繪**，不需要跑這支。
#     這支只在「要把圖貼進投影片」的時候才需要。
# =============================================================================
set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="$HERE/message_flow.md"
OUT="${1:-$HERE/flow}"
mkdir -p "$OUT"

# 從 Markdown 切出每一段 ```mermaid```，各存成一個 .mmd
python3 - "$SRC" "$OUT" <<'PY'
import re, sys, io, os
src, out = sys.argv[1], sys.argv[2]
s = io.open(src, encoding="utf-8").read()
# 用前面最近的標題當檔名，比 flow_0 flow_1 好認
parts = re.split(r"^(#{2,3} .+)$", s, flags=re.M)
title, n = "intro", 0
for p in parts:
    if p.startswith("#"):
        title = re.sub(r"[^\w一-鿿]+", "_", p.lstrip("# ").strip())[:40]
        continue
    for b in re.findall(r"```mermaid\n(.*?)```", p, re.S):
        n += 1
        f = os.path.join(out, f"{n:02d}_{title}.mmd")
        io.open(f, "w", encoding="utf-8").write(b)
        print(f)
PY

echo "--- 轉成 PNG ---"
for f in "$OUT"/*.mmd; do
    npx -y @mermaid-js/mermaid-cli -i "$f" -o "${f%.mmd}.png" -b white -s 2 \
        && echo "  ✓ $(basename "${f%.mmd}.png")"
done
echo "完成，PNG 在 $OUT"

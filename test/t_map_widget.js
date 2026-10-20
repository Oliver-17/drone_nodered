#!/usr/bin/env node
// =============================================================================
//  t_map_widget.js —— 把儀表板地圖的 JS 真的跑起來
//
//  跑法（不需要 ROS、不需要 Node-RED、不需要瀏覽器）：
//      node ~/ros2_ws/src/drone_nodered/test/t_map_widget.js
//
//  為什麼要有這個測試：
//      ui-template 的 widget 壞掉時**終端機不會有任何錯誤** —— 畫面就是
//      一格空白，或者圖縮在左上角。2026-10-20 一天之內踩了三次：
//        ① <script> 外面宣告 const      → 整格空白
//        ② viewBox 寫在範本裡被轉小寫    → 內容縮在左上角
//        ③ svg 尺寸用 class 控制         → 塌成十幾像素
//      沒有任何一次是從 log 看出來的，全部靠「盯著畫面猜」。
//
//      所以這支測試做兩件瀏覽器才做得到的事：
//        1. 真的把 widget 的 export default 和「拆成畫面欄位」的 JS 執行起來
//           （語法錯、呼叫不存在的東西，在這裡就會炸）
//        2. 餵一份假的 api/status，一路跑到 computed，檢查算出來的座標、
//           顏色類別、SVG 的 points 字串對不對
//
//  🔴 這支測試**驗不到**的事（只有瀏覽器會發生）：
//      - 範本把駝峰屬性轉小寫（viewBox / preserveAspectRatio 那個坑）
//      - Vuetify 的樣式表蓋掉尺寸
//      所以下面另外用字串規則把那兩件事釘住，當作迴歸防線。
// =============================================================================

const fs = require('fs')
const path = require('path')
const vm = require('vm')

const FLOW = path.join(__dirname, '..', 'nodered', 'flows', 'drone_basic.json')
const MAP_ID = 'd0c0ffee0000020f'      // ui-template「地圖」
const SPLIT_ID = 'd0c0ffee00000209'    // function「拆成畫面欄位」

const fails = []
function check (ok, what, detail) {
  console.log((ok ? '  ✓ ' : '  ✗ ') + what + (detail ? `   ${detail}` : ''))
  if (!ok) { fails.push(what) }
}

const flows = JSON.parse(fs.readFileSync(FLOW, 'utf8'))
const mapNode = flows.find(n => n.id === MAP_ID)
const splitNode = flows.find(n => n.id === SPLIT_ID)
if (!mapNode || !splitNode) {
  console.log('✗ 在 flow 裡找不到地圖 widget 或拆解函式 —— id 被改掉了？')
  process.exit(1)
}

// =============================================================================
//  ① 把 widget 的 <script> 跑起來
// =============================================================================
console.log('① widget 的 export default 能不能執行')
const m = mapNode.format.match(/<script>([\s\S]*?)<\/script>/)
if (!m) {
  console.log('✗ 地圖 widget 裡找不到 <script>')
  process.exit(1)
}
// ui-template 只吃 "export default {...}" —— 換成 module.exports 就能用 vm 跑
const src = m[1].replace(/export\s+default\s*{/, 'module.exports = {')
let comp
try {
  const sandbox = { module: { exports: {} }, console }
  vm.createContext(sandbox)
  vm.runInContext(src, sandbox, { filename: 'map-widget' })
  comp = sandbox.module.exports
  check(true, 'script 語法正確、export default 拿得到')
} catch (e) {
  check(false, 'script 語法正確、export default 拿得到', e.message)
  console.log('\n✗ 無法繼續')
  process.exit(1)
}
check(typeof comp.data === 'function', 'data() 存在')
check(typeof comp.methods === 'object', 'methods 存在')
check(typeof comp.computed === 'object', 'computed 存在')

// 🔴 這一條釘住 2026-10-20 的坑①：<script> 裡除了 export default 不能有
//    別的頂層宣告，否則整格空白而且沒有錯誤訊息。
// ⚠️ 只看**縮排為 0** 的行。先 .trim() 再比對會把函式內部的宣告
//    （例如 watch 裡的 const p = …）全部當成頂層，一路誤報 ——
//    這個專案已經被「檢查腳本自己誤報」騙過兩次了。
const topLevel = src
  .split('\n')
  .filter(l => /^(const|let|var|function|class)\s/.test(l))
check(topLevel.length === 0,
  '<script> 裡沒有 export default 以外的頂層宣告（那會讓整格空白）',
  topLevel.length ? `發現：${topLevel[0].trim()}` : '')

// =============================================================================
//  ② 組一個最小的 Vue 替身，把 data / watch / computed 接起來
// =============================================================================
function mount () {
  const self = Object.assign({}, comp.data(), comp.methods)
  // computed 做成 getter，和 Vue 一樣用 this 存取
  for (const [k, fn] of Object.entries(comp.computed)) {
    Object.defineProperty(self, k, { get: () => fn.call(self), enumerable: true })
  }
  self.$nextTick = (f) => { if (f) { f.call(self) } }
  self.$refs = {}          // 沒有真的 DOM → applyViewBox 會安靜地 return
  self.send = (payload) => comp.watch.msg.call(self, { payload })
  return self
}

// 倉庫的場地方框（map 座標 [x0,y0,x1,y1]）：4 面外牆 + 4 排貨架
const BOXES = [
  [-20, -14, -18, 14], [18, -14, 20, 14],        // 西牆、東牆
  [-20, -14, 20, -12], [-20, 12, 20, 14],        // 南牆、北牆
  [-9.2, -7, -7.6, 7], [-3.6, -7, -2, 7],
  [2, -7, 3.6, 7], [7.6, -7, 9.2, 7]
]
const LOOP = [[5.6, -9], [5.6, 9], [0, 9], [0, -9]]

console.log('② 餵資料進去：場地、無人機、車、環線')
let w = mount()
w.send({ boxes: BOXES, x: 5.6, y: -9, h: 90, tx: 5.6, ty: -2.5, tage: 0.2, tlost: 3,
         loop: LOOP, loop_i: 1 })
check(w.field !== null, 'field 算出來了')
check(w.shelves.length === 4, '認出 4 排貨架（外牆不算）', `得到 ${w.shelves.length}`)
// 場地內緣：西牆內側 -18、東牆 +18、南牆 -12、北牆 +12
check(Math.abs(w.field.x - (-18)) < 1e-9 && Math.abs(w.field.w - 36) < 1e-9,
  '場地內緣 x 是 -18~+18（不含 2 m 厚的牆）',
  `x=${w.field.x} w=${w.field.w}`)
check(Math.abs(w.field.h - 24) < 1e-9, '場地內緣 y 跨 24 m', `h=${w.field.h}`)
check(w.hasPose && w.hasTarget, '無人機和車都有位置')

console.log('③ 車的紅點')
check(Math.abs(w.target.x - 5.6) < 1e-9 && Math.abs(w.target.y - (-2.5)) < 1e-9,
  '車的 map 座標照抄，前端不做任何換算')
check(w.targetStale === false, '0.2 秒前看到 → 新鮮（紅色）')
w.send({ tx: 5.6, ty: -2.5, tage: 5.0, tlost: 3 })
check(w.targetStale === true, '5 秒前看到（> tlost 3）→ 過期（灰色）')
w.send({ tx: 5.6, ty: -2.5, tage: 3.0, tlost: 3 })
check(w.targetStale === false, '剛好 3.0 秒 → 還不算過期（門檻是「超過」）')
w.send({ tx: 1, ty: 2 })                       // 沒有 tage（舊版節點）
check(w.hasTarget && w.targetStale === false,
  '節點沒送 tage → 當成新鮮，不要畫成灰的讓人以為跟丟了')
// 🔴 最重要的一條：欄位不在時**不可以**退回 (0,0)
w = mount()
w.send({ boxes: BOXES })
check(w.hasTarget === false && w.hasPose === false,
  '沒有位置欄位 → hasTarget/hasPose 都是 false（絕不退回 (0,0) 畫在場地正中）')

// 🔴 出生點（2026-10-20，Oliver：「一開始車子就顯示出來好了」）
//    紅點是 YOLO 看到的位置 → 飛機還沒看到車之前地圖上完全沒有車。
//    出生點是場景設定值，所以①一開始就要有、②看到車之後就要讓位。
console.log('③b 車的出生點（場景設定值，空心圈）')
w = mount()
w.send({ boxes: BOXES, thx: 0, thy: -6 })
check(w.hasHome === true && w.hasTarget === false,
  '只有出生點、還沒偵測到 → 畫空心圈（這就是「一開始就看得到車」）')
check(Math.abs(w.home.x - 0) < 1e-9 && Math.abs(w.home.y - (-6)) < 1e-9,
  '出生點的 map 座標照抄', JSON.stringify(w.home))
w.send({ boxes: BOXES, thx: 0, thy: -6, tx: 5.6, ty: -2.5, tage: 0.2, tlost: 3 })
check(w.hasTarget === true && w.hasHome === false,
  '偵測到車 → 實心紅點取代空心圈（量到的位置優先）')
w.send({ boxes: BOXES, thx: 0, thy: -6, tx: 5.6, ty: -2.5, tage: 9.0, tlost: 3 })
check(w.targetStale === true && w.hasHome === false,
  '跟丟之後畫灰點（最後看到的位置），**不要**退回出生點')
w = mount()
w.send({ boxes: BOXES })
check(w.hasHome === false,
  '節點沒送出生點（實機、或沒設 target_home_map）→ 不畫，絕不退回 (0,0)')

console.log('④ 搜尋環線')
w = mount()
w.send({ boxes: BOXES, loop: LOOP, loop_i: 2 })
check(w.loop.length === 4, '環線 4 個點')
// map 的 y 往上、SVG 的 y 往下 → points 的 y 要是負的
check(w.loopPts === '5.6,9 5.6,-9 0,-9 0,9',
  'polygon 的 points：y 取負（map 往北 = 畫面往上）', w.loopPts)
check(w.loopDots.filter(p => p.cur).length === 1, '只有一個點被標成「現在飛去這裡」')
check(w.loopDots[2].cur === true, 'loop_i=2 → 第 2 個點高亮')
w.send({ loop: LOOP, loop_i: null })
check(w.loopDots.filter(p => p.cur).length === 0,
  'loop_i 是 null（跟隨階段）→ 沒有點高亮')
w.send({ loop: [], loop_i: null })
check(w.loop.length === 0, '沒在追蹤 → 環線清空（不要畫上一趟的路線）')

console.log('⑤ 機頭角度 → SVG 的 rotate')
w = mount()
for (const [h, want] of [[90, -90], [0, 0], [-90, 90], [180, -180]]) {
  w.send({ boxes: BOXES, x: 0, y: 0, h })
  const got = w.droneTf
  check(got.includes(`rotate(${want})`),
    `map 角度 ${h}° → rotate(${want})`, got)
}

// =============================================================================
//  ⑥ 瀏覽器才會發生的那兩個坑：用字串規則釘住
// =============================================================================
console.log('⑥ 兩個「畫面壞掉但沒有錯誤」的迴歸防線')
const tpl = mapNode.format.match(/<template>([\s\S]*?)<\/template>/)[1]
// 坑②：駝峰屬性寫在範本裡會被轉成小寫，SVG 不認得
check(!/<svg[^>]*\sviewBox=/.test(tpl) && !/<svg[^>]*\spreserveAspectRatio=/.test(tpl),
  'viewBox / preserveAspectRatio 沒有寫在範本的 <svg> 標籤上（必須用 setAttribute）')
check(/setAttribute\('viewBox'/.test(src) && /setAttribute\('preserveAspectRatio'/.test(src),
  '兩個屬性都是用 setAttribute 設的')
// 坑③：svg 的尺寸必須是 inline style，class 會被 Vuetify 蓋掉
const svgTag = tpl.match(/<svg[\s\S]*?>/)[0]
check(/style="[^"]*width:\s*100%/.test(svgTag) && /style="[^"]*height:\s*\d/.test(svgTag),
  'svg 的寬高寫在 inline style 裡（用 class 會被 Vuetify 蓋掉）')

// =============================================================================
//  ⑦ 把「拆成畫面欄位」接起來跑一次（後端 JSON → 地圖 payload 的那一段）
// =============================================================================
console.log('⑦ 後端的 api/status 一路跑到地圖 payload')
function split (status) {
  const fn = new Function('msg', 'node', splitNode.func + '\nreturn null;')
  return fn({ payload: { data: JSON.stringify(status) } }, { warn () {}, error () {} })
}
const out = split({
  connected: true, armed: true, landed: false, mode: 'AUTO_LOITER',
  altitude_m: 5.0, speed_mps: 2.1, ready: { all: true },
  map_x: 5.6, map_y: -9.0, map_heading_deg: 90.0,
  target_map_x: 5.6, target_map_y: -2.5, target_age_s: 0.2, target_lost_s: 3.0,
  scene: { boxes: BOXES, target_home: [0, -6] },
  track: { active: true, phase: 'searching', search_index: 1, search_laps: 0,
           search_map: LOOP, search_s: 12.0, reacquire: false },
  task: {}, nav: {}, patrol: {}, watchdog: {}
})
check(Array.isArray(out) && out.length === 6, '六個出口都有', `得到 ${out && out.length}`)
const mp = out[5].payload
check(mp.tx === 5.6 && mp.ty === -2.5 && mp.tage === 0.2 && mp.tlost === 3.0,
  '車的位置傳到地圖了', JSON.stringify({ tx: mp.tx, ty: mp.ty, tage: mp.tage }))
check(Array.isArray(mp.loop) && mp.loop.length === 4 && mp.loop_i === 1,
  '搜尋環線和「現在第幾點」傳到地圖了')
check(mp.thx === 0 && mp.thy === -6, '車的出生點傳到地圖了',
  JSON.stringify({ thx: mp.thx, thy: mp.thy }))
check(out[3].payload.includes('巡航找目標') && out[3].payload.includes('第 1 點'),
  '狀態文字顯示在找目標', out[3].payload)

// 跟隨階段：search_index 是 null，環線還是要畫（看得出待會跟丟要回哪找）
const mp2 = split({
  connected: true, ready: { all: true }, altitude_m: 5, speed_mps: 1.2,
  scene: { boxes: BOXES },
  track: { active: true, phase: 'following', search_index: null, search_map: LOOP,
           distance: 2.1, want: 2.12, locked_s: 8.0, lost_s: 0.1 },
  task: {}, nav: {}, patrol: {}, watchdog: {}
})[5].payload
check(mp2.loop.length === 4 && mp2.loop_i === null,
  '跟隨階段：環線照畫，但沒有點被高亮')

// 沒追蹤 → 不送環線
const mp3 = split({
  connected: true, ready: { all: true }, altitude_m: 0, speed_mps: 0,
  scene: { boxes: BOXES }, track: { active: false },
  task: {}, nav: {}, patrol: {}, watchdog: {}
})[5].payload
check(mp3.loop === undefined, '沒在追蹤 → 不送環線')

// 沒連上 PX4 的那條提早 return：場地還是要送（不然看不到倉庫）
const early = split({ connected: false, scene: { boxes: BOXES, target_home: [0, -6] } })
check(early.length === 6 && early[5].payload.boxes.length === BOXES.length,
  '沒連上 PX4 也要送場地方框（否則只剩「等場地資料…」）')
// 🔴 「一開始」指的就是這條路：控制台剛開、模擬還沒起來、PX4 沒連上。
//    出生點在這裡掉了的話，使用者看到的還是「地圖上沒有車」。
check(early[5].payload.thx === 0 && early[5].payload.thy === -6,
  '沒連上 PX4 也要送車的出生點（這就是「一開始」那個時刻）')
// 節點是舊版 / 實機沒設定 → 沒有這個欄位，但不能讓拆解函式壞掉
const noHome = split({ connected: false, scene: { boxes: BOXES } })
check(noHome[5].payload.thx === undefined,
  '節點沒送 target_home → 不編造一個位置出來')

// 整條接起來：後端 JSON → 拆解 → widget
console.log('⑧ 端到端：後端 JSON → widget 的 computed')
w = mount()
w.send(mp)
check(w.hasPose && w.hasTarget && w.loop.length === 4,
  '一份 api/status 跑完，無人機、車、環線都畫得出來')
check(w.loopDots[1].cur === true, '高亮的是第 1 點', JSON.stringify(w.loopDots[1]))
check(Math.abs(w.target.y - (-2.5)) < 1e-9 && w.targetStale === false,
  '車在 y=-2.5、紅色')
check(w.home !== null && w.hasHome === false,
  '出生點收到了，但已經偵測到車 → 不畫它')

console.log()
if (fails.length) {
  console.log(`✗ ${fails.length} 項失敗：`)
  fails.forEach(f => console.log(`    - ${f}`))
  process.exit(1)
}
console.log('✓ 全部通過 —— widget 的 JS 跑得起來，座標、顏色、環線都對')

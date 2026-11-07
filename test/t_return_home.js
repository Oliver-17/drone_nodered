#!/usr/bin/env node
// 「返航」積木的保險。
//
// 返航沒有對應的服務，它是「導航到 scene.home」+「降落」兩個現成積木串起來的。
// 所以真正會出錯的只有一個地方：取回家座標那段。這支測它。
//
// 🔴 最重要的那條在 ⑤：**絕對不能在拿不到 home 的時候退回 (0,0)**。
//    map 的原點是倉庫正中央，不是起飛點（倉庫的生成點在 map(-14, -9)）。
//    退回 (0,0) 會變成「飛到走道中間降落」—— 一個看起來完全正常、
//    但完全錯誤的行為，而且不會有任何錯誤訊息。
const fs = require('fs');
const path = require('path');

const FLOW = path.join(__dirname, '..', 'nodered', 'flows', 'drone_basic.json');
const nodes = JSON.parse(fs.readFileSync(FLOW, 'utf8'));
const fetchNode = nodes.find(n => n.id === 'd0c0ffeeaca90001');
if (!fetchNode) { console.error('✗ 找不到「取回家座標」節點'); process.exit(1); }

function run (status) {
  let st = null;
  const global = { get: k => (k === 'drone_status' ? status : undefined) };
  const node = { status: s => { st = s; } };
  const fn = new Function('msg', 'global', 'node', fetchNode.func);
  // 上一個積木送來的是 {完成:…, 說明:…} —— 要確認它不會被整包塞進服務
  return { out: fn({ payload: { 完成: 'goto', 說明: '已到達' } }, global, node), st };
}

let pass = 0, fail = 0;
const check = (name, cond, extra = '') => {
  if (cond) { console.log(`  ✓ ${name}`); pass++; }
  else { console.log(`  ✗ ${name}  ${extra}`); fail++; }
};

console.log('① 正常：status 有 scene.home');
let r = run({ scene: { home: [-14.0, -9.0], boxes: [] }, task: { seq: 7 } });
check('走出口 1（成功）', r.out[0] !== null && r.out[1] === null);
check('payload 只有 map_x/map_y（不能把 {完成,說明} 帶下去）',
      JSON.stringify(r.out[0].payload) === '{"map_x":-14,"map_y":-9}',
      JSON.stringify(r.out[0] && r.out[0].payload));
check('狀態列顯示回家座標', /回家 \(-14, -9\)/.test(r.st.text), r.st && r.st.text);

console.log('② 沒有 scene.home（實機沒設 local_origin_map）');
r = run({ scene: { boxes: [] }, task: { seq: 7 } });
check('走出口 2（失敗）', r.out[0] === null && r.out[1] !== null);
check('訊息要指出是 local_origin_map 沒設', /local_origin_map/.test(r.out[1].payload.失敗),
      r.out[1] && r.out[1].payload.失敗);

console.log('③ 完全沒有狀態（Node-RED 剛開、還沒收到第一拍）');
r = run(undefined);
check('走出口 2，而且不丟例外', r.out[0] === null && r.out[1] !== null);

console.log('④ home 的格式壞掉');
for (const bad of [[], ['a', 'b'], [1], null, '0,0']) {
  r = run({ scene: { home: bad } });
  check(`home=${JSON.stringify(bad)} → 失敗出口`, r.out[0] === null && r.out[1] !== null);
}

console.log('⑤ 🔴 絕對不能退回 (0,0)');
for (const s of [{ scene: {} }, undefined, { scene: { home: [] } }]) {
  r = run(s);
  check(`${JSON.stringify(s)} → 沒有任何輸出帶 map_x`,
        !(r.out[0] && 'map_x' in (r.out[0].payload || {})));
}

console.log('⑥ 積木的接線（改壞了就等於返航不會降落）');
const sf = nodes.find(n => n.id === 'd0c0ffeeaca90000');
const inner = Object.fromEntries(nodes.filter(n => n.z === 'd0c0ffeeaca90000').map(n => [n.name, n]));
check('subflow 存在、兩個出口', !!sf && sf.out.length === 2);
check('取回家座標 → 導航到生成點',
      inner['取回家座標'].wires[0][0] === inner['導航到生成點'].id);
check('導航到生成點（完成）→ 降落',
      inner['導航到生成點'].wires[0][0] === inner['降落'].id);
check('出口1（完成）只接降落的完成', sf.out[0].wires.length === 1 &&
      sf.out[0].wires[0].id === inner['降落'].id && sf.out[0].wires[0].port === 0);
check('出口2（失敗）接三個失敗來源', sf.out[1].wires.length === 3 &&
      sf.out[1].wires.every(w => w.port === 1));
check('導航用的是「導航到座標」積木', inner['導航到生成點'].type === 'subflow:d0c0ffeeaca80000');
check('降落用的是「降落」積木', inner['降落'].type === 'subflow:d0c0ffeeaca50000');

console.log(`\n${fail === 0 ? '✓ 全部通過 —— 返航拿得到家的座標，拿不到時不會亂飛'
                             : '✗ 有 ' + fail + ' 項失敗'}（共 ${pass + fail} 項）`);
process.exit(fail ? 1 : 0);

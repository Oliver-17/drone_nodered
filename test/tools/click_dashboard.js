#!/usr/bin/env node
// =============================================================================
//  click_dashboard.js —— 用無頭 Chrome 真的「按」儀表板上的按鈕，按完截圖
//
//      node click_dashboard.js "<分區標題>" "<按鈕文字>" [截圖.png] [按完等幾秒]
//      node click_dashboard.js "地面車指令" "追蹤目標" /tmp/after.png 20
//
//  為什麼要真的按，不直接呼叫服務：要驗的是「這顆按鈕接到誰」。
//  直接呼叫 /husky/api/track 只證明服務會動，證明不了按鈕線有接對 ——
//  2026-11-15 才發現儀表板上唯一那顆「追蹤目標」打的其實是無人機。
//
//  用分區標題 + 按鈕文字定位：「追蹤目標」「返航」兩個分區都有，只看文字
//  會按錯台。
//
//  不需要另外安裝套件：WebSocket 用 Node-RED 自己帶的 ws。
// =============================================================================
const { spawn } = require("child_process");
const http = require("http");
const fs = require("fs");
const WebSocket = require("/usr/lib/node_modules/node-red/node_modules/ws");

const [GROUP, LABEL, SHOT = "/tmp/dash_click.png", WAIT_S = "15"] = process.argv.slice(2);
const URL = "http://localhost:1880/dashboard/control";
const PORT = 9333;
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

function getJson(path) {
    return new Promise((res, rej) => {
        http.get({ host: "127.0.0.1", port: PORT, path }, (r) => {
            let d = "";
            r.on("data", (c) => (d += c));
            r.on("end", () => { try { res(JSON.parse(d)); } catch (e) { rej(e); } });
        }).on("error", rej);
    });
}

(async () => {
    if (!GROUP || !LABEL) {
        console.error("用法：node click_dashboard.js <分區標題> <按鈕文字> [截圖] [等幾秒]");
        process.exit(2);
    }
    const chrome = spawn("google-chrome", ["--headless=new", "--disable-gpu",
        "--hide-scrollbars", "--window-size=1400,1600",
        `--remote-debugging-port=${PORT}`, `--user-data-dir=/tmp/cdp_profile_${PORT}`, URL],
        { stdio: "ignore" });
    let ws;
    const done = (code) => { try { ws && ws.close(); } catch (e) {} chrome.kill(); process.exit(code); };
    try {
        let pages = [];
        for (let i = 0; i < 40 && !pages.find((p) => p.type === "page"); i++) {
            await sleep(250);
            pages = await getJson("/json").catch(() => []);
        }
        const page = pages.find((p) => p.type === "page");
        if (!page) throw new Error("Chrome 沒有開出頁面");
        ws = new WebSocket(page.webSocketDebuggerUrl);
        await new Promise((r) => ws.on("open", r));
        let id = 0;
        const pending = {};
        ws.on("message", (m) => { const j = JSON.parse(m); if (pending[j.id]) { pending[j.id](j); delete pending[j.id]; } });
        const call = (method, params = {}) => new Promise((r) => { pending[++id] = r; ws.send(JSON.stringify({ id, method, params })); });
        // userGesture：讓 el.click() 算成「使用者點的」。不加的話 window.open 會被
        // Chrome 的彈出視窗攔截靜靜擋掉 —— 真人用滑鼠點不會，只有腳本點會。
        const evalJs = async (expr) => (await call("Runtime.evaluate", { expression: expr, returnByValue: true, userGesture: true })).result.result.value;

        // 等儀表板畫完（按鈕出現）
        let ready = false;
        for (let i = 0; i < 60 && !ready; i++) {
            await sleep(500);
            ready = await evalJs(`document.querySelectorAll(".nrdb-ui-group").length > 0`);
        }
        if (!ready) throw new Error("等不到儀表板的分區");
        await sleep(1500);

        const result = await evalJs(`(() => {
            const groups = [...document.querySelectorAll(".nrdb-ui-group")];
            const g = groups.find((el) => (el.querySelector(".v-card-title, .nrdb-ui-group-title, h2, h3") || {}).textContent?.trim() === ${JSON.stringify(GROUP)});
            if (!g) return "找不到分區：" + groups.map((el) => (el.querySelector(".v-card-title") || {}).textContent).join("、");
            const b = [...g.querySelectorAll("button")].find((el) => el.textContent.trim() === ${JSON.stringify(LABEL)});
            if (!b) return "分區裡找不到按鈕：" + [...g.querySelectorAll("button")].map((el) => el.textContent.trim()).join("、");
            b.click();
            return "OK";
        })()`);
        if (result !== "OK") throw new Error(result);
        console.log(`已按下「${GROUP}」的「${LABEL}」，等 ${WAIT_S} 秒後截圖`);
        await sleep(Number(WAIT_S) * 1000);
        const shot = await call("Page.captureScreenshot", { format: "png" });
        fs.writeFileSync(SHOT, Buffer.from(shot.result.data, "base64"));
        console.log("截圖：" + SHOT);

        // 按鈕如果開了新視窗（例如「無人機相機」用 window.open 開 /camera），
        // 只截儀表板會以為什麼都沒發生。把每個新視窗的網址印出來，也各截一張。
        const others = (await getJson("/json").catch(() => []))
            .filter((p) => p.type === "page" && p.id !== page.id);
        for (const [i, p] of others.entries()) {
            console.log("新視窗：" + p.url);
            const w = new WebSocket(p.webSocketDebuggerUrl);
            await new Promise((r) => w.on("open", r));
            const shotW = await new Promise((r) => {
                w.on("message", (m) => { const j = JSON.parse(m); if (j.id === 1) r(j); });
                w.send(JSON.stringify({ id: 1, method: "Page.captureScreenshot", params: { format: "png" } }));
            });
            const f = SHOT.replace(/\.png$/, "") + `_win${i + 1}.png`;
            fs.writeFileSync(f, Buffer.from(shotW.result.data, "base64"));
            console.log("新視窗截圖：" + f);
            w.close();
        }
        done(0);
    } catch (e) {
        console.error("✗ " + e.message);
        done(1);
    }
})();

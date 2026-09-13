// 页面冒烟测试:在 Node 中用 DOM/Three 桩真实加载 script.js,
// 覆盖 初始化 → 播放 → 模式切换 → 随机 的完整链路。
// 运行:node --import ./tests/stubs/register.mjs tests/smoke.mjs
const listeners = {};
function makeEl(id, tag = "div") {
  const el = {
    id, tag, listeners: {},
    style: {}, dataset: {}, value: id === "compare-delta" ? "0.1" : (id === "speed" ? "1" : (id === "trail-time" ? "33" : "2")),
    checked: true, textContent: "", innerHTML: "",
    clientWidth: 800, clientHeight: 600,
    classList: { toggle(){}, add(){}, remove(){} },
    addEventListener(ev, fn) { (listeners[id || tag] ||= {})[ev] = fn; },
    appendChild(child) { if (child && child.tag === "canvas") el._canvas = child; },
    append(...kids) { for (const k of kids) if (k && k.tag === "canvas") el._canvas = k; },
    fire() {
      const ls = listeners[id || tag] || {};
      for (const ev of ["click", "change", "input"]) {
        if (!ls[ev]) continue;
        if (ev === "click") ls[ev]();
        else ls[ev]({ target: { checked: el.checked, value: String(el.value) } });
      }
    },
  };
  return el;
}
const els = {};
globalThis.document = {
  getElementById(id) { return (els[id] ||= makeEl(id)); },
  createElement(tag) { return makeEl("(created)", tag); },
  createTextNode(t) { return { text: t }; },
  querySelectorAll(sel) {
    if (sel.includes("mode-segment")) {
      return ["planar", "spherical"].map((mode) => ({
        ...makeEl(`mode-${mode}`), dataset: { mode },
        addEventListener(ev, fn) { (listeners[`mode:${mode}`] ||= {})[ev] = fn; },
      }));
    }
    if (sel.includes("ui-level")) {
      return ["simple", "advanced"].map((level) => ({
        ...makeEl(`ui-${level}`), dataset: { level },
        addEventListener(ev, fn) { (listeners[`ui:${level}`] ||= {})[ev] = fn; },
      }));
    }
    return [];
  },
};
globalThis.window = { addEventListener(){} };
let rafCb = null;
globalThis.requestAnimationFrame = (cb) => { rafCb = cb; return 1; };

await import("../script.js");
console.log("PASS 初始化无异常");

function runFrames(count, dtMs = 16.7) {
  let t = 16;
  for (let f = 0; f < count; f++) {
    const cb = rafCb;
    if (!cb) throw new Error("渲染循环丢失");
    rafCb = null;
    cb(t);
    t += dtMs;
  }
}

// 播放 3 秒
els["play-toggle-btn"].fire();
runFrames(180);
console.log("PASS 播放 180 帧(平面/物理步进/采点/同步/数据面板)");

// 暂停 → 高级界面 → 切球面模式 → 再播放 1 秒(重建 + 球面积分)
els["play-toggle-btn"].fire();
listeners["ui:advanced"].click();
listeners["mode:spherical"].click();
els["play-toggle-btn"].fire();
runFrames(60);
console.log("PASS 球面模式重建 + 60 帧播放");

// 随机初始值(球面,含 φ/ωφ)→ 再播 1 秒
els["random-btn"].fire();
runFrames(60);
console.log("PASS 随机初始值 + 60 帧播放");

// 对比模式开启 → 60 帧(A/B 双摆)
els["compare-enable"].fire();
runFrames(60);
console.log("PASS 对比模式 + 60 帧双摆播放");

console.log("全部冒烟测试通过");

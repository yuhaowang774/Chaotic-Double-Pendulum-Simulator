// 把 "three" 与 OrbitControls 重定向到测试桩
export async function resolve(specifier, context, next) {
  if (specifier === "three") {
    return { shortCircuit: true, url: new URL("./three-stub.mjs", import.meta.url).href };
  }
  if (specifier === "three/addons/controls/OrbitControls.js") {
    return { shortCircuit: true, url: new URL("./orbit-stub.mjs", import.meta.url).href };
  }
  return next(specifier, context);
}

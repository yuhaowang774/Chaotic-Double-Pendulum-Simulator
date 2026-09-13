// Node 冒烟测试用的最小 three 桩:仅覆盖 script.js 实际用到的 API。
const AdditiveBlending = 1;
class Color {
  constructor() {}
  setHSL() { return this; }
}
class Vector3 {
  constructor(x = 0, y = 0, z = 0) { this.x = x; this.y = y; this.z = z; }
  set(x, y, z) { this.x = x; this.y = y; this.z = z; return this; }
}
class Quaternion { setFromUnitVectors() {} }
function stubElement(tag) {
  return { tag, addEventListener() {}, appendChild() {} };
}
class Object3D {
  constructor() {
    this.position = { set() {} };
    this.quaternion = new Quaternion();
    this.scale = { set() {} };
  }
  add() {}
  remove() {}
}
class Scene extends Object3D {}
class PerspectiveCamera extends Object3D {
  constructor() { super(); this.aspect = 1; this.updateProjectionMatrix = () => {}; this.lookAt = () => {}; }
}
class WebGLRenderer {
  constructor() { this.domElement = stubElement("canvas"); }
  setSize() {}
  setPixelRatio() {}
  render() {}
}
class AmbientLight extends Object3D {}
class DirectionalLight extends Object3D {}
class BufferAttribute { constructor(arr) { this.array = arr; } }
class BufferGeometry {
  constructor() { this.attributes = {}; }
  setAttribute(name, attr) { this.attributes[name] = attr; }
  setDrawRange() {}
  dispose() {}
}
class MeshStandardMaterial extends Object3D {
  constructor(o) { super(); Object.assign(this, o); this.dispose = () => {}; }
}
class LineBasicMaterial extends Object3D {
  constructor(o) { super(); Object.assign(this, o); this.dispose = () => {}; }
}
class SphereGeometry extends BufferGeometry {}
class CylinderGeometry extends BufferGeometry {}
class Mesh extends Object3D {
  constructor(g, m) { super(); this.geometry = g; this.material = m; }
}
class Line extends Object3D {
  constructor(g, m) { super(); this.geometry = g; this.material = m; }
}
class GridHelper extends Object3D {}
export {
  AdditiveBlending, Color, Vector3, Quaternion, Scene, PerspectiveCamera,
  WebGLRenderer, AmbientLight, DirectionalLight, BufferAttribute,
  BufferGeometry, MeshStandardMaterial, LineBasicMaterial, SphereGeometry,
  CylinderGeometry, Mesh, Line, GridHelper,
};

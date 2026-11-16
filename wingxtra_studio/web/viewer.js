// 3D view of the aircraft. Body axes (x forward, y right, z down) map to three.js (x, -z, y).
import * as THREE from 'three';
import { OrbitControls } from '/vendor/OrbitControls.js';

const KIND = {
  wing: { color: 0xf2f1ec }, tail: { color: 0xf2f1ec }, body: { color: 0x51667c },
  boom: { color: 0x2f3a46 }, rotor: { color: 0x2a78d6, opacity: 0.35 }, prop: { color: 0x2a78d6, opacity: 0.25 },
};

export class Viewer {
  constructor(container) {
    this.container = container;
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true, preserveDrawingBuffer: true });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    container.appendChild(this.renderer.domElement);
    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(32, 1, 0.01, 500);
    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.addEventListener('change', () => this.render());
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x8a94a3, 1.6));
    const sun = new THREE.DirectionalLight(0xffffff, 1.5);
    sun.position.set(2, 5, 3);
    this.scene.add(sun);
    this.group = new THREE.Group();
    this.scene.add(this.group);
    this.extent = 2;
    this.fitted = false;
    new ResizeObserver(() => this.resize()).observe(container);
    this.resize();
  }

  resize() {
    const w = this.container.clientWidth, h = this.container.clientHeight;
    if (!w || !h) return;
    this.renderer.setSize(w, h);
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.render();
  }

  render() { this.renderer.render(this.scene, this.camera); }

  marker(position, color, radius) {
    const mesh = new THREE.Mesh(new THREE.SphereGeometry(radius, 20, 14),
      new THREE.MeshBasicMaterial({ color, depthTest: false }));
    mesh.position.set(position[0], -position[2], position[1]);
    mesh.renderOrder = 10;
    return mesh;
  }

  setAircraft(mesh, neutralPointX) {
    for (const child of [...this.group.children]) {
      child.geometry.dispose(); child.material.dispose(); this.group.remove(child);
    }
    for (const part of mesh.parts) {
      const src = part.positions, pos = new Float32Array(src.length);
      for (let i = 0; i < src.length; i += 3) { pos[i] = src[i]; pos[i + 1] = -src[i + 2]; pos[i + 2] = src[i + 1]; }
      const geometry = new THREE.BufferGeometry();
      geometry.setAttribute('position', new THREE.BufferAttribute(pos, 3));
      geometry.setIndex(part.indices);
      geometry.computeVertexNormals();
      const style = KIND[part.kind] || KIND.body;
      const material = new THREE.MeshStandardMaterial({
        color: style.color, roughness: 0.65, metalness: 0.05, side: THREE.DoubleSide,
        transparent: style.opacity !== undefined, opacity: style.opacity ?? 1, depthWrite: style.opacity === undefined,
      });
      this.group.add(new THREE.Mesh(geometry, material));
    }
    const r = mesh.extent * 0.012;
    this.group.add(this.marker(mesh.cg, 0xeb6834, r));
    this.group.add(this.marker([neutralPointX, 0, mesh.cg[2]], 0x2a78d6, r));
    const rescaled = Math.abs(mesh.extent - this.extent) / this.extent > 0.25;
    this.extent = mesh.extent;
    if (!this.fitted || rescaled) { this.view('iso'); this.fitted = true; } else { this.render(); }
  }

  view(name) {
    const d = this.extent * 1.55;
    const dir = { iso: [0.75, 0.5, 0.85], top: [0.001, 1, 0], side: [0, 0.05, 1], front: [1, 0.08, 0] }[name] || [1, 1, 1];
    const v = new THREE.Vector3(...dir).normalize().multiplyScalar(d);
    const target = new THREE.Vector3(-this.extent * 0.15, 0, 0);
    this.camera.position.copy(target).add(v);
    this.controls.target.copy(target);
    this.controls.update();
    this.render();
  }
}

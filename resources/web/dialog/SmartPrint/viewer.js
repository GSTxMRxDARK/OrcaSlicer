// 3D view of the part: safety-factor colouring, motor / support / force markers and picking.
// Coordinates are the printer's (mm, Z up), the same as the analysis.

const SafetyScale = (() => {
  // Stops in safety factor, coloured from the CSS ramp so the map, legend and numbers agree.
  const stops = [
    [0.0, "--sf-break"], [1.0, "--sf-break"], [1.25, "--sf-thin"], [1.7, "--sf-ok"],
    [2.5, "--sf-good"], [4.0, "--sf-safe"]
  ];
  let colors = null;
  function css(name) {
    return new THREE.Color(getComputedStyle(document.documentElement).getPropertyValue(name).trim() || "#888");
  }
  function refresh() { colors = stops.map(([v, name]) => [v, css(name)]); }
  function color(sf, out) {
    if (!colors) refresh();
    if (!(sf > 0)) return out.copy(colors[0][1]);
    for (let i = 1; i < colors.length; ++i) {
      if (sf <= colors[i][0]) {
        const [v0, c0] = colors[i - 1];
        const [v1, c1] = colors[i];
        return out.copy(c0).lerp(c1, (sf - v0) / (v1 - v0));
      }
    }
    return out.copy(colors[colors.length - 1][1]);
  }
  function cssColor(sf) { return "#" + color(sf, new THREE.Color()).getHexString(); }
  return { color, cssColor, refresh };
})();

class PartViewer {
  constructor(container) {
    this.container = container;
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    this.renderer.setPixelRatio(window.devicePixelRatio || 1);
    container.appendChild(this.renderer.domElement);
    this.renderer.domElement.tabIndex = 0;

    this.scene = new THREE.Scene();
    this.camera = new THREE.PerspectiveCamera(35, 1, 1, 10000);
    this.camera.up.set(0, 0, 1);
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x9aa0a8, 2.0));
    const fill = new THREE.DirectionalLight(0xffffff, 0.6);
    fill.position.set(-1, 1.2, -0.4);
    this.scene.add(fill);
    const key = new THREE.DirectionalLight(0xffffff, 1.4);
    key.position.set(1, -1.5, 2);
    this.scene.add(key);
    this.key = key;

    this.mesh = null;
    this.markers = new THREE.Group();
    this.scene.add(this.markers);
    this.labels = [];       // {el, position}
    this.callout = null;    // {el, position}
    this.onPick = null;     // (point, normal) => void
    this.onMarkerClick = null;

    this.target = new THREE.Vector3();
    this.spherical = new THREE.Spherical(400, 1.0, -Math.PI / 2.6);
    this.installControls();
    new ResizeObserver(() => this.resize()).observe(container);
    this.resize();
    this.requestRender();
  }

  // ---- Model ----
  setMesh(vertices, indices) {
    if (this.mesh) {
      this.scene.remove(this.mesh);
      this.mesh.geometry.dispose();
    }
    // Flat shading needs unshared vertices; keep the source index to map per-vertex values later.
    const count = indices.length;
    const positions = new Float32Array(count * 3);
    for (let i = 0; i < count; ++i) {
      const v = indices[i] * 3;
      positions[i * 3] = vertices[v];
      positions[i * 3 + 1] = vertices[v + 1];
      positions[i * 3 + 2] = vertices[v + 2];
    }
    const geometry = new THREE.BufferGeometry();
    geometry.setAttribute("position", new THREE.BufferAttribute(positions, 3));
    geometry.setAttribute("color", new THREE.BufferAttribute(new Float32Array(count * 3).fill(0.72), 3));
    geometry.computeVertexNormals();
    geometry.computeBoundingBox();
    geometry.computeBoundingSphere();
    this.sourceIndex = indices;
    const material = new THREE.MeshStandardMaterial({ vertexColors: true, roughness: 0.75, metalness: 0.0, flatShading: true });
    this.mesh = new THREE.Mesh(geometry, material);
    this.scene.add(this.mesh);

    const box = geometry.boundingBox;
    this.bounds = box;
    box.getCenter(this.target);
    const size = box.getSize(new THREE.Vector3());
    const radius = Math.max(size.length() / 2, 10);
    this.fitRadius = radius;
    this.fit();
    this.camera.near = radius / 50;
    this.camera.far = radius * 50;
    this.camera.updateProjectionMatrix();
    this.addBed(box, radius);
    this.updateCamera();
  }

  // Distance that keeps the whole part in view for the current window shape.
  fit() {
    const vHalf = THREE.MathUtils.degToRad(this.camera.fov / 2);
    const hHalf = Math.atan(Math.tan(vHalf) * this.camera.aspect);
    this.spherical.radius = this.fitRadius / Math.sin(Math.min(vHalf, hHalf)) * 0.95;
    this.updateCamera();
  }

  addBed(box, radius) {
    if (this.bed) this.scene.remove(this.bed);
    const step = radius > 150 ? 20 : 10;
    const size = Math.ceil(radius * 2.6 / step) * step;
    const grid = new THREE.GridHelper(size, size / step, 0x8a9096, 0x8a9096);
    grid.rotation.x = Math.PI / 2;
    grid.position.set(this.target.x, this.target.y, box.min.z - 0.05);
    grid.material.transparent = true;
    grid.material.opacity = 0.28;
    this.bed = grid;
    this.scene.add(grid);
  }

  // Per source-vertex values (Float32Array) to colours; null resets to the neutral grey.
  setSafety(values) {
    if (!this.mesh) return;
    const colors = this.mesh.geometry.getAttribute("color");
    const c = new THREE.Color();
    const neutral = new THREE.Color(0.72, 0.74, 0.76);
    for (let i = 0; i < this.sourceIndex.length; ++i) {
      if (values) SafetyScale.color(values[this.sourceIndex[i]], c);
      else c.copy(neutral);
      colors.setXYZ(i, c.r, c.g, c.b);
    }
    colors.needsUpdate = true;
    this.requestRender();
  }

  // ---- Markers ----
  clearMarkers() {
    this.markers.children.slice().forEach((m) => {
      this.markers.remove(m);
      m.traverse((o) => { if (o.geometry) o.geometry.dispose(); });
    });
    this.labels.forEach((l) => l.el.remove());
    this.labels = [];
    this.requestRender();
  }

  accent() {
    return new THREE.Color(getComputedStyle(document.documentElement).getPropertyValue("--sp-accent").trim() || "#009688");
  }

  addDisc(center, radius, up, color, opacity, label, id) {
    const geom = new THREE.CylinderGeometry(radius, radius, Math.max(radius * 0.08, 0.6), 48);
    const mat = new THREE.MeshBasicMaterial({ color, transparent: true, opacity, depthWrite: false, depthTest: false });
    const disc = new THREE.Mesh(geom, mat);
    disc.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), up.clone().normalize());
    disc.position.copy(center).addScaledVector(up, 0.4);
    disc.renderOrder = 2;
    this.markers.add(disc);
    const ring = new THREE.Mesh(new THREE.TorusGeometry(radius, Math.max(radius * 0.04, 0.35), 8, 64), new THREE.MeshBasicMaterial({ color }));
    ring.quaternion.setFromUnitVectors(new THREE.Vector3(0, 0, 1), up.clone().normalize());
    ring.position.copy(disc.position);
    this.markers.add(ring);
    if (label !== undefined) this.addLabel(center.clone().addScaledVector(up, radius * 0.6), label, id);
  }

  addSupport(center, radius) {
    const color = new THREE.Color(0x3b6fd8);
    const sphere = new THREE.Mesh(new THREE.SphereGeometry(radius, 32, 16),
      new THREE.MeshBasicMaterial({ color, transparent: true, opacity: 0.22, depthWrite: false, depthTest: false }));
    sphere.position.copy(center);
    sphere.renderOrder = 3;
    this.markers.add(sphere);
    const ring = new THREE.Mesh(new THREE.TorusGeometry(radius, Math.max(radius * 0.03, 0.4), 8, 64), new THREE.MeshBasicMaterial({ color, depthTest: false }));
    ring.position.copy(center);
    ring.renderOrder = 4;
    this.markers.add(ring);
  }

  addForce(center, force) {
    const dir = force.clone();
    const len = dir.length();
    if (len <= 0) return;
    dir.normalize();
    const size = this.bounds ? this.bounds.getSize(new THREE.Vector3()).length() : 100;
    const arrowLen = size * 0.16;
    const origin = center.clone().addScaledVector(dir, -arrowLen);
    const arrow = new THREE.ArrowHelper(dir, origin, arrowLen, 0xd6402f, arrowLen * 0.28, arrowLen * 0.16);
    arrow.traverse((o) => { if (o.material) { o.material.depthTest = false; o.renderOrder = 5; } });
    this.markers.add(arrow);
  }

  addLabel(position, text, id) {
    const el = document.createElement("div");
    el.className = "marker-label";
    el.textContent = text;
    el.addEventListener("click", (e) => {
      e.stopPropagation();
      if (this.onMarkerClick) this.onMarkerClick(id);
    });
    this.container.appendChild(el);
    this.labels.push({ el, position: position.clone(), id });
    this.requestRender();
  }

  selectLabel(id) { this.labels.forEach((l) => l.el.classList.toggle("is-selected", l.id === id)); }

  setCallout(el, position) {
    this.callout = position ? { el, position: position.clone() } : null;
    el.hidden = !position;
    this.requestRender();
  }

  // ---- Camera ----
  installControls() {
    const el = this.renderer.domElement;
    let drag = null;
    el.addEventListener("contextmenu", (e) => e.preventDefault());
    el.addEventListener("pointerdown", (e) => {
      el.setPointerCapture(e.pointerId);
      drag = { x: e.clientX, y: e.clientY, startX: e.clientX, startY: e.clientY, button: e.button, pan: e.button === 2 || e.button === 1 || e.shiftKey };
    });
    el.addEventListener("pointermove", (e) => {
      if (!drag) return;
      const dx = e.clientX - drag.x, dy = e.clientY - drag.y;
      drag.x = e.clientX; drag.y = e.clientY;
      if (drag.pan) this.pan(dx, dy);
      else {
        this.spherical.theta -= dx * 0.008;
        this.spherical.phi = THREE.MathUtils.clamp(this.spherical.phi - dy * 0.008, 0.05, Math.PI - 0.05);
        this.updateCamera();
      }
    });
    el.addEventListener("pointerup", (e) => {
      if (drag && drag.button === 0 && Math.hypot(e.clientX - drag.startX, e.clientY - drag.startY) < 4) this.pick(e);
      drag = null;
    });
    el.addEventListener("wheel", (e) => {
      e.preventDefault();
      this.spherical.radius *= Math.pow(1.0015, e.deltaY);
      this.updateCamera();
    }, { passive: false });
    el.addEventListener("dblclick", () => { if (this.bounds) { this.bounds.getCenter(this.target); this.fit(); } });
  }

  pan(dx, dy) {
    const h = this.container.clientHeight || 1;
    const scale = 2 * this.spherical.radius * Math.tan(THREE.MathUtils.degToRad(this.camera.fov / 2)) / h;
    const right = new THREE.Vector3().setFromMatrixColumn(this.camera.matrix, 0);
    const up = new THREE.Vector3().setFromMatrixColumn(this.camera.matrix, 1);
    this.target.addScaledVector(right, -dx * scale).addScaledVector(up, dy * scale);
    this.updateCamera();
  }

  updateCamera() {
    // Spherical coordinates around +Z.
    const s = this.spherical;
    const x = s.radius * Math.sin(s.phi) * Math.cos(s.theta);
    const y = s.radius * Math.sin(s.phi) * Math.sin(s.theta);
    const z = s.radius * Math.cos(s.phi);
    this.camera.position.set(this.target.x + x, this.target.y + y, this.target.z + z);
    this.camera.lookAt(this.target);
    this.key.position.copy(this.camera.position).add(new THREE.Vector3(0, 0, s.radius));
    this.requestRender();
  }

  pick(event) {
    if (!this.mesh || !this.onPick) return;
    const rect = this.renderer.domElement.getBoundingClientRect();
    const ndc = new THREE.Vector2(((event.clientX - rect.left) / rect.width) * 2 - 1, -((event.clientY - rect.top) / rect.height) * 2 + 1);
    const ray = new THREE.Raycaster();
    ray.setFromCamera(ndc, this.camera);
    const hit = ray.intersectObject(this.mesh, false)[0];
    if (hit) this.onPick(hit.point.clone(), hit.face ? hit.face.normal.clone() : new THREE.Vector3(0, 0, 1));
  }

  resize() {
    const w = Math.max(this.container.clientWidth, 1), h = Math.max(this.container.clientHeight, 1);
    this.renderer.setSize(w, h, false);
    this.renderer.domElement.style.width = w + "px";
    this.renderer.domElement.style.height = h + "px";
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.requestRender();
  }

  requestRender() {
    if (this.pending) return;
    this.pending = true;
    requestAnimationFrame(() => {
      this.pending = false;
      this.renderer.render(this.scene, this.camera);
      this.placeOverlays();
    });
  }

  project(position) {
    const p = position.clone().project(this.camera);
    return { x: (p.x * 0.5 + 0.5) * this.container.clientWidth, y: (-p.y * 0.5 + 0.5) * this.container.clientHeight, visible: p.z < 1 };
  }

  placeOverlays() {
    for (const l of this.labels) {
      const p = this.project(l.position);
      l.el.style.left = p.x + "px";
      l.el.style.top = p.y + "px";
      const w = this.container.clientWidth, h = this.container.clientHeight;
      l.el.hidden = !p.visible || p.x < 0 || p.y < 0 || p.x > w || p.y > h;
    }
    if (this.callout) {
      const p = this.project(this.callout.position);
      this.callout.el.style.left = p.x + "px";
      this.callout.el.style.top = p.y + "px";
    }
  }
}

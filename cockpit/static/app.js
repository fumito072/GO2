/* GO2 COCKPIT frontend
 * WS: JSON(telemetry/ack) + binary(1=LiDAR点群, 2=ハイトマップ)
 * 座標系: odom (x前方, y左, z上)。画面マップは +x=上, +y=左。
 */
"use strict";

const $ = (id) => document.getElementById(id);

// ---------------- colormap (turbo風) ----------------
const CMAP_STOPS = [
  [0.00, [48, 18, 59]], [0.25, [62, 156, 254]], [0.50, [70, 247, 131]],
  [0.75, [249, 186, 56]], [1.00, [122, 4, 3]],
];
function cmap(t) {
  t = Math.min(1, Math.max(0, t));
  for (let i = 1; i < CMAP_STOPS.length; i++) {
    if (t <= CMAP_STOPS[i][0]) {
      const [t0, c0] = CMAP_STOPS[i - 1], [t1, c1] = CMAP_STOPS[i];
      const a = (t - t0) / (t1 - t0);
      return [0, 1, 2].map((k) => Math.round(c0[k] + a * (c1[k] - c0[k])));
    }
  }
  return CMAP_STOPS[CMAP_STOPS.length - 1][1];
}

// 色スケール範囲(m, odom z)。データの5-95%点へゆっくり追従。
let zRange = { lo: -0.1, hi: 1.0 };
const Z_MIN_SPAN = 0.6;   // 平坦な床でノイズが虹色にならないよう色域の最小幅を決める
function updateZRange(values) {
  if (!values.length) return;
  const s = Array.from(values).sort((a, b) => a - b);
  let lo = s[Math.floor(s.length * 0.05)], hi = s[Math.floor(s.length * 0.95)];
  const pad = Math.max(0.1, (hi - lo) * 0.1);
  lo -= pad;
  hi += pad;
  if (hi - lo < Z_MIN_SPAN) {          // 床だけが見えている間は色を暴れさせない
    const c = (lo + hi) / 2;
    lo = c - Z_MIN_SPAN / 2;
    hi = c + Z_MIN_SPAN / 2;
  }
  zRange.lo += 0.1 * (lo - zRange.lo);
  zRange.hi += 0.1 * (hi - zRange.hi);
}
const zNorm = (z) => (z - zRange.lo) / Math.max(0.05, zRange.hi - zRange.lo);

// ---------------- state ----------------
let ws = null;
let telem = {};
let pose = null;          // [x,y,z,yaw]
let armed = false;
let hmap = null;          // {cx, cy, res, n, data(Float32Array)}
let stair = null;         // 段差検出結果
let expmap = null;        // {ox, oy, res, w, h, cells(Uint8Array)} 占有格子
let explore = null;       // 探索タスク snapshot(telemetry経由)

function log(msg, cls) {
  const el = document.createElement("div");
  if (cls) el.className = cls;
  el.textContent = new Date().toTimeString().slice(0, 8) + " " + msg;
  const box = $("log");
  box.prepend(el);
  while (box.children.length > 60) box.lastChild.remove();
}

// ---------------- WebSocket ----------------
function connect() {
  ws = new WebSocket((location.protocol === "https:" ? "wss://" : "ws://") + location.host + "/ws");
  ws.binaryType = "arraybuffer";
  ws.onopen = () => {
    $("conn-dot").className = "dot ok";
    $("conn-text").textContent = "接続";
    log("サーバ接続", "ok");
    $("cam").src = "/video?ts=" + Date.now();  // MJPEG再接続
  };
  ws.onclose = () => {
    $("conn-dot").className = "dot ng";
    $("conn-text").textContent = "切断 — 再接続中…";
    setTimeout(connect, 1500);
  };
  ws.onmessage = (ev) => {
    if (typeof ev.data === "string") {
      const d = JSON.parse(ev.data);
      if (d.type === "telemetry") onTelemetry(d);
      else if (d.type === "ack") onAck(d);
    } else {
      if (!(ev.data instanceof ArrayBuffer) || ev.data.byteLength < 1) {
        console.warn("空または不正なWebSocket binary frameを破棄しました");
        return;
      }
      const view = new DataView(ev.data);
      const kind = view.getUint8(0);
      if (kind === 1) onLidar(ev.data);
      else if (kind === 2) onHeightmap(ev.data);
      else if (kind === 3) onExpmap(ev.data);
    }
  };
}

function send(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) ws.send(JSON.stringify(obj));
}

function onAck(d) {
  if (d.what === "arm") {
    setArmedUI(d.armed);
    log(d.armed ? "ARMED — 操縦有効" : "DISARMED", d.armed ? "err" : "ok");
  } else if (d.what && d.what.startsWith("explore")) {
    const r = d.result || {};
    if (typeof r === "string") { log(d.what + ": " + r, "ok"); return; }
    log("🗺 " + (r.say || r.kind || "ok"),
        ["error", "rejected", "busy"].includes(r.kind) ? "err" : "ok");
  } else {
    log(d.what + ": " + d.result, d.result === "ok" ? "ok" : "err");
  }
}

// ---------------- telemetry ----------------
function onTelemetry(d) {
  telem = d;
  if (d.pose) pose = d.pose;
  if (d.armed !== armed) setArmedUI(d.armed);

  $("mode-badge").textContent = d.mock ? "MOCK" : "REAL";
  $("mode-badge").className = "stat " + (d.mock ? "mock" : "real");
  $("batt").textContent = d.battery != null ? d.battery.toFixed(1) : "--";
  $("lowage").textContent = d.low_age_ms != null && d.low_age_ms < 1e6 ? d.low_age_ms.toFixed(0) : "--";
  $("cloudhz").textContent = d.cloud_hz != null ? d.cloud_hz.toFixed(1) : "--";
  $("posesrc").textContent = d.pose_src || "--";
  $("cam-age").textContent = d.cam_age != null ? d.cam_age.toFixed(1) + "s前" : "映像なし";

  const cloudDiag = $("cloud-diag");
  if (cloudDiag) {
    const status = d.cloud_status || "waiting";
    const frame = (d.cloud_frame || "--").replace(/^.*\//, "").toUpperCase();
    const raw = d.cloud_raw_n == null ? "--" : d.cloud_raw_n;
    const kept = d.cloud_ui_n == null ? "--" : d.cloud_ui_n;
    cloudDiag.textContent = status === "ok" ? frame + " " + raw + "→" + kept : status.toUpperCase();
    cloudDiag.className = "key-diag " + status;
    cloudDiag.title = "frame=" + (d.cloud_frame || "--")
      + " / raw=" + raw + " / UI=" + kept
      + " / elevation=" + (d.cloud_elev_n == null ? "--" : d.cloud_elev_n)
      + (d.cloud_error ? " / " + d.cloud_error : "");
  }

  if (d.rpy) {
    $("t-rpy").textContent = d.rpy.map((v) => (v * 57.296).toFixed(1) + "°").join(" ");
    drawHorizon(d.rpy[0], d.rpy[1]);
  }
  $("t-vel").textContent = d.vel ? d.vel.slice(0, 2).map((v) => v.toFixed(2)).join(", ") : "--";
  $("t-pos").textContent = d.pos ? d.pos.map((v) => v.toFixed(2)).join(", ") : "--";
  $("t-bh").textContent = d.body_height != null ? d.body_height.toFixed(3) : "--";
  $("t-cmd").textContent = d.cmd ? "vx=" + d.cmd[0] + " vy=" + d.cmd[1] + " wz=" + d.cmd[2] : "--";

  renderMission(d.mission);
  stair = d.stair || null;
  renderStair(d.stair, d.stair_task);
  renderRl(d.rl);
  renderExplore(d.explore);

  if (d.q && d.joint_names) {
    const tb = $("joints").tBodies[0];
    if (tb.rows.length !== 12) {
      tb.innerHTML = "";
      d.joint_names.forEach(() => {
        const r = tb.insertRow();
        for (let i = 0; i < 4; i++) r.insertCell();
      });
    }
    d.joint_names.forEach((n, i) => {
      const c = tb.rows[i].cells;
      c[0].textContent = n.replace("_joint", "");
      c[1].textContent = d.q[i].toFixed(3);
      c[2].textContent = d.dq[i].toFixed(2);
      c[3].textContent = (d.tau ? d.tau[i] : 0).toFixed(1);
    });
  }
  updateRobotMarker();
}

function setArmedUI(on) {
  armed = on;
  $("arm-toggle").checked = on;
  const lb = $("arm-label");
  lb.textContent = on ? "ARMED" : "DISARMED";
  lb.className = "arm-label" + (on ? " armed" : "");
  document.body.classList.toggle("disarmed", !on);
}

// ---------------- 姿勢インジケータ ----------------
function drawHorizon(roll, pitch) {
  const cv = $("horizon"), ctx = cv.getContext("2d");
  const W = cv.width, H = cv.height, R = W / 2 - 2;
  ctx.clearRect(0, 0, W, H);
  ctx.save();
  ctx.beginPath();
  ctx.arc(W / 2, H / 2, R, 0, 7);
  ctx.clip();
  ctx.translate(W / 2, H / 2);
  ctx.rotate(-roll);
  const py = pitch * 90;  // 1rad ≈ 90px
  ctx.fillStyle = "#123b59";
  ctx.fillRect(-W, -H * 2 + py, W * 2, H * 2);   // 空
  ctx.fillStyle = "#2a251c";
  ctx.fillRect(-W, py, W * 2, H * 2);            // 地面
  ctx.strokeStyle = "#e7f5fb";
  ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.moveTo(-W, py); ctx.lineTo(W, py); ctx.stroke();
  ctx.restore();
  // 機体シンボル
  ctx.strokeStyle = "#35d8ff"; ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(W / 2 - 22, H / 2); ctx.lineTo(W / 2 - 7, H / 2);
  ctx.moveTo(W / 2 + 7, H / 2); ctx.lineTo(W / 2 + 22, H / 2);
  ctx.arc(W / 2, H / 2, 3, 0, 7);
  ctx.stroke();
}

// ---------------- LiDAR レーダー 3D (three.js, z-up world座標そのまま) ----------------
// L1 の点群は odom(world)系なので、受信した点を「置き換え」ではなく蓄積していけば
// 走査済みの壁・階段がそのまま形として残る(= コンセプト画のような空間の点群)。
// 無限に増えないよう VOXEL 単位で重複除去し、上限を超えたら古い点から上書きする。
// 実機L1(cloud_deskewed)は機体周辺 ~2m の狭いパッチを毎スキャン11k点で返す。
// 粗いボクセルだと数千点に潰れて形が見えないので、細かめに刻んで密度を稼ぐ。
const ACC_MAX = 120000;    // 蓄積する点の上限(GPUに載る量。重ければ下げる)
const VOXEL = 0.025;       // [m] このセルに既に点があれば追加しない
const RECV_MAX = 8000;     // 1フレームで受け取る最大点数(サーバ側と同じ)
const KEEP_RADIUS = 10.0;  // [m] 機体からこれ以上離れた点は蓄積しない
const SWEEP_RATE = 1.5;    // [rad/s] ≒14 RPM
const SWEEP_SPREAD = 1.0;  // [rad] 尾を引く角度
const SWEEP_RADIUS = 4.0;  // [m] ビーム/リングの最大半径(実データの広がりに合わせる)
const PING_PERIOD = 2.6;   // [s] 拡がるピング波の周期

let scene, camera3, renderer, points, robotGroup;
let radarGroup, ringGroup, pingRing, sweepOn = true;
let sweepAng = 0, lastFrameT = 0;
// az は機体後方を基準にしたユーザー操作オフセット。低い斜視で壁面や蹴上げを読みやすくする。
const orbit = {
  az: -0.24, el: 0.34, r: 3.6,
  target: new THREE.Vector3(0, 0, 0), ready: false,
};
const egoTarget = new THREE.Vector3();

// 蓄積バッファ(リング): voxelキー → スロット。満杯になったら最古スロットを再利用。
const accSlot = new Map();
const accKey = new Array(ACC_MAX);
let accCount = 0, accHead = 0;

// 高さ→色、実受信からの経過時間、距離による奥行き表現はGPU側で計算する。
// aSeen は「初めて見えた時刻」ではなく「最後に実データで観測した時刻」。
// 画面上の装飾スイープは点群を再発光させず、freshness は実際の受信だけで決まる。
const PT_VERT = `
attribute float aSeen;
uniform vec3 uRobot;
uniform float uSize;     // 点の大きさ [m]
uniform float uScale;    // 描画バッファ高さ/2 [px] — 距離減衰の基準
uniform float uTime;
uniform float uZLo;
uniform float uZHi;
varying vec3 vColor;
varying float vFresh;
varying float vViewDepth;
varying float vRange;

// レーダー用の高さランプ: 低い=深い青 → シアン → 高い=白熱。虹色より空間の形が読める。
vec3 ramp(float t) {
  t = clamp(t, 0.0, 1.0);
  vec3 c0 = vec3(0.031, 0.200, 0.310);   // #08334f 低い(床より下)
  vec3 c1 = vec3(0.122, 0.560, 0.769);   // #1f8fc4
  vec3 c2 = vec3(0.208, 0.847, 1.000);   // #35d8ff アクセント
  vec3 c3 = vec3(0.263, 0.937, 0.816);   // #43efd0 ミント
  vec3 c4 = vec3(0.918, 1.000, 0.976);   // #eafff9 高い(壁/障害物)
  if (t < 0.35) return mix(c0, c1, t / 0.35);
  if (t < 0.60) return mix(c1, c2, (t - 0.35) / 0.25);
  if (t < 0.82) return mix(c2, c3, (t - 0.60) / 0.22);
  return mix(c3, c4, (t - 0.82) / 0.18);
}

void main() {
  vColor = ramp((position.z - uZLo) / max(0.05, uZHi - uZLo));
  vec3 d = position - uRobot;
  vRange = length(d.xy);
  float age = max(0.0, uTime - aSeen);
  vFresh = exp(-age * 1.6);                 // 5Hz受信中は明るく、遮蔽後は約2秒で履歴色へ
  vec4 mv = modelViewMatrix * vec4(position, 1.0);
  vViewDepth = max(0.0, -mv.z);
  // perspectiveに加えてfresh hitを少し大きくし、近傍/遠方の判別を助ける。
  float ps = uSize * (0.9 + 0.65 * vFresh) * (uScale / max(0.8, vViewDepth));
  gl_PointSize = clamp(ps, 2.0, 7.5);
  gl_Position = projectionMatrix * mv;
}`;

const PT_FRAG = `
precision mediump float;
varying vec3 vColor;
varying float vFresh;
varying float vViewDepth;
varying float vRange;
void main() {
  vec2 pc = gl_PointCoord - 0.5;
  float r = length(pc);
  if (r > 0.5) discard;
  float soft = smoothstep(0.5, 0.10, r);                 // 丸くソフトな点
  if (soft < 0.04) discard;                              // 透明な縁で奥の点を隠さない
  float depthCue = 1.0 - smoothstep(5.0, 12.0, vViewDepth);
  float rangeCue = 1.0 - smoothstep(6.0, 10.0, vRange);
  vec3 history = vColor * 0.52;
  vec3 recent = min(vColor * 1.08 + vec3(0.12, 0.28, 0.34) * vFresh, vec3(1.0));
  vec3 c = mix(history, recent, vFresh) * (0.68 + 0.32 * depthCue);
  float alpha = soft * (0.10 + 0.90 * vFresh)
              * (0.62 + 0.38 * depthCue) * (0.78 + 0.22 * rangeCue);
  gl_FragColor = vec4(c, alpha);
}`;

/** レーダーのスイープ扇形。先端(角度0)が最も明るく、尾に向かって暗くなる。 */
function makeSweepWedge() {
  const segs = 36, pos = [], col = [];
  const base = new THREE.Color(0x35d8ff);
  for (let i = 0; i < segs; i++) {
    const a0 = -SWEEP_SPREAD * (i / segs), a1 = -SWEEP_SPREAD * ((i + 1) / segs);
    const f0 = Math.pow(1 - i / segs, 2.0), f1 = Math.pow(1 - (i + 1) / segs, 2.0);
    pos.push(0, 0, 0,
             SWEEP_RADIUS * Math.cos(a0), SWEEP_RADIUS * Math.sin(a0), 0,
             SWEEP_RADIUS * Math.cos(a1), SWEEP_RADIUS * Math.sin(a1), 0);
    // 中心は明るく、外周・尾は暗く(加算合成なので色がそのまま強度)
    const fc = (f0 + f1) / 2;
    col.push(base.r * 0.5 * fc, base.g * 0.5 * fc, base.b * 0.5 * fc,
             base.r * 0.26 * f0, base.g * 0.26 * f0, base.b * 0.26 * f0,
             base.r * 0.26 * f1, base.g * 0.26 * f1, base.b * 0.26 * f1);
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute("position", new THREE.Float32BufferAttribute(pos, 3));
  g.setAttribute("color", new THREE.Float32BufferAttribute(col, 3));
  return new THREE.Mesh(g, new THREE.MeshBasicMaterial({
    vertexColors: true, transparent: true, opacity: 0.18,
    depthTest: false, depthWrite: false,
    blending: THREE.AdditiveBlending, side: THREE.DoubleSide, toneMapped: false,
  }));
}

function circleGeom(r, segs = 96) {
  const p = [];
  for (let i = 0; i < segs; i++) {
    const a = (i / segs) * Math.PI * 2;
    p.push(r * Math.cos(a), r * Math.sin(a), 0);
  }
  const g = new THREE.BufferGeometry();
  g.setAttribute("position", new THREE.Float32BufferAttribute(p, 3));
  return g;
}

function initLidar3D() {
  const box = $("lidar3d");
  try {
    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
  } catch (e) {
    box.innerHTML = '<div style="padding:20px;color:#86aec2">WebGLが利用できないため3D表示は無効です。<br>ハイトマップ/カメラ/操縦は使用できます。</div>';
    return;
  }
  renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 2));
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x01070d);
  scene.fog = new THREE.Fog(0x01070d, 9, 26);
  camera3 = new THREE.PerspectiveCamera(55, 1, 0.1, 200);
  camera3.up.set(0, 0, 1);
  box.appendChild(renderer.domElement);

  const grid = new THREE.GridHelper(20, 20, 0x1d4d64, 0x0c2434);
  grid.rotation.x = Math.PI / 2;  // XY平面(z-up)に
  grid.material.transparent = true;
  grid.material.opacity = 0.5;
  grid.material.depthWrite = false;
  grid.renderOrder = 0;
  scene.add(grid);

  const geo = new THREE.BufferGeometry();
  geo.setAttribute("position", new THREE.BufferAttribute(new Float32Array(ACC_MAX * 3), 3));
  geo.setAttribute("aSeen", new THREE.BufferAttribute(new Float32Array(ACC_MAX), 1));
  geo.setDrawRange(0, 0);
  points = new THREE.Points(geo, new THREE.ShaderMaterial({
    uniforms: {
      uRobot: { value: new THREE.Vector3() },
      uSize: { value: 0.06 },     // [m]
      uScale: { value: 300 },     // resize() で実バッファ高さに合わせる
      uTime: { value: 0 },
      uZLo: { value: -0.1 },
      uZHi: { value: 1.0 },
    },
    vertexShader: PT_VERT, fragmentShader: PT_FRAG,
    // 点群自身で深度を書き、手前の面が奥の点を隠すことで立体形状を読みやすくする。
    transparent: true, depthTest: true, depthWrite: true,
    blending: THREE.NormalBlending,
  }));
  points.frustumCulled = false;
  points.renderOrder = 3;
  scene.add(points);

  // --- レーダー: 回転するスイープ扇形 + 先端ビーム(ロボット位置に追従) ---
  radarGroup = new THREE.Group();
  radarGroup.add(makeSweepWedge());
  const beamGeo = new THREE.BufferGeometry();
  beamGeo.setAttribute("position",
    new THREE.Float32BufferAttribute([0, 0, 0, SWEEP_RADIUS, 0, 0], 3));
  radarGroup.add(new THREE.Line(beamGeo, new THREE.LineBasicMaterial({
    color: 0x8df3ff, transparent: true, opacity: 0.42,
    blending: THREE.AdditiveBlending, depthTest: false, depthWrite: false,
  })));
  radarGroup.traverse((o) => { o.renderOrder = 1; });
  scene.add(radarGroup);

  // --- 距離リング(1m刻み) + 拡がるピング波。回転しないので別グループ ---
  ringGroup = new THREE.Group();
  for (let r = 1; r <= 4; r++) {
    ringGroup.add(new THREE.LineLoop(circleGeom(r), new THREE.LineBasicMaterial({
      color: r === 4 ? 0x43efd0 : 0x2b6f8c,
      transparent: true, opacity: r === 4 ? 0.34 : 0.20,
      depthTest: false, depthWrite: false,
    })));
  }
  pingRing = new THREE.LineLoop(circleGeom(1), new THREE.LineBasicMaterial({
    color: 0x35d8ff, transparent: true, opacity: 0.32,
    blending: THREE.AdditiveBlending, depthTest: false, depthWrite: false,
  }));
  ringGroup.add(pingRing);
  ringGroup.traverse((o) => { o.renderOrder = 1; });
  scene.add(ringGroup);

  // ロボットマーカー
  robotGroup = new THREE.Group();
  const body = new THREE.Mesh(new THREE.BoxGeometry(0.65, 0.31, 0.22),
    new THREE.MeshBasicMaterial({ color: 0x43efd0, wireframe: true }));
  robotGroup.add(body);
  const nose = new THREE.Mesh(new THREE.ConeGeometry(0.09, 0.28, 12),
    new THREE.MeshBasicMaterial({ color: 0xffad0a }));
  nose.rotation.z = -Math.PI / 2;  // coneは+y向き → +x向きに
  nose.position.x = 0.45;
  robotGroup.add(nose);
  scene.add(robotGroup);

  // 操作: ドラッグ=回転 / ホイール=ズーム
  let drag = null;
  box.addEventListener("pointerdown", (e) => { drag = [e.clientX, e.clientY]; box.setPointerCapture(e.pointerId); });
  box.addEventListener("pointermove", (e) => {
    if (!drag) return;
    orbit.az -= (e.clientX - drag[0]) * 0.008;
    orbit.el = Math.min(1.5, Math.max(0.05, orbit.el + (e.clientY - drag[1]) * 0.006));
    drag = [e.clientX, e.clientY];
  });
  box.addEventListener("pointerup", () => { drag = null; });
  box.addEventListener("wheel", (e) => {
    e.preventDefault();
    orbit.r = Math.min(30, Math.max(1.5, orbit.r * (e.deltaY > 0 ? 1.12 : 0.89)));
  }, { passive: false });

  const resize = () => {
    const w = box.clientWidth, h = box.clientHeight;
    if (!w || !h) return;
    renderer.setSize(w, h);
    camera3.aspect = w / h;
    camera3.updateProjectionMatrix();
    points.material.uniforms.uScale.value = renderer.domElement.height / 2;
  };
  new ResizeObserver(resize).observe(box);
  resize();
  $("sweep-info").textContent = (SWEEP_RATE / (2 * Math.PI) * 60).toFixed(0) + " RPM";

  (function animate(now) {
    requestAnimationFrame(animate);
    const dt = lastFrameT ? Math.min(0.1, (now - lastFrameT) / 1000) : 0;
    lastFrameT = now;
    if (sweepOn) sweepAng = (sweepAng + dt * SWEEP_RATE) % (Math.PI * 2);

    // 地面(足元)にレーダー面を置く。base高から公称の脚長ぶん下げる。
    const gz = pose ? pose[2] - 0.31 : 0;
    const u = points.material.uniforms;
    if (pose) {
      radarGroup.position.set(pose[0], pose[1], gz + 0.01);
      ringGroup.position.set(pose[0], pose[1], gz + 0.005);
      u.uRobot.value.set(pose[0], pose[1], pose[2]);
    }
    radarGroup.rotation.z = sweepAng;
    radarGroup.visible = sweepOn;
    u.uTime.value = now / 1000;
    u.uZLo.value = zRange.lo;      // 色スケールは全蓄積点に一括で効く(GPU側で色付け)
    u.uZHi.value = zRange.hi;

    // ピング波: 周期ごとに 0.3m → SWEEP_RADIUS へ拡がりながら消える
    const ph = sweepOn ? ((now / 1000) % PING_PERIOD) / PING_PERIOD : 0;
    pingRing.visible = sweepOn && ph < 0.85;
    pingRing.scale.setScalar(0.3 + ph * SWEEP_RADIUS);
    pingRing.material.opacity = 0.32 * Math.max(0, 1 - ph / 0.85);

    // 機体の後方から前方を見るego視点。少し前を注視して機体を画面下寄りに置く。
    const yaw = pose ? pose[3] : 0;
    if (pose) {
      egoTarget.set(
        pose[0] + Math.cos(yaw) * 0.65,
        pose[1] + Math.sin(yaw) * 0.65,
        gz + 0.38);
      const follow = orbit.ready ? 1 - Math.exp(-dt * 8.0) : 1;
      orbit.target.lerp(egoTarget, follow);
      orbit.ready = true;
    }
    const t = orbit.target;
    const cameraAz = yaw + Math.PI + orbit.az;
    camera3.position.set(
      t.x + orbit.r * Math.cos(orbit.el) * Math.cos(cameraAz),
      t.y + orbit.r * Math.cos(orbit.el) * Math.sin(cameraAz),
      t.z + orbit.r * Math.sin(orbit.el));
    camera3.lookAt(t);
    renderer.render(scene, camera3);
  })(0);
}

$("btn-sweep").onclick = () => {
  sweepOn = !sweepOn;
  $("btn-sweep").classList.toggle("active", sweepOn);
  $("sweep-info").textContent = sweepOn
    ? (SWEEP_RATE / (2 * Math.PI) * 60).toFixed(0) + " RPM" : "STATIC";
};
$("btn-clear").onclick = () => {
  clearCloud();
  log("LiDAR: 蓄積点群を消去 — 再スキャンします");
};

function clearCloud() {
  accSlot.clear();
  accCount = 0;
  accHead = 0;
  if (points) points.geometry.setDrawRange(0, 0);
  $("pt-count").textContent = "0 RX / 0 MAP";
}

function onLidar(buf) {
  if (!points) return;  // WebGL無効時
  if (!(buf instanceof ArrayBuffer) || buf.byteLength < 5) {
    console.warn("LiDAR frameが短すぎます", buf && buf.byteLength);
    return;
  }
  const declared = new DataView(buf).getUint32(1, true);
  const expectedBytes = 5 + declared * 12;
  if (buf.byteLength !== expectedBytes) {
    console.warn("LiDAR frameの点数とpayload長が不一致のため破棄しました",
                 declared, buf.byteLength, expectedBytes);
    return;
  }
  const n = Math.min(declared, RECV_MAX);
  if (!n) return;
  const xyz = new Float32Array(buf.slice(5, 5 + n * 12));
  const pos = points.geometry.attributes.position.array;
  const seen = points.geometry.attributes.aSeen.array;
  const zs = [];
  for (let i = 0; i < n; i++) {
    const z = xyz[i * 3 + 2];
    if (Number.isFinite(z)) zs.push(z);
  }
  updateZRange(zs);

  const now = performance.now() / 1000;
  const px = pose ? pose[0] : 0, py = pose ? pose[1] : 0;
  const inv = 1 / VOXEL;
  let rxCount = 0;
  for (let i = 0; i < n; i++) {
    const x = xyz[i * 3], y = xyz[i * 3 + 1], z = xyz[i * 3 + 2];
    if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z)) continue;
    if ((x - px) ** 2 + (y - py) ** 2 > KEEP_RADIUS * KEEP_RADIUS) continue;
    rxCount++;
    // 同じボクセルはslotを再利用し、座標とlastSeenを実受信時刻で更新する。
    // これにより履歴の形は残しつつ、今まさに観測できている面だけが明るくなる。
    // odom座標は歩くほど大きくなるので、衝突しない一意キーにする(±1400m まで安全)。
    const ix = Math.round(x * inv) + 32768;
    const iy = Math.round(y * inv) + 32768;
    const iz = Math.round(z * inv) + 512;
    const key = (ix * 65536 + iy) * 1024 + iz;
    let slot = accSlot.get(key);
    if (slot === undefined) {
      slot = accHead;
      if (accCount === ACC_MAX) accSlot.delete(accKey[slot]); // 最古の点を追い出す
      accSlot.set(key, slot);
      accKey[slot] = key;
      accHead = (accHead + 1) % ACC_MAX;
      accCount = Math.min(accCount + 1, ACC_MAX);
    }
    pos[slot * 3] = x;
    pos[slot * 3 + 1] = y;
    pos[slot * 3 + 2] = z;
    seen[slot] = now;
  }
  points.geometry.setDrawRange(0, accCount);
  points.geometry.attributes.position.needsUpdate = true;
  points.geometry.attributes.aSeen.needsUpdate = true;
  const fmtPts = (v) => v >= 10000 ? (v / 1000).toFixed(1) + "k" : String(v);
  $("pt-count").textContent = fmtPts(rxCount) + " RX / " + fmtPts(accCount) + " MAP";
}

function updateRobotMarker() {
  if (!pose || !robotGroup) return;
  robotGroup.position.set(pose[0], pose[1], pose[2]);
  robotGroup.rotation.z = pose[3];
}

// ---------------- カメラHUD (高度テープ / ピッチテープ) ----------------
// 高度 = pose[2] (odom base z)。段差を検出していれば「登り切った後の高さ」を目標として出す。
const hudCv = $("cam-hud");
const hudCtx = hudCv.getContext("2d");
let hudW = 0, hudH = 0;
let hudAlt = null, hudPitch = 0, hudRoll = 0;   // 表示用の平滑値

function resizeHud() {
  const box = $("cam-wrap");
  const dpr = Math.min(window.devicePixelRatio || 1, 2);
  hudW = box.clientWidth;
  hudH = box.clientHeight;
  hudCv.width = Math.round(hudW * dpr);
  hudCv.height = Math.round(hudH * dpr);
  hudCtx.setTransform(dpr, 0, 0, dpr, 0, 0);
}
new ResizeObserver(resizeHud).observe($("cam-wrap"));
resizeHud();

/** 縦の目盛テープ。中央=現在値、上ほど大。 */
function drawTape(ctx, o) {
  const { x, cy, h, value, step, majorEvery, span, unit, digits, label, side } = o;
  const pxPerUnit = h / span;
  const dir = side === "right" ? 1 : -1;   // 目盛が伸びる向き
  ctx.save();
  ctx.font = "10px ui-monospace, monospace";
  ctx.textBaseline = "middle";

  // 軸
  const grad = ctx.createLinearGradient(0, cy - h / 2, 0, cy + h / 2);
  grad.addColorStop(0, "rgba(53,216,255,0)");
  grad.addColorStop(0.5, "rgba(53,216,255,.75)");
  grad.addColorStop(1, "rgba(53,216,255,0)");
  ctx.strokeStyle = grad;
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(x, cy - h / 2); ctx.lineTo(x, cy + h / 2);
  ctx.stroke();

  // ラベル
  ctx.fillStyle = "rgba(134,174,194,.9)";
  ctx.textAlign = side === "right" ? "left" : "right";
  ctx.fillText(label, x + dir * 4, cy - h / 2 - 11);

  if (value == null) {
    ctx.fillStyle = "rgba(134,174,194,.6)";
    ctx.fillText("--", x + dir * 8, cy);
    ctx.restore();
    return;
  }

  // 目盛
  const lo = value - span / 2, hi = value + span / 2;
  const i0 = Math.ceil(lo / step);
  for (let i = i0; i * step <= hi; i++) {
    const v = i * step;
    const y = cy - (v - value) * pxPerUnit;
    const major = i % majorEvery === 0;
    const fade = 1 - Math.min(1, Math.abs(y - cy) / (h / 2)) * 0.75;
    ctx.strokeStyle = `rgba(134,174,194,${(major ? 0.95 : 0.5) * fade})`;
    ctx.lineWidth = major ? 1.4 : 1;
    ctx.beginPath();
    ctx.moveTo(x, y);
    ctx.lineTo(x + dir * (major ? 11 : 6), y);
    ctx.stroke();
    if (major) {
      ctx.fillStyle = `rgba(207,233,245,${0.95 * fade})`;
      ctx.textAlign = side === "right" ? "left" : "right";
      ctx.fillText(v.toFixed(digits), x + dir * 15, y);
    }
  }

  // 現在値の読み取り窓
  const bw = 58, bh = 19;
  const bx = side === "right" ? x - bw - 5 : x + 5;
  ctx.fillStyle = "rgba(3,16,26,.86)";
  ctx.strokeStyle = "#35d8ff";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.rect(bx, cy - bh / 2, bw, bh);
  ctx.fill();
  ctx.stroke();
  ctx.beginPath();   // 軸を指す三角
  const tipX = side === "right" ? bx + bw : bx;
  ctx.moveTo(tipX, cy - 5); ctx.lineTo(tipX + dir * 6, cy); ctx.lineTo(tipX, cy + 5);
  ctx.closePath();
  ctx.fillStyle = "#35d8ff";
  ctx.fill();
  ctx.fillStyle = "#e7f5fb";
  ctx.textAlign = "center";
  ctx.font = "bold 12px ui-monospace, monospace";
  ctx.fillText(value.toFixed(digits) + unit, bx + bw / 2, cy + 1);
  ctx.restore();
}

/** テープ上の任意の値に印を付ける(GND / 段差の頂点)。 */
function tapeMark(ctx, x, cy, h, span, value, at, color, text) {
  const y = cy - (at - value) * (h / span);
  if (y < cy - h / 2 - 2 || y > cy + h / 2 + 2) return;
  ctx.save();
  ctx.strokeStyle = color;
  ctx.fillStyle = color;
  ctx.lineWidth = 1.5;
  ctx.setLineDash([5, 3]);
  ctx.beginPath();
  ctx.moveTo(x - 26, y); ctx.lineTo(x, y);
  ctx.stroke();
  ctx.setLineDash([]);
  ctx.font = "9px ui-monospace, monospace";
  ctx.textAlign = "right";
  ctx.textBaseline = "bottom";
  ctx.fillText(text, x - 2, y - 2);
  ctx.restore();
}

function drawCamHud() {
  requestAnimationFrame(drawCamHud);
  if (!hudW || !hudH) return;
  const ctx = hudCtx;
  ctx.clearRect(0, 0, hudW, hudH);
  if (hudW < 300 || hudH < 190) return;   // 狭すぎる時はHUDを出さない

  // 平滑化(テレメトリは10Hz、描画は60Hz)
  const altT = pose ? pose[2] : null;
  if (altT != null) hudAlt = hudAlt == null ? altT : hudAlt + 0.18 * (altT - hudAlt);
  const rpy = telem.rpy || [0, 0, 0];
  hudPitch += 0.18 * (rpy[1] * 57.296 - hudPitch);
  hudRoll += 0.18 * (rpy[0] * 57.296 - hudRoll);

  const cy = hudH / 2;
  const h = Math.min(hudH - 66, 300);
  const xR = hudW - 56;   // 右: 高度
  const xL = 56;          // 左: ピッチ

  const ALT_SPAN = 0.9;   // ±0.45 m
  drawTape(ctx, { x: xR, cy, h, value: hudAlt, step: 0.05, majorEvery: 2,
                  span: ALT_SPAN, unit: "", digits: 2, label: "ALT m", side: "right" });
  if (hudAlt != null) {
    // 段差の頂点(登り切ったときのbase高) と odom原点(GND)
    if (stair && (stair.kind === "step" || stair.kind === "stairs") && stair.height) {
      tapeMark(ctx, xR, cy, h, ALT_SPAN, hudAlt, hudAlt + stair.height, "#ffad0a",
               "STEP +" + stair.height.toFixed(2));
    }
    // odom原点の平面 = オドメトリ開始時の床。登った量はこの線との差で読める。
    tapeMark(ctx, xR, cy, h, ALT_SPAN, hudAlt, 0.0, "rgba(89,239,131,.85)", "FLOOR 0");
  }
  drawTape(ctx, { x: xL, cy, h, value: pose ? hudPitch : null, step: 2.5, majorEvery: 4,
                  span: 45, unit: "", digits: 0, label: "PITCH °", side: "left" });

  // ロールを示す水平基準線(中央、機体シンボルの左右)
  ctx.save();
  ctx.translate(hudW / 2, cy);
  ctx.rotate(-hudRoll * Math.PI / 180);
  ctx.strokeStyle = "rgba(67,239,208,.55)";
  ctx.lineWidth = 1;
  ctx.beginPath();
  ctx.moveTo(-70, 0); ctx.lineTo(-30, 0);
  ctx.moveTo(30, 0); ctx.lineTo(70, 0);
  ctx.stroke();
  ctx.restore();
}
drawCamHud();

// ---------------- ハイトマップ ----------------
function onHeightmap(buf) {
  if (!(buf instanceof ArrayBuffer) || buf.byteLength < 15) {
    console.warn("ハイトマップframeが短すぎるため破棄しました", buf && buf.byteLength);
    return;
  }
  const v = new DataView(buf);
  const cx = v.getFloat32(1, true), cy = v.getFloat32(5, true);
  const res = v.getFloat32(9, true), n = v.getUint16(13, true);
  const expectedBytes = 15 + n * n * 4;
  if (!n || n > 512 || buf.byteLength !== expectedBytes) {
    console.warn("ハイトマップframeのshapeとpayload長が不一致のため破棄しました",
                 n, buf.byteLength, expectedBytes);
    return;
  }
  hmap = { cx, cy, res, n, data: new Float32Array(buf.slice(15, 15 + n * n * 4)) };
  drawHeightmap();
}

function drawHeightmap() {
  if (!hmap) return;
  const cv = $("hmap"), ctx = cv.getContext("2d");
  const { n, data, cx, cy, res } = hmap;
  const cell = cv.width / n;
  ctx.fillStyle = "#020a11";
  ctx.fillRect(0, 0, cv.width, cv.height);
  // 画面: 上=+x(前方), 左=+y
  for (let i = 0; i < n; i++) {          // i ↔ world x
    for (let j = 0; j < n; j++) {        // j ↔ world y
      const h = data[i * n + j];
      if (!isFinite(h)) continue;
      const c = cmap(zNorm(h));
      ctx.fillStyle = `rgb(${c[0]},${c[1]},${c[2]})`;
      ctx.fillRect((n - 1 - j) * cell, (n - 1 - i) * cell, cell + 0.5, cell + 0.5);
    }
  }
  if (pose) {
    const toScr = (wx, wy) => [
      (n - 1 - ((wy - cy) / res + n / 2)) * cell + cell / 2,
      (n - 1 - ((wx - cx) / res + n / 2)) * cell + cell / 2];
    const [sx, sy] = toScr(pose[0], pose[1]);
    const yaw = pose[3];
    // 方策の height_scan footprint (body系 x±0.8, y±0.5) を破線で表示
    ctx.strokeStyle = "#e7f5fb88";
    ctx.setLineDash([4, 4]);
    ctx.beginPath();
    [[0.8, 0.5], [0.8, -0.5], [-0.8, -0.5], [-0.8, 0.5]].forEach(([bx, by], k) => {
      const wx = pose[0] + bx * Math.cos(yaw) - by * Math.sin(yaw);
      const wy = pose[1] + bx * Math.sin(yaw) + by * Math.cos(yaw);
      const [x, y] = toScr(wx, wy);
      k === 0 ? ctx.moveTo(x, y) : ctx.lineTo(x, y);
    });
    ctx.closePath();
    ctx.stroke();
    ctx.setLineDash([]);
    drawStairOverlay(ctx, toScr);
    // ロボット三角形
    ctx.save();
    ctx.translate(sx, sy);
    ctx.rotate(-yaw);           // 画面上=+x なので -yaw 回転
    ctx.fillStyle = "#43efd0";
    ctx.beginPath();
    ctx.moveTo(0, -10); ctx.lineTo(6, 8); ctx.lineTo(-6, 8);
    ctx.closePath(); ctx.fill();
    ctx.restore();
  }
  drawLegend();
}

function drawLegend() {
  const cv = $("hmap-legend"), ctx = cv.getContext("2d");
  const H = cv.height, W = cv.width;
  ctx.clearRect(0, 0, W, H);
  for (let y = 0; y < H; y++) {
    const c = cmap(1 - y / H);
    ctx.fillStyle = `rgb(${c[0]},${c[1]},${c[2]})`;
    ctx.fillRect(0, y, 14, 1);
  }
  ctx.fillStyle = "#86aec2";
  ctx.font = "10px monospace";
  ctx.fillText(zRange.hi.toFixed(2), 17, 10);
  ctx.fillText(((zRange.hi + zRange.lo) / 2).toFixed(2), 17, H / 2 + 3);
  ctx.fillText(zRange.lo.toFixed(2), 17, H - 3);
}

// ホバーで高さ読み取り
$("hmap").addEventListener("mousemove", (e) => {
  if (!hmap) return;
  const r = e.target.getBoundingClientRect();
  const { n, data, cx, cy, res } = hmap;
  const cell = r.width / n;
  const j = n - 1 - Math.floor((e.clientX - r.left) / cell);
  const i = n - 1 - Math.floor((e.clientY - r.top) / cell);
  if (i < 0 || i >= n || j < 0 || j >= n) return;
  const h = data[i * n + j];
  const wx = cx + (i - n / 2) * res, wy = cy + (j - n / 2) * res;
  $("hmap-info").textContent =
    `(${wx.toFixed(1)}, ${wy.toFixed(1)})m  h=${isFinite(h) ? h.toFixed(2) + "m" : "未観測"}`;
});
$("hmap").addEventListener("mouseleave", () => {
  $("hmap-info").textContent = "8m四方 / 0.1m格子";
});

// ---------------- 探索マップ(占有格子) ----------------
// frame: [u8=3][f32 ox][f32 oy][f32 res][u16 W][u16 H][u8 cell...] cell: 0=未観測 1=FREE 2=OCCUPIED
const EXPMAP_HEAD = 17;

function onExpmap(buf) {
  if (!(buf instanceof ArrayBuffer) || buf.byteLength < EXPMAP_HEAD) return;
  const v = new DataView(buf);
  const ox = v.getFloat32(1, true), oy = v.getFloat32(5, true);
  const res = v.getFloat32(9, true);
  const w = v.getUint16(13, true), h = v.getUint16(15, true);
  if (!w || !h || w > 1024 || h > 1024 ||
      buf.byteLength !== EXPMAP_HEAD + w * h) {
    console.warn("探索マップframeのshape不一致で破棄", w, h, buf.byteLength);
    return;
  }
  expmap = { ox, oy, res, w, h,
             cells: new Uint8Array(buf.slice(EXPMAP_HEAD)) };
  drawExpmap();
}

// 未観測 / FREE / OCCUPIED (RGBA)
const EXP_RGBA = [[7, 16, 25, 255], [36, 96, 117, 255], [214, 69, 65, 255]];
let expOff = null;   // オフスクリーン canvas(1px=1cell)。縮小潰れ防止

function drawExpmap() {
  const cv = $("expmap");
  if (!cv || !expmap) return;
  // 表示サイズに内部解像度を合わせる(マップ最大化レイアウト対応)
  const wpx = cv.clientWidth, hpx = cv.clientHeight;
  if (wpx && hpx && (cv.width !== wpx || cv.height !== hpx)) {
    cv.width = wpx; cv.height = hpx;
  }
  const ctx = cv.getContext("2d");
  const { ox, oy, res, w, h, cells } = expmap;
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  ctx.fillStyle = "#071019";
  ctx.fillRect(0, 0, cv.width, cv.height);
  // 観測済み cell の外接範囲 + 余白だけを表示(20m四方全体だと細かすぎる)
  let minx = w, maxx = -1, miny = h, maxy = -1;
  for (let iy = 0; iy < h; iy++) {
    for (let ix = 0; ix < w; ix++) {
      if (cells[iy * w + ix] !== 0) {
        if (ix < minx) minx = ix;
        if (ix > maxx) maxx = ix;
        if (iy < miny) miny = iy;
        if (iy > maxy) maxy = iy;
      }
    }
  }
  if (maxx < 0) return;  // まだ観測なし
  const pad = 6;
  minx = Math.max(0, minx - pad); maxx = Math.min(w - 1, maxx + pad);
  miny = Math.max(0, miny - pad); maxy = Math.min(h - 1, maxy + pad);
  const vw = maxx - minx + 1, vh = maxy - miny + 1;
  const span = Math.max(vw, vh);
  // 長方形キャンバスでは短辺に合わせ、余白分は中央寄せ
  const S = Math.min(cv.width, cv.height);
  const cell = S / span;
  const padX = (cv.width - S) / 2, padY = (cv.height - S) / 2;

  // 1cell=1px の画像を作り、nearest-neighbor で拡大(セル潰れ・色の滲みを防ぐ)
  if (!expOff) expOff = document.createElement("canvas");
  if (expOff.width !== w || expOff.height !== h) {
    expOff.width = w; expOff.height = h;
  }
  const octx = expOff.getContext("2d");
  const img = octx.createImageData(w, h);
  for (let i = 0; i < w * h; i++) {
    const c = EXP_RGBA[cells[i]] || EXP_RGBA[2];
    img.data[i * 4] = c[0]; img.data[i * 4 + 1] = c[1];
    img.data[i * 4 + 2] = c[2]; img.data[i * 4 + 3] = c[3];
  }
  octx.putImageData(img, 0, 0);

  // 画面: 上=+x(前方), 左=+y(ハイトマップと同じ向き)。
  // 画像画素(ix,iy) → 画面(c1-cell*iy, c2-cell*ix) の軸入替+反転を行列で表現。
  const c1 = (span - vh) / 2 * cell + cell * (maxy + 1) + padX;
  const c2 = (span - vw) / 2 * cell + cell * (maxx + 1) + padY;
  ctx.imageSmoothingEnabled = false;
  ctx.setTransform(0, -cell, -cell, 0, c1, c2);
  ctx.drawImage(expOff, 0, 0);
  ctx.setTransform(1, 0, 0, 1, 0, 0);
  const toScr = (wx, wy) => [c1 - cell * ((wy - oy) / res),
                             c2 - cell * ((wx - ox) / res)];
  // 軌跡
  if (explore && explore.trace && explore.trace.length > 1) {
    ctx.strokeStyle = "#43efd066";
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    explore.trace.forEach(([tx, ty], k) => {
      const [sx, sy] = toScr(tx, ty);
      k === 0 ? ctx.moveTo(sx, sy) : ctx.lineTo(sx, sy);
    });
    ctx.stroke();
  }
  // 現在の goal
  if (explore && explore.goal) {
    const [gx, gy] = toScr(explore.goal[0], explore.goal[1]);
    ctx.strokeStyle = "#f7c948";
    ctx.lineWidth = 2;
    ctx.beginPath(); ctx.arc(gx, gy, 6, 0, Math.PI * 2); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(gx - 9, gy); ctx.lineTo(gx + 9, gy);
    ctx.moveTo(gx, gy - 9); ctx.lineTo(gx, gy + 9); ctx.stroke();
  }
  drawExpmapPose(ctx, toScr);
}

function drawExpmapPose(ctx, toScr) {
  if (!pose || !toScr) return;
  const [sx, sy] = toScr(pose[0], pose[1]);
  ctx.save();
  ctx.translate(sx, sy);
  ctx.rotate(-pose[3]);
  ctx.fillStyle = "#43efd0";
  ctx.beginPath();
  ctx.moveTo(0, -8); ctx.lineTo(5, 6); ctx.lineTo(-5, 6);
  ctx.closePath(); ctx.fill();
  ctx.restore();
}

let exploreLoggedStatus = "";
let exploreEvSeq = 0;   // 受信済みの探索イベント通番(意思決定ログの逐次表示)

function renderExplore(e) {
  explore = e || null;
  const st = $("explore-status"), panel = $("explore-proposal");
  if (!st) return;
  if (e && e.events) {
    for (const [seq, text] of e.events) {
      if (seq > exploreEvSeq) { exploreEvSeq = seq; log(`🗺 ${text}`); }
    }
  }
  if (!e || e.status === "idle") {
    st.textContent = "EXPLORE IDLE";
    st.className = "";
    panel.classList.add("hidden");
    return;
  }
  panel.classList.toggle("hidden", e.status !== "proposal");
  if (e.status === "proposal") $("explore-readback").textContent = e.detail || "";
  const tag = { proposal: "確認待ち", running: "探索中", done: "✔ 完了",
                stalled: "△ 打ち切り", stopped: "■ 停止", aborted: "■ 中断",
                refused: "✕ 拒否", error: "⚠ エラー" }[e.status] || e.status;
  const cnt = e.counts
    ? ` free=${e.counts.free} occ=${e.counts.occupied}` : "";
  st.textContent = `[${tag}] ${e.detail || ""}` +
    (e.status === "running" ? ` (${e.elapsed}s${cnt})` : "");
  st.className = e.status === "running" ? "running" : "";
  if (e.status !== exploreLoggedStatus) {
    exploreLoggedStatus = e.status;
    if (["done", "stalled", "stopped", "aborted", "refused", "error"]
        .includes(e.status)) {
      log(`🗺 探索: [${tag}] ${e.detail || ""}`,
          e.status === "done" ? "ok" : "err");
    }
  }
}

function sendExploreText() {
  const text = $("explore-input").value.trim();
  if (!text) { log("探索指示を入力してください", "err"); return; }
  send({ type: "explore", text });
}
$("btn-explore-go").onclick = sendExploreText;
$("explore-input").addEventListener("keydown", (e) => {
  if (e.key === "Enter" && !e.isComposing) { e.preventDefault(); sendExploreText(); }
});
$("btn-explore-confirm").onclick = () => {
  if (hintIfDisarmed()) return;
  send({ type: "explore_confirm" });
};
$("btn-explore-cancel").onclick = () => send({ type: "explore_cancel" });
// 🤖 自律モード: ワンクリックで探索提案(開始には復唱確認が必要=安全契約)
$("btn-explore-auto").onclick = () => {
  if (hintIfDisarmed()) return;
  send({ type: "explore_auto" });
};
// ⛔ 緊急停止: stopAll = cmd 0 + action:stop(ARM不問・サーバ側で全自律系中断)
$("btn-estop").onclick = () => {
  stopAll();
  log("⛔ 緊急停止(全系停止・ソフトウェア停止)", "err");
};

// ---------------- 操縦 ----------------
const keys = { w: false, a: false, s: false, d: false, q: false, e: false };
const keyboardKeys = new Set();
const pointerKeys = new Map();   // pointerId -> Set<key>（マルチタッチ/斜め入力用）
let lastNonzero = false;
let voiceCmd = null;   // {vx,vy,wz(単位), until(ms), say} — キー入力があれば無効
let disarmHintAt = 0;

function syncKeys() {
  for (const k of Object.keys(keys)) keys[k] = keyboardKeys.has(k);
  for (const held of pointerKeys.values()) {
    for (const k of held) keys[k] = true;
  }
}

function clearManualInput() {
  keyboardKeys.clear();
  pointerKeys.clear();
  syncKeys();
}

function buttonKeys(btn) {
  const raw = btn.dataset.keys || btn.dataset.k || "";
  return raw.split(",")
    .map((k) => k.trim().toLowerCase())
    .filter((k) => Object.prototype.hasOwnProperty.call(keys, k));
}

function scaleVal() { return parseFloat($("scale").value); }
$("scale").oninput = () => { $("scale-val").textContent = scaleVal().toFixed(1); };

function composeCmd() {
  const sc = scaleVal();
  const c = {
    vx: (keys.w ? sc : 0) + (keys.s ? -0.6 * sc : 0),
    vy: (keys.a ? 0.5 * sc : 0) + (keys.d ? -0.5 * sc : 0),
    wz: (keys.q ? 1.2 * sc : 0) + (keys.e ? -1.2 * sc : 0),
  };
  // 前進+平行移動でも速度スケールを超えないよう平面速度を正規化する。
  const planar = Math.hypot(c.vx, c.vy);
  if (planar > sc && planar > 0) {
    c.vx *= sc / planar;
    c.vy *= sc / planar;
  }
  if (c.vx || c.vy || c.wz) { voiceCmd = null; return c; }  // 手動操作が音声より優先
  if (voiceCmd) {
    const left = (voiceCmd.until - Date.now()) / 1000;
    if (left <= 0) {
      voiceCmd = null;
      $("voice-text").textContent = "";
    } else {
      $("voice-text").textContent = `♪ ${voiceCmd.say} 残り${left.toFixed(1)}s (Spaceで停止)`;
      return { vx: voiceCmd.vx * sc, vy: voiceCmd.vy * sc, wz: voiceCmd.wz * sc };
    }
  }
  return c;
}

setInterval(() => {
  if (!ws || ws.readyState !== WebSocket.OPEN) return;
  const c = composeCmd();
  const nz = c.vx !== 0 || c.vy !== 0 || c.wz !== 0;
  if (armed && nz) {
    send({ type: "cmd", ...c });
    lastNonzero = true;
  } else if (lastNonzero) {
    send({ type: "cmd", vx: 0, vy: 0, wz: 0 });
    lastNonzero = false;
  }
  // パッドのハイライト（斜めボタンは構成キーがすべて有効な時だけ点灯）
  document.querySelectorAll(".pad .key").forEach((btn) => {
    const held = buttonKeys(btn);
    const active = held.length > 0 && held.every((k) => keys[k]);
    btn.classList.toggle("active", active);
    btn.setAttribute("aria-pressed", String(active));
  });
}, 100);

function stopAll() {
  clearManualInput();
  voiceCmd = null;
  $("voice-text").textContent = "";
  send({ type: "mission_stop" });
  send({ type: "stair_stop" });
  send({ type: "rl_stop" });
  send({ type: "cmd", vx: 0, vy: 0, wz: 0 });
  send({ type: "action", name: "stop" });
}

function hintIfDisarmed() {
  if (armed) return false;
  if (Date.now() - disarmHintAt > 2500) {
    disarmHintAt = Date.now();
    log("DISARM中です — 右上のARMスイッチをONにしてください", "err");
  }
  return true;
}

// e.code(物理キー)で判定: 日本語IMEがONでも W/S 等が効くようにする
const CODE2KEY = { KeyW: "w", KeyA: "a", KeyS: "s", KeyD: "d", KeyQ: "q", KeyE: "e" };
const typing = () => ["INPUT", "TEXTAREA"].includes(document.activeElement?.tagName);
window.addEventListener("keydown", (e) => {
  if (typing()) return;              // 任務入力欄などタイプ中はテレオペ無効
  if (e.repeat) return;
  if (e.code === "Space") { e.preventDefault(); stopAll(); return; }
  const k = CODE2KEY[e.code];
  if (k) { keyboardKeys.add(k); syncKeys(); hintIfDisarmed(); }
});
window.addEventListener("keyup", (e) => {
  const k = CODE2KEY[e.code];
  if (k) { keyboardKeys.delete(k); syncKeys(); }
});
window.addEventListener("blur", () => {   // タブ切替中の押しっぱなし防止
  clearManualInput();
});

document.querySelectorAll(".pad .key").forEach((btn) => {
  const raw = btn.dataset.keys || btn.dataset.k || "";
  if (raw.split(",").some((k) => k.trim().toLowerCase() === "space")) {
    btn.onclick = stopAll;
    return;
  }
  const held = buttonKeys(btn);
  if (!held.length) return;
  btn.classList.add("needs-arm");
  btn.addEventListener("pointerdown", (e) => {
    e.preventDefault();
    pointerKeys.set(e.pointerId, new Set(held));
    syncKeys();
  });
  ["pointerup", "pointerleave", "pointercancel"].forEach((ev) =>
    btn.addEventListener(ev, (e) => {
      pointerKeys.delete(e.pointerId);
      syncKeys();
    }));
});

// 要素外で離された場合も必ず解除する（要素側で解除済みならno-op）。
["pointerup", "pointercancel"].forEach((ev) =>
  window.addEventListener(ev, (e) => {
    if (pointerKeys.delete(e.pointerId)) syncKeys();
  }));

// ---------------- 音声操縦 ----------------
let mediaStream = null;
let recorder = null;
let recChunks = [];

async function micDown(e) {
  e.preventDefault();
  if (recorder && recorder.state === "recording") return;
  try {
    if (!mediaStream) {
      mediaStream = await navigator.mediaDevices.getUserMedia({ audio: true });
    }
  } catch (err) {
    log("マイク使用不可: " + err.message + "(http://localhost で開いていますか?)", "err");
    return;
  }
  recChunks = [];
  recorder = new MediaRecorder(mediaStream, { mimeType: "audio/webm" });
  recorder.ondataavailable = (ev) => { if (ev.data.size) recChunks.push(ev.data); };
  recorder.onstop = sendVoice;
  recorder.start();
  $("btn-mic").classList.add("recording");
  $("voice-text").textContent = "● 録音中… 離すと送信";
}

function micUp() {
  if (recorder && recorder.state === "recording") recorder.stop();
  $("btn-mic").classList.remove("recording");
}

async function sendVoice() {
  const blob = new Blob(recChunks, { type: "audio/webm" });
  if (blob.size < 1000) { $("voice-text").textContent = ""; return; }
  $("voice-text").textContent = "… 認識中";
  let d;
  try {
    const r = await fetch("/voice", { method: "POST", body: blob });
    d = await r.json();
    if (!r.ok) throw new Error(d.error || r.status);
  } catch (err) {
    $("voice-text").textContent = "";
    log("音声認識エラー: " + err.message, "err");
    return;
  }
  const it = d.intent;
  log(`♪「${d.text}」→ ${it.say}`, it.action === "none" ? "err" : "ok");
  $("voice-text").textContent = `「${d.text}」→ ${it.say}`;
  if (d.goal) {
    // 契約パーサが処理済み(提案/確認/停止/取消)。提案はEXPLORE MAPパネルに出る。
    if (it.action === "stop") stopAll();
    return;
  }
  if (it.action === "none" && d.text.length >= 6) {
    // 単純コマンドでない発話はAI任務の入力欄へ(実行はユーザーが▶で)
    $("mission-input").value = d.text;
    log("AI任務として実行するには ▶ AI実行 を押してください");
    return;
  }
  execIntent(it);
}

function execIntent(it) {
  switch (it.action) {
    case "stop":
      stopAll();
      break;
    case "stand_up": case "stand_down": case "balance_stand":
      if (hintIfDisarmed()) return;
      send({ type: "action", name: it.action });
      break;
    case "move": case "turn": case "strafe":
      if (hintIfDisarmed()) return;
      voiceCmd = { vx: it.vx || 0, vy: it.vy || 0, wz: it.wz || 0,
                   until: Date.now() + it.dur * 1000, say: it.say };
      break;
  }
}

const micBtn = $("btn-mic");
micBtn.addEventListener("pointerdown", micDown);
["pointerup", "pointerleave", "pointercancel"].forEach((ev) =>
  micBtn.addEventListener(ev, micUp));

// ---------------- AI任務 (VLA: 言語+カメラ+LiDAR → 行動) ----------------
let missionLoggedStep = 0;

function startMission() {
  const text = $("mission-input").value.trim();
  if (!text) { log("任務の指示を入力してください", "err"); return; }
  if (hintIfDisarmed()) return;
  missionLoggedStep = 0;
  send({ type: "mission", instruction: text });
  log("AI任務を開始: 「" + text + "」");
}
$("btn-mission-go").onclick = startMission;
$("btn-mission-go").classList.add("needs-arm");
$("btn-mission-stop").onclick = () => send({ type: "mission_stop" });
$("mission-input").addEventListener("keydown", (e) => {
  if (e.key === "Enter" && !e.isComposing) { e.preventDefault(); startMission(); }
});

function renderMission(m) {
  const el = $("mission-status");
  if (!m || m.status === "idle") { el.textContent = ""; el.className = ""; return; }
  const tag = { running: "実行中", done: "✔ 完了", aborted: "■ 中断", error: "⚠ エラー" }[m.status] || m.status;
  el.textContent = `[${tag}] ${m.detail || ""}` +
    (m.status === "running" ? ` (step${m.step}, ${m.elapsed}s)` : "");
  el.className = m.status === "running" ? "running" : "";
  // 新しい判断をログへ
  if (m.last && m.step > missionLoggedStep && m.last.action) {
    missionLoggedStep = m.step;
    log(`🤖 step${m.step}: ${m.last.action} vx=${m.last.vx} wz=${m.last.wz}` +
        ` — ${m.last.reason} (${m.last.latency}s)`, "ok");
  }
}

// ---------------- 段差検出・登坂タスク ----------------
let stairLoggedState = "";

const KIND_LABEL = { none: "なし", step: "段差", stairs: "階段", wall: "壁(登坂不可)", drop: "落差!" };

function updateStairMetrics(s) {
  const metric = (id, value, suffix = "") => {
    $(id).textContent = Number.isFinite(value) ? value.toFixed(2) + suffix : "--";
  };
  metric("stair-height", s?.height, " m");
  metric("stair-distance", s?.distance, " m");
  const yaw = Number.isFinite(s?.yaw_err) ? `${s.yaw_err >= 0 ? "+" : ""}${s.yaw_err.toFixed(2)}` : "--";
  $("stair-yaw").textContent = yaw;
  metric("stair-width", s?.width, " m");

  const kind = s?.kind || "none";
  const hazard = $("stair-hazard");
  hazard.textContent = kind === "drop" ? "DROP" : kind === "wall" ? "WALL" :
    (kind === "step" || kind === "stairs") ? "CLEAR" : s ? "NO TARGET" : "SCANNING";
  hazard.parentElement.classList.toggle("warning", kind === "drop" || kind === "wall");
}

function renderStair(s, task) {
  const el = $("stair-detect");
  updateStairMetrics(s);
  const k = s?.kind || "none";
  el.className = s ? k : "";
  if (!s) {
    el.textContent = "段差: --";
  } else if (k === "step" || k === "stairs") {
    el.textContent = `${KIND_LABEL[k]}: 高さ${s.height.toFixed(2)}m 距離${s.distance.toFixed(2)}m ` +
      `yaw${s.yaw_err >= 0 ? "+" : ""}${s.yaw_err.toFixed(2)} 幅${s.width.toFixed(2)}m`;
  } else if (k === "wall" || k === "drop") {
    el.textContent = `⚠ ${KIND_LABEL[k]} ${s.reason || ""}`;
  } else {
    el.textContent = "段差: " + (s.reason || "なし");
  }
  $("btn-stair").disabled = !s || !(k === "step" || k === "stairs");

  const ts = $("stair-status");
  if (!task || task.state === "idle") { ts.textContent = ""; ts.className = ""; return; }
  const running = !["done", "aborted", "error", "refused", "idle"].includes(task.state);
  ts.textContent = `[${task.state}] ${task.detail || ""}` +
    (task.climbed ? ` — ${task.climbed}段登坂` : "") +
    (running ? ` (${task.elapsed}s)` : "");
  ts.className = running ? "running" : (task.state === "refused" ? "refused" : "");
  if (task.state !== stairLoggedState) {
    stairLoggedState = task.state;
    if (task.state === "refused") log("🪜 " + task.detail, "err");
    else if (task.state === "done") log("🪜 " + task.detail, "ok");
    else if (task.state === "aborted") log("🪜 " + task.detail, "err");
    else log("🪜 " + task.state + ": " + (task.detail || ""));
  }
}

// ハイトマップに検出エッジを描く(drawHeightmapから呼ばれる)
function drawStairOverlay(ctx, toScr) {
  if (!stair || !pose) return;
  const k = stair.kind;
  if (!["step", "stairs", "wall", "drop"].includes(k)) return;
  const yaw = pose[3], d = stair.distance;
  const half = Math.max(0.25, (stair.width || 0.5) / 2);
  // body系のエッジ線 (x=d, y=±half) を回転させて world → 画面へ
  const p = (by) => {
    const bx = d + (stair.yaw_err ? -by * Math.tan(stair.yaw_err) : 0);
    return toScr(pose[0] + bx * Math.cos(yaw) - by * Math.sin(yaw),
                 pose[1] + bx * Math.sin(yaw) + by * Math.cos(yaw));
  };
  const [x1, y1] = p(half), [x2, y2] = p(-half);
  ctx.strokeStyle = k === "drop" || k === "wall" ? "#ff4338" : "#59ef83";
  ctx.lineWidth = 3;
  ctx.beginPath(); ctx.moveTo(x1, y1); ctx.lineTo(x2, y2); ctx.stroke();
  ctx.fillStyle = ctx.strokeStyle;
  ctx.font = "11px monospace";
  const label = k === "drop" ? "落差" : k === "wall" ? "壁" :
    `${stair.height.toFixed(2)}m`;
  ctx.fillText(label, (x1 + x2) / 2 + 6, (y1 + y2) / 2 - 5);
}

$("btn-stair").onclick = () => {
  if (hintIfDisarmed()) return;
  send({ type: "stair_start", backend: "sport",
         confirm: $("stair-confirm").checked,
         multi: $("stair-multi").checked });
  log("🪜 登坂タスク開始(純正)");
};
$("btn-stair").classList.add("needs-arm");
$("btn-stair-stop").onclick = () => send({ type: "stair_stop" });

// ---- 学習方策(M3) ----
let rlLoggedState = "";

function renderRl(r) {
  const el = $("rl-status");
  const live = r && r.running && !r.dry_run;
  $("btn-rl-climb").classList.toggle("live", !$("rl-dry").checked);
  if (!r || r.state === "idle") { el.textContent = ""; el.className = ""; return; }
  el.className = live ? "live" : "";
  const tag = { starting: "起動中", ramping: "立位ランプ", running: "方策実行中",
                stopping: "停止中", stopped: "停止", error: "エラー", preflight: "点検" }[r.state] || r.state;
  let txt = `[${tag}${r.dry_run ? " dry-run" : " ●LIVE"}] ${r.detail || ""}`;
  if (r.running) txt += ` (${r.elapsed}s)`;
  if (r.log && r.log.length) txt += "\n" + r.log.slice(-3).join("\n");
  el.textContent = txt;
  if (r.state !== rlLoggedState) {
    rlLoggedState = r.state;
    const cls = r.state === "error" ? "err" : r.state === "running" ? "ok" : "";
    log("🧠 " + tag + ": " + (r.detail || ""), cls);
  }
}

$("btn-rl-climb").onclick = () => {
  if (hintIfDisarmed()) return;
  const dry = $("rl-dry").checked;
  const policy = $("rl-policy").value;
  if (!dry) {
    const ok = confirm(
      "⚠ 実弾(LIVE)で学習方策を実行します。\n\n" +
      "・sportモードを解除し、モータを50Hzで直接制御します\n" +
      "・ロボットは吊り下げ、または周囲2mをクリアにしてください\n" +
      "・物理的な緊急停止を手元に用意してください\n\n" +
      "policy=" + policy + " で登坂を開始しますか?");
    if (!ok) { log("🧠 LIVE実行をキャンセルしました"); return; }
  }
  send({ type: "stair_start", backend: "rl", confirm: $("stair-confirm").checked,
         dry_run: dry, policy: policy });
  log("🧠 学習方策で登坂開始 " + (dry ? "(dry-run)" : "(● LIVE)"), dry ? "" : "err");
};
$("btn-rl-climb").classList.add("needs-arm");
$("btn-rl-stop").onclick = () => { send({ type: "stair_stop" }); send({ type: "rl_stop" }); };
$("btn-rl-restore").onclick = () => { send({ type: "rl_restore" }); log("🧠 Sportモードへ復帰要求"); };

// 音声モデルの準備状況を表示
fetch("/voice/status").then((r) => r.json()).then((d) => {
  if (d.error) log("音声: " + d.error, "err");
  else if (!d.ready) {
    log("音声モデルをロード中…(数十秒)");
    const t = setInterval(() => fetch("/voice/status").then((r) => r.json()).then((s) => {
      if (s.ready) { clearInterval(t); log("音声認識 準備完了 — マイクボタンを押しながら話す", "ok"); }
    }), 3000);
  } else log("音声認識 準備完了 — マイクボタンを押しながら話す", "ok");
}).catch(() => {});

$("arm-toggle").onchange = (e) => send({ type: "arm", on: e.target.checked });
$("btn-stop").onclick = stopAll;
$("btn-damp").onclick = () => {
  clearManualInput();
  voiceCmd = null;
  lastNonzero = false;
  $("voice-text").textContent = "";
  send({ type: "cmd", vx: 0, vy: 0, wz: 0 });
  send({ type: "arm", on: false });
  send({ type: "action", name: "damp" });
  log("DAMP送信 — 入力を解除しDISARMしました", "err");
};
$("btn-standup").onclick = () => send({ type: "action", name: "stand_up" });
$("btn-standdown").onclick = () => send({ type: "action", name: "stand_down" });
$("btn-balance").onclick = () => send({ type: "action", name: "balance_stand" });
["btn-standup", "btn-standdown", "btn-balance"].forEach((id) => $(id).classList.add("needs-arm"));

// ---------------- start ----------------
setArmedUI(false);
try { initLidar3D(); } catch (e) { console.error("lidar3d init失敗:", e); }
connect();
log("コックピット起動 — ARMするまで移動コマンドは送信されません");

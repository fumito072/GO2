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
function updateZRange(values) {
  if (!values.length) return;
  const s = Array.from(values).sort((a, b) => a - b);
  const lo = s[Math.floor(s.length * 0.05)], hi = s[Math.floor(s.length * 0.95)];
  const pad = Math.max(0.15, (hi - lo) * 0.1);
  zRange.lo += 0.1 * ((lo - pad) - zRange.lo);
  zRange.hi += 0.1 * ((hi + pad) - zRange.hi);
}
const zNorm = (z) => (z - zRange.lo) / Math.max(0.05, zRange.hi - zRange.lo);

// ---------------- state ----------------
let ws = null;
let telem = {};
let pose = null;          // [x,y,z,yaw]
let armed = false;
let hmap = null;          // {cx, cy, res, n, data(Float32Array)}
let stair = null;         // 段差検出結果

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
      const view = new DataView(ev.data);
      const kind = view.getUint8(0);
      if (kind === 1) onLidar(ev.data);
      else if (kind === 2) onHeightmap(ev.data);
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
  ctx.fillStyle = "#1d3a5f";
  ctx.fillRect(-W, -H * 2 + py, W * 2, H * 2);   // 空
  ctx.fillStyle = "#4a3524";
  ctx.fillRect(-W, py, W * 2, H * 2);            // 地面
  ctx.strokeStyle = "#d7e1ec";
  ctx.lineWidth = 1.5;
  ctx.beginPath(); ctx.moveTo(-W, py); ctx.lineTo(W, py); ctx.stroke();
  ctx.restore();
  // 機体シンボル
  ctx.strokeStyle = "#3ddc97"; ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.moveTo(W / 2 - 22, H / 2); ctx.lineTo(W / 2 - 7, H / 2);
  ctx.moveTo(W / 2 + 7, H / 2); ctx.lineTo(W / 2 + 22, H / 2);
  ctx.arc(W / 2, H / 2, 3, 0, 7);
  ctx.stroke();
}

// ---------------- LiDAR 3D (three.js, z-up world座標そのまま) ----------------
const MAX_PTS = 8000;
let scene, camera3, renderer, points, robotGroup;
const orbit = { az: -2.4, el: 0.55, r: 6.0, target: new THREE.Vector3(0, 0, 0) };

function initLidar3D() {
  const box = $("lidar3d");
  try {
    renderer = new THREE.WebGLRenderer({ antialias: true });
  } catch (e) {
    box.innerHTML = '<div style="padding:20px;color:#7d8ea3">WebGLが利用できないため3D表示は無効です。<br>ハイトマップ/カメラ/操縦は使用できます。</div>';
    return;
  }
  scene = new THREE.Scene();
  scene.background = new THREE.Color(0x0a0e13);
  camera3 = new THREE.PerspectiveCamera(55, 1, 0.1, 200);
  camera3.up.set(0, 0, 1);
  box.appendChild(renderer.domElement);

  const grid = new THREE.GridHelper(20, 20, 0x2a3a4d, 0x18222e);
  grid.rotation.x = Math.PI / 2;  // XY平面(z-up)に
  scene.add(grid);
  scene.add(new THREE.AxesHelper(0.8));

  const geo = new THREE.BufferGeometry();
  geo.setAttribute("position", new THREE.BufferAttribute(new Float32Array(MAX_PTS * 3), 3));
  geo.setAttribute("color", new THREE.BufferAttribute(new Float32Array(MAX_PTS * 3), 3));
  geo.setDrawRange(0, 0);
  points = new THREE.Points(geo, new THREE.PointsMaterial({ size: 0.035, vertexColors: true }));
  points.frustumCulled = false;
  scene.add(points);

  // ロボットマーカー
  robotGroup = new THREE.Group();
  const body = new THREE.Mesh(new THREE.BoxGeometry(0.65, 0.31, 0.22),
    new THREE.MeshBasicMaterial({ color: 0x3ddc97, wireframe: true }));
  robotGroup.add(body);
  const nose = new THREE.Mesh(new THREE.ConeGeometry(0.09, 0.28, 12),
    new THREE.MeshBasicMaterial({ color: 0xffb020 }));
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
    if (w && h) { renderer.setSize(w, h); camera3.aspect = w / h; camera3.updateProjectionMatrix(); }
  };
  new ResizeObserver(resize).observe(box);
  resize();

  (function animate() {
    requestAnimationFrame(animate);
    const t = orbit.target;
    camera3.position.set(
      t.x + orbit.r * Math.cos(orbit.el) * Math.cos(orbit.az),
      t.y + orbit.r * Math.cos(orbit.el) * Math.sin(orbit.az),
      t.z + orbit.r * Math.sin(orbit.el));
    camera3.lookAt(t);
    renderer.render(scene, camera3);
  })();
}

function onLidar(buf) {
  if (!points) return;  // WebGL無効時
  const n = Math.min(new DataView(buf).getUint32(1, true), MAX_PTS);
  const xyz = new Float32Array(buf.slice(5, 5 + n * 12));
  const pos = points.geometry.attributes.position.array;
  const col = points.geometry.attributes.color.array;
  const zs = [];
  for (let i = 0; i < n; i++) zs.push(xyz[i * 3 + 2]);
  updateZRange(zs);
  for (let i = 0; i < n; i++) {
    pos[i * 3] = xyz[i * 3];
    pos[i * 3 + 1] = xyz[i * 3 + 1];
    pos[i * 3 + 2] = xyz[i * 3 + 2];
    const c = cmap(zNorm(xyz[i * 3 + 2]));
    col[i * 3] = c[0] / 255; col[i * 3 + 1] = c[1] / 255; col[i * 3 + 2] = c[2] / 255;
  }
  points.geometry.setDrawRange(0, n);
  points.geometry.attributes.position.needsUpdate = true;
  points.geometry.attributes.color.needsUpdate = true;
}

function updateRobotMarker() {
  if (!pose || !robotGroup) return;
  robotGroup.position.set(pose[0], pose[1], pose[2]);
  robotGroup.rotation.z = pose[3];
  // カメラ追従(ゆっくり)
  orbit.target.lerp(new THREE.Vector3(pose[0], pose[1], pose[2]), 0.06);
}

// ---------------- ハイトマップ ----------------
function onHeightmap(buf) {
  const v = new DataView(buf);
  const cx = v.getFloat32(1, true), cy = v.getFloat32(5, true);
  const res = v.getFloat32(9, true), n = v.getUint16(13, true);
  hmap = { cx, cy, res, n, data: new Float32Array(buf.slice(15, 15 + n * n * 4)) };
  drawHeightmap();
}

function drawHeightmap() {
  if (!hmap) return;
  const cv = $("hmap"), ctx = cv.getContext("2d");
  const { n, data, cx, cy, res } = hmap;
  const cell = cv.width / n;
  ctx.fillStyle = "#10151c";
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
    ctx.strokeStyle = "#d7e1ec88";
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
    ctx.fillStyle = "#3ddc97";
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
  ctx.fillStyle = "#7d8ea3";
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

// ---------------- 操縦 ----------------
const keys = { w: false, a: false, s: false, d: false, q: false, e: false };
let lastNonzero = false;
let voiceCmd = null;   // {vx,vy,wz(単位), until(ms), say} — キー入力があれば無効
let disarmHintAt = 0;

function scaleVal() { return parseFloat($("scale").value); }
$("scale").oninput = () => { $("scale-val").textContent = scaleVal().toFixed(1); };

function composeCmd() {
  const sc = scaleVal();
  const c = {
    vx: (keys.w ? sc : 0) + (keys.s ? -0.6 * sc : 0),
    vy: (keys.a ? 0.5 * sc : 0) + (keys.d ? -0.5 * sc : 0),
    wz: (keys.q ? 1.2 * sc : 0) + (keys.e ? -1.2 * sc : 0),
  };
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
  // パッドのハイライト
  for (const k of Object.keys(keys)) {
    document.querySelector(`.key[data-k="${k}"]`).classList.toggle("active", keys[k]);
  }
}, 100);

function stopAll() {
  for (const k of Object.keys(keys)) keys[k] = false;
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
  if (k) { keys[k] = true; hintIfDisarmed(); }
});
window.addEventListener("keyup", (e) => {
  const k = CODE2KEY[e.code];
  if (k) keys[k] = false;
});
window.addEventListener("blur", () => {   // タブ切替中の押しっぱなし防止
  for (const k of Object.keys(keys)) keys[k] = false;
});

document.querySelectorAll(".pad .key").forEach((btn) => {
  const k = btn.dataset.k;
  if (k === "space") { btn.onclick = stopAll; return; }
  btn.classList.add("needs-arm");
  btn.addEventListener("pointerdown", (e) => { e.preventDefault(); keys[k] = true; });
  ["pointerup", "pointerleave", "pointercancel"].forEach((ev) =>
    btn.addEventListener(ev, () => { keys[k] = false; }));
});

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

function renderStair(s, task) {
  const el = $("stair-detect");
  if (!s) { el.textContent = "段差: --"; el.className = ""; return; }
  const k = s.kind || "none";
  el.className = k;
  if (k === "step" || k === "stairs") {
    el.textContent = `${KIND_LABEL[k]}: 高さ${s.height.toFixed(2)}m 距離${s.distance.toFixed(2)}m ` +
      `yaw${s.yaw_err >= 0 ? "+" : ""}${s.yaw_err.toFixed(2)} 幅${s.width.toFixed(2)}m`;
  } else if (k === "wall" || k === "drop") {
    el.textContent = `⚠ ${KIND_LABEL[k]} ${s.reason || ""}`;
  } else {
    el.textContent = "段差: " + (s.reason || "なし");
  }
  $("btn-stair").disabled = !(k === "step" || k === "stairs");

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
  ctx.strokeStyle = k === "drop" || k === "wall" ? "#ff5d5d" : "#3ddc97";
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
  send({ type: "cmd", vx: 0, vy: 0, wz: 0 });
  send({ type: "action", name: "damp" });
  log("DAMP送信(脱力)", "err");
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

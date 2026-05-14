/**
 * Wavemap Viewer — 3D point cloud visualization over ROS 2 rosbridge
 *
 * Subscribes to PointCloud2 topics published by wavemap and renders
 * them in real-time using Three.js.  Supports save/load via wavemap
 * ROS 2 services.
 */

/* ───────── State ───────── */
let ros = null;
let pointcloudSub = null;
let scene, camera, renderer, controls;
let pointCloud = null;
let gridHelper = null;
let axesHelper = null;

let totalPoints = 0;
let frameCount = 0;
let lastFpsTime = performance.now();

const BG_COLORS = {
  dark:     0x0d0f14,
  midnight: 0x0a0e1a,
  light:    0xdfe3ec,
};

/* ───────── DOM refs ───────── */
const $url        = document.getElementById('ros-url');
const $btnConnect = document.getElementById('btn-connect');
const $btnSub     = document.getElementById('btn-subscribe');
const $btnSave    = document.getElementById('btn-save');
const $btnLoad    = document.getElementById('btn-load');
const $btnClear   = document.getElementById('btn-clear');
const $btnResetCam = document.getElementById('btn-reset-cam');
const $btnTop     = document.getElementById('btn-top');
const $btnFront   = document.getElementById('btn-front');
const $status     = document.getElementById('connection-status');
const $statusText = document.querySelector('.status-text');
const $statPoints = document.getElementById('stat-points');
const $statFps    = document.getElementById('stat-fps');
const $pointSize  = document.getElementById('point-size');
const $pointSizeVal = document.getElementById('point-size-val');
const $colorMode  = document.getElementById('color-mode');
const $bgColor    = document.getElementById('bg-color');
const $showGrid   = document.getElementById('show-grid');
const $showAxes   = document.getElementById('show-axes');
const $savePath   = document.getElementById('save-path');
const $loadPath   = document.getElementById('load-path');
const $ioStatus   = document.getElementById('io-status');
const $pclTopic   = document.getElementById('pointcloud-topic');
const $canvas     = document.getElementById('canvas3d');

/* ───────── Toast ───────── */
function toast(msg, type = 'info', duration = 3500) {
  const container = document.getElementById('toast-container');
  const el = document.createElement('div');
  el.className = `toast ${type}`;
  el.textContent = msg;
  container.appendChild(el);
  setTimeout(() => {
    el.style.animation = 'fadeOut 250ms ease forwards';
    setTimeout(() => el.remove(), 250);
  }, duration);
}

/* ───────── Three.js ───────── */
function initThree() {
  scene = new THREE.Scene();
  scene.background = new THREE.Color(BG_COLORS.dark);
  scene.fog = new THREE.FogExp2(BG_COLORS.dark, 0.015);

  camera = new THREE.PerspectiveCamera(
    60,
    window.innerWidth / window.innerHeight,
    0.1,
    500
  );
  camera.position.set(8, 6, 8);
  camera.lookAt(0, 0, 0);

  renderer = new THREE.WebGLRenderer({
    canvas: $canvas,
    antialias: true,
    alpha: false,
  });
  renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
  updateRendererSize();

  controls = new THREE.OrbitControls(camera, renderer.domElement);
  controls.enableDamping = true;
  controls.dampingFactor = 0.08;
  controls.zoomSpeed = 1.2;
  controls.panSpeed = 0.8;
  controls.target.set(0, 0, 0);

  // Grid
  gridHelper = new THREE.GridHelper(40, 40, 0x2a2e3d, 0x1e2230);
  scene.add(gridHelper);

  // Axes
  axesHelper = new THREE.AxesHelper(3);
  scene.add(axesHelper);

  // Ambient light for reference
  scene.add(new THREE.AmbientLight(0xffffff, 0.3));

  // Empty point cloud
  createEmptyPointCloud();

  window.addEventListener('resize', onResize);
  animate();
}

function updateRendererSize() {
  const viewport = document.getElementById('viewport');
  const w = viewport.clientWidth;
  const h = viewport.clientHeight;
  renderer.setSize(w, h);
  camera.aspect = w / h;
  camera.updateProjectionMatrix();
}

function onResize() {
  updateRendererSize();
}

function createEmptyPointCloud() {
  if (pointCloud) {
    scene.remove(pointCloud);
    pointCloud.geometry.dispose();
    pointCloud.material.dispose();
  }

  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute([], 3));
  geo.setAttribute('color', new THREE.Float32BufferAttribute([], 3));

  const mat = new THREE.PointsMaterial({
    size: parseFloat($pointSize.value),
    vertexColors: true,
    sizeAttenuation: true,
    transparent: true,
    opacity: 0.9,
  });

  pointCloud = new THREE.Points(geo, mat);
  scene.add(pointCloud);
  totalPoints = 0;
  $statPoints.textContent = '0';
}

function animate() {
  requestAnimationFrame(animate);
  controls.update();
  renderer.render(scene, camera);

  frameCount++;
  const now = performance.now();
  if (now - lastFpsTime >= 1000) {
    $statFps.textContent = frameCount;
    frameCount = 0;
    lastFpsTime = now;
  }
}

/* ───────── PointCloud2 Parser ───────── */
function parsePointCloud2(msg) {
  const { height, width, fields, point_step, row_step, data, is_bigendian } = msg;
  const numPoints = height * width;

  if (numPoints === 0) return { positions: [], intensities: [] };

  // Decode base64
  const raw = atob(data);
  const bytes = new Uint8Array(raw.length);
  for (let i = 0; i < raw.length; i++) {
    bytes[i] = raw.charCodeAt(i);
  }
  const view = new DataView(bytes.buffer);
  const littleEndian = !is_bigendian;

  // Find field offsets
  let xOff = -1, yOff = -1, zOff = -1, iOff = -1;
  for (const f of fields) {
    switch (f.name) {
      case 'x': xOff = f.offset; break;
      case 'y': yOff = f.offset; break;
      case 'z': zOff = f.offset; break;
      case 'intensity': iOff = f.offset; break;
    }
  }

  if (xOff < 0 || yOff < 0 || zOff < 0) {
    console.warn('PointCloud2 missing x/y/z fields');
    return { positions: [], intensities: [] };
  }

  const positions = new Float32Array(numPoints * 3);
  const intensities = new Float32Array(numPoints);

  for (let i = 0; i < numPoints; i++) {
    const base = i * point_step;
    const x = view.getFloat32(base + xOff, littleEndian);
    const y = view.getFloat32(base + yOff, littleEndian);
    const z = view.getFloat32(base + zOff, littleEndian);

    // Skip NaN / inf
    if (!isFinite(x) || !isFinite(y) || !isFinite(z)) {
      positions[i * 3] = 0;
      positions[i * 3 + 1] = 0;
      positions[i * 3 + 2] = 0;
      intensities[i] = 0;
      continue;
    }

    // ROS: x-forward, y-left, z-up → Three.js: x-right, y-up, z-forward
    positions[i * 3]     = x;
    positions[i * 3 + 1] = z;  // height
    positions[i * 3 + 2] = -y;

    intensities[i] = iOff >= 0
      ? view.getFloat32(base + iOff, littleEndian)
      : z;
  }

  return { positions, intensities };
}

/* ───────── Color Mapping ───────── */
function heightColor(h) {
  // Gradient: blue → cyan → green → yellow → red
  const t = Math.max(0, Math.min(1, (h + 1) / 6)); // map ~[-1, 5] → [0, 1]
  const r = Math.min(1, Math.max(0, 1.5 - Math.abs(t - 0.75) * 4));
  const g = Math.min(1, Math.max(0, 1.5 - Math.abs(t - 0.5) * 4));
  const b = Math.min(1, Math.max(0, 1.5 - Math.abs(t - 0.25) * 4));
  return [r, g, b];
}

function occupancyColor(val) {
  // occupied = red, free = green, unknown = grey
  const t = Math.max(0, Math.min(1, (val + 2) / 4));
  return [t, 1 - t, 0.15];
}

function solidColor() {
  return [0.31, 0.43, 0.97]; // accent color
}

function applyColors(positions, intensities) {
  const mode = $colorMode.value;
  const n = positions.length / 3;
  const colors = new Float32Array(n * 3);

  for (let i = 0; i < n; i++) {
    const h = positions[i * 3 + 1]; // y in Three.js = height
    const intensity = intensities[i];
    let c;

    switch (mode) {
      case 'height':
        c = heightColor(h);
        break;
      case 'occupancy':
        c = occupancyColor(intensity);
        break;
      case 'solid':
      default:
        c = solidColor();
        break;
    }

    colors[i * 3]     = c[0];
    colors[i * 3 + 1] = c[1];
    colors[i * 3 + 2] = c[2];
  }

  return colors;
}

/* ───────── Update Cloud ───────── */
function updatePointCloud(msg) {
  const { positions, intensities } = parsePointCloud2(msg);
  if (positions.length === 0) return;

  const colors = applyColors(positions, intensities);

  const geo = pointCloud.geometry;
  geo.setAttribute('position', new THREE.Float32BufferAttribute(positions, 3));
  geo.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
  geo.attributes.position.needsUpdate = true;
  geo.attributes.color.needsUpdate = true;
  geo.computeBoundingSphere();

  totalPoints = positions.length / 3;
  $statPoints.textContent = totalPoints.toLocaleString();
}

/* ───────── ROS Connection ───────── */
function setStatus(state, text) {
  $status.className = `status ${state}`;
  $statusText.textContent = text;
}

function connect() {
  if (ros) {
    ros.close();
    ros = null;
  }

  setStatus('connecting', 'Connecting…');
  $btnConnect.textContent = 'Connecting…';
  $btnConnect.disabled = true;

  ros = new ROSLIB.Ros({ url: $url.value });

  ros.on('connection', () => {
    setStatus('connected', 'Connected');
    $btnConnect.textContent = 'Disconnect';
    $btnConnect.disabled = false;
    $btnSub.disabled = false;
    $btnSave.disabled = false;
    $btnLoad.disabled = false;
    toast('Connected to rosbridge', 'success');
    subscribe(); // auto-subscribe on connect
  });

  ros.on('error', (err) => {
    setStatus('error', 'Error');
    $btnConnect.textContent = 'Retry';
    $btnConnect.disabled = false;
    toast('Connection error: ' + (err.message || 'unknown'), 'error');
  });

  ros.on('close', () => {
    setStatus('disconnected', 'Disconnected');
    $btnConnect.textContent = 'Connect';
    $btnConnect.disabled = false;
    $btnSub.disabled = true;
    $btnSave.disabled = true;
    $btnLoad.disabled = true;
    if (pointcloudSub) {
      pointcloudSub = null;
    }
  });
}

function disconnect() {
  if (ros) {
    ros.close();
    ros = null;
  }
}

/* ───────── Subscribe ───────── */
function subscribe() {
  if (!ros) return;

  if (pointcloudSub) {
    pointcloudSub.unsubscribe();
    pointcloudSub = null;
  }

  const topic = $pclTopic.value.trim();
  if (!topic) return;

  pointcloudSub = new ROSLIB.Topic({
    ros: ros,
    name: topic,
    messageType: 'sensor_msgs/msg/PointCloud2',
    compression: 'none',
    throttle_rate: 200,  // max ~5 Hz for browser
  });

  pointcloudSub.subscribe((msg) => {
    updatePointCloud(msg);
  });

  toast(`Subscribed to ${topic}`, 'info');
}

/* ───────── Save / Load ───────── */
function saveMap() {
  if (!ros) return;

  const path = $savePath.value.trim();
  if (!path) {
    $ioStatus.textContent = 'Please enter a file path';
    $ioStatus.className = 'io-status error';
    return;
  }

  $ioStatus.textContent = 'Saving…';
  $ioStatus.className = 'io-status';

  const svc = new ROSLIB.Service({
    ros: ros,
    name: '/wavemap/save_map',
    serviceType: 'wavemap_msgs/srv/FilePath',
  });

  const req = new ROSLIB.ServiceRequest({ file_path: path });

  svc.callService(req, (result) => {
    $ioStatus.textContent = `✓ Saved to ${path}`;
    $ioStatus.className = 'io-status success';
    toast(`Map saved to ${path}`, 'success');
  }, (err) => {
    $ioStatus.textContent = `✗ Save failed`;
    $ioStatus.className = 'io-status error';
    toast('Save failed: ' + err, 'error');
  });
}

function loadMap() {
  if (!ros) return;

  const path = $loadPath.value.trim();
  if (!path) {
    $ioStatus.textContent = 'Please enter a file path';
    $ioStatus.className = 'io-status error';
    return;
  }

  $ioStatus.textContent = 'Loading…';
  $ioStatus.className = 'io-status';

  const svc = new ROSLIB.Service({
    ros: ros,
    name: '/wavemap/load_map',
    serviceType: 'wavemap_msgs/srv/FilePath',
  });

  const req = new ROSLIB.ServiceRequest({ file_path: path });

  svc.callService(req, (result) => {
    $ioStatus.textContent = `✓ Loaded from ${path}`;
    $ioStatus.className = 'io-status success';
    toast(`Map loaded from ${path}`, 'success');
  }, (err) => {
    $ioStatus.textContent = `✗ Load failed`;
    $ioStatus.className = 'io-status error';
    toast('Load failed: ' + err, 'error');
  });
}

/* ───────── Camera Presets ───────── */
function resetCamera() {
  camera.position.set(8, 6, 8);
  controls.target.set(0, 0, 0);
  controls.update();
}

function topDownView() {
  camera.position.set(0, 20, 0.01);
  controls.target.set(0, 0, 0);
  controls.update();
}

function frontView() {
  camera.position.set(0, 3, 15);
  controls.target.set(0, 0, 0);
  controls.update();
}

/* ───────── Event Listeners ───────── */
$btnConnect.addEventListener('click', () => {
  if (ros && ros.isConnected) {
    disconnect();
  } else {
    connect();
  }
});

$btnSub.addEventListener('click', subscribe);
$btnSave.addEventListener('click', saveMap);
$btnLoad.addEventListener('click', loadMap);
$btnClear.addEventListener('click', createEmptyPointCloud);
$btnResetCam.addEventListener('click', resetCamera);
$btnTop.addEventListener('click', topDownView);
$btnFront.addEventListener('click', frontView);

$pointSize.addEventListener('input', () => {
  const val = parseFloat($pointSize.value);
  $pointSizeVal.textContent = val.toFixed(1);
  if (pointCloud) {
    pointCloud.material.size = val;
  }
});

$colorMode.addEventListener('change', () => {
  // Re-color existing points
  if (pointCloud && pointCloud.geometry.attributes.position) {
    const positions = pointCloud.geometry.attributes.position.array;
    // Use height as a fallback for intensity
    const n = positions.length / 3;
    const intensities = new Float32Array(n);
    for (let i = 0; i < n; i++) {
      intensities[i] = positions[i * 3 + 1];
    }
    const colors = applyColors(positions, intensities);
    pointCloud.geometry.setAttribute('color', new THREE.Float32BufferAttribute(colors, 3));
    pointCloud.geometry.attributes.color.needsUpdate = true;
  }
});

$bgColor.addEventListener('change', () => {
  const color = BG_COLORS[$bgColor.value] || BG_COLORS.dark;
  scene.background = new THREE.Color(color);
  scene.fog.color = new THREE.Color(color);

  // Update grid colors based on theme
  if ($bgColor.value === 'light') {
    gridHelper.material[0].color.setHex(0xaaaaaa);
    gridHelper.material[1].color.setHex(0xcccccc);
  } else {
    gridHelper.material[0].color.setHex(0x2a2e3d);
    gridHelper.material[1].color.setHex(0x1e2230);
  }
});

$showGrid.addEventListener('change', () => {
  if (gridHelper) gridHelper.visible = $showGrid.checked;
});

$showAxes.addEventListener('change', () => {
  if (axesHelper) axesHelper.visible = $showAxes.checked;
});

// Allow Enter in URL field to connect
$url.addEventListener('keydown', (e) => {
  if (e.key === 'Enter') connect();
});

/* ───────── Init ───────── */
initThree();

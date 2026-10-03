import * as THREE from './vendor/three.module.js';
import { OrbitControls } from './vendor/OrbitControls.js';

const $ = (id) => document.getElementById(id);
const query = new URLSearchParams(location.search);
const dataRoot = new URL(query.get('data') || '../results/dynamic_filtering/temporal_review/', location.href);
const canvas = $('scene');
const renderer = new THREE.WebGLRenderer({ canvas, antialias: true, preserveDrawingBuffer: true });
renderer.setClearColor(0x101514, 1);
renderer.setPixelRatio(Math.min(devicePixelRatio || 1, 2));
const scene = new THREE.Scene();
const camera = new THREE.PerspectiveCamera(55, innerWidth / innerHeight, .005, 1000);
camera.up.set(0, 0, 1);
const controls = new OrbitControls(camera, canvas);
controls.enableDamping = false;
controls.screenSpacePanning = true;
const colors = [0x8b979f, 0x49bbae, 0xe86169, 0xe7b455].map((hex) => new THREE.Color(hex));
let report = null, region = null, frames = [], dataset = null;
let layer = null, context = null, source = 'scan', frameIndex = 0;
let token = 0, playing = false, playTime = 0, lastTick = 0, distance = 5;
const labels = { removal_00: 'Removal region 1', removal_01: 'Removal region 2', removal_02: 'Removal region 3',
  planar_vertical: 'Vertical plane', planar_horizontal: 'Horizontal plane', linear_geometry: 'Linear structure',
  uncertain_patch_14: 'Unconfirmed patch 14' };
const number = (value) => value.toLocaleString('en-US');

function icon(button, name) {
  button.replaceChildren();
  const element = document.createElement('i');
  element.dataset.lucide = name;
  button.append(element);
  lucide.createIcons();
}
function stop() {
  playing = false;
  icon($('play'), 'play');
  $('play').title = 'Play';
  $('play').setAttribute('aria-label', 'Play');
}
function dispose(object) {
  if (!object) return;
  scene.remove(object);
  object.geometry.dispose();
  object.material.dispose();
}
async function fetchData(url, type = 'json') {
  const response = await fetch(url);
  if (!response.ok) throw new Error(`${response.status}: ${url}`);
  return type === 'json' ? response.json() : response.arrayBuffer();
}
function cloud(points, classes, contextOnly = false) {
  const count = classes.length;
  if (points.length !== count * 4) throw new Error('Point/class count mismatch');
  const positions = new Float32Array(count * 3);
  const rgb = new Float32Array(count * 3);
  for (let i = 0; i < count; i++) {
    const color = contextOnly ? new THREE.Color(0x65706c) : colors[classes[i]];
    if (!color) throw new Error('Unknown point class');
    for (let a = 0; a < 3; a++) positions[i * 3 + a] = points[i * 4 + a] - region.center[a];
    color.toArray(rgb, i * 3);
  }
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.BufferAttribute(positions, 3));
  geometry.setAttribute('color', new THREE.BufferAttribute(rgb, 3));
  const material = new THREE.PointsMaterial({ size: Number($('point-size').value), vertexColors: true,
    transparent: contextOnly, opacity: contextOnly ? .22 : 1, depthWrite: !contextOnly });
  const object = new THREE.Points(geometry, material);
  object.frustumCulled = false;
  scene.add(object);
  return object;
}
function fit(view = 'fit') {
  if (!region) return;
  const panel = document.querySelector('.panel').getBoundingClientRect();
  const mobile = innerWidth <= 700;
  const left = mobile ? 12 : panel.right + 18;
  const bottom = mobile ? panel.top - 35 : innerHeight - 30;
  const width = Math.max(innerWidth - left - 12, 100);
  const height = Math.max(bottom - 85, 100);
  camera.aspect = innerWidth / innerHeight;
  camera.setViewOffset(innerWidth, innerHeight, innerWidth / 2 - (left + width / 2),
    innerHeight / 2 - (85 + height / 2), innerWidth, innerHeight);
  const radius = new THREE.Vector3(...region.upper).sub(new THREE.Vector3(...region.lower)).length() * .55;
  const room = Math.min(height / innerHeight, width / innerWidth * camera.aspect);
  distance = radius / Math.tan(THREE.MathUtils.degToRad(camera.fov / 2)) / room * 1.1;
  controls.target.set(0, 0, 0);
  camera.up.set(0, 0, 1);
  if (view === 'top') { camera.up.set(0, 1, 0); camera.position.set(0, 0, distance); }
  else if (view === 'front') camera.position.set(0, -distance, 0);
  else if (view === 'side') camera.position.set(distance, 0, 0);
  else camera.position.copy(new THREE.Vector3(1.25, -1.35, .95).normalize().multiplyScalar(distance));
  camera.near = .005;
  camera.far = Math.max(distance * 20, 100);
  camera.updateProjectionMatrix();
  controls.update();
}
function setFrame(index) {
  if (!frames.length) return;
  frameIndex = THREE.MathUtils.clamp(index, 0, frames.length - 1);
  const frame = frames[frameIndex];
  if (source === 'scan' && layer) layer.geometry.setDrawRange(frame.offset, frame.count);
  $('time').value = String(frame.seconds);
  $('time-number').value = frame.seconds.toFixed(1);
  $('elapsed').textContent = `${frame.seconds.toFixed(1)} s`;
  $('frame-number').textContent = `Frame ${frame.frame_index}`;
  canvas.dataset.frame = String(frame.frame_index);
  canvas.dataset.seconds = String(frame.seconds);
  canvas.dataset.source = source;
  let classes;
  if (source === 'scan') classes = dataset.scanClasses.subarray(frame.offset, frame.offset + frame.count);
  else classes = dataset.baselineClasses.filter((value) => source === 'baseline' ||
    (source === 'filtered' ? value !== 2 : value === 2));
  const counts = [0, 0, 0, 0];
  for (const value of classes) counts[value]++;
  $('points').textContent = number(classes.length);
  ['unassociated', 'retained', 'removed', 'restored'].forEach((id, i) => $(id).textContent = number(counts[i]));
  $('empty').hidden = source !== 'scan' || frame.count > 0;
  $('context').disabled = source !== 'scan';
  if (context) context.visible = source === 'scan' && $('context').checked;
}
function switchSource(next) {
  stop();
  source = next;
  document.querySelectorAll('[data-source]').forEach((button) => button.classList.toggle('selected', button.dataset.source === source));
  if (!dataset) return;
  dispose(layer);
  if (source === 'scan') layer = cloud(dataset.scan, dataset.scanClasses);
  else {
    const ids = [];
    dataset.baselineClasses.forEach((value, i) => {
      if (source === 'baseline' || (source === 'filtered' ? value !== 2 : value === 2)) ids.push(i);
    });
    const packed = new Float32Array(ids.length * 4), classes = new Uint8Array(ids.length);
    ids.forEach((i, j) => { packed.set(dataset.baseline.subarray(i * 4, i * 4 + 4), j * 4); classes[j] = dataset.baselineClasses[i]; });
    layer = cloud(packed, classes);
  }
  setFrame(frameIndex);
}
function seek(seconds) {
  if (!frames.length || !Number.isFinite(seconds)) return;
  let low = 0, high = frames.length;
  while (low < high) {
    const middle = (low + high) >>> 1;
    if (frames[middle].seconds < seconds) low = middle + 1;
    else high = middle;
  }
  const index = Math.min(low, frames.length - 1);
  setFrame(index > 0 && Math.abs(frames[index - 1].seconds - seconds) < Math.abs(frames[index].seconds - seconds) ? index - 1 : index);
}
function fail(error) {
  $('status').textContent = 'Load failed';
  $('error').textContent = error.message;
  $('error').hidden = false;
  canvas.dataset.ready = 'false';
}
async function loadRegion(initialTime = null) {
  const current = ++token;
  stop();
  dataset = null;
  frames = [];
  dispose(layer); dispose(context); layer = null; context = null;
  $('error').hidden = true;
  $('status').textContent = 'Loading region';
  canvas.dataset.ready = 'false';
  const selected = report.regions.find((r) => r.key === $('region').value);
  const root = new URL(`${report.id}/${selected.key}/`, dataRoot);
  const files = await Promise.all(['frames.json', 'baseline.bin', 'baseline_classes.bin', 'scan.bin', 'scan_classes.bin']
    .map((name) => fetchData(new URL(name, root), name.endsWith('.json') ? 'json' : 'binary')));
  if (current !== token) return;
  region = selected;
  frames = files[0];
  dataset = { baseline: new Float32Array(files[1]), baselineClasses: new Uint8Array(files[2]),
    scan: new Float32Array(files[3]), scanClasses: new Uint8Array(files[4]) };
  if (!frames.length || frames.some((frame) => frame.offset < 0 || frame.count < 0 || frame.offset + frame.count > dataset.scanClasses.length)) {
    throw new Error('Invalid frame offsets');
  }
  context = cloud(dataset.baseline, dataset.baselineClasses, true);
  switchSource('scan');
  if (initialTime !== null && Number.isFinite(initialTime)) seek(initialTime);
  else {
    const target = region.kind.startsWith('persistent') ? region.peak_visible_frame : region.peak_removed_frame;
    const best = frames.reduce((a, b) => Math.abs(a.frame_index - target) < Math.abs(b.frame_index - target) ? a : b);
    setFrame(frames.indexOf(best));
  }
  for (const id of ['time', 'time-number']) { $(id).min = String(frames[0].seconds); $(id).max = String(frames.at(-1).seconds); }
  fit();
  $('status').textContent = `${frames.length} sampled frames`;
  canvas.dataset.ready = 'true';
  canvas.dataset.recording = report.id;
  canvas.dataset.region = region.key;
}
async function loadRecording(initial = false) {
  const current = ++token;
  stop();
  const id = $('recording').value;
  $('status').textContent = 'Loading recording';
  $('recording-title').textContent = `MID360 / ${$('recording').selectedOptions[0].text}`;
  const next = await fetchData(new URL(`${id}/report.json`, dataRoot));
  if (current !== token) return;
  report = next;
  $('region').replaceChildren();
  for (const item of report.regions) {
    const option = document.createElement('option'); option.value = item.key; option.textContent = labels[item.key] || item.key;
    $('region').append(option);
  }
  if (initial && report.regions.some((item) => item.key === query.get('region'))) $('region').value = query.get('region');
  await loadRegion(initial && query.has('t') ? Number(query.get('t')) : null);
}
document.querySelectorAll('[data-source]').forEach((button) => button.addEventListener('click', () => switchSource(button.dataset.source)));
$('recording').addEventListener('change', () => loadRecording().catch(fail));
$('region').addEventListener('change', () => loadRegion().catch(fail));
$('time').addEventListener('input', () => { stop(); seek(Number($('time').value)); });
$('time-number').addEventListener('change', () => { stop(); seek(Number($('time-number').value)); });
$('previous').addEventListener('click', () => { stop(); setFrame(frameIndex - 1); });
$('next').addEventListener('click', () => { stop(); setFrame(frameIndex + 1); });
$('play').addEventListener('click', () => {
  if (!frames.length || source !== 'scan') return;
  if (playing) { stop(); return; }
  if (frameIndex === frames.length - 1) setFrame(0);
  playing = true; playTime = frames[frameIndex].seconds; lastTick = performance.now();
  icon($('play'), 'pause'); $('play').title = 'Pause'; $('play').setAttribute('aria-label', 'Pause');
});
$('context').addEventListener('change', () => setFrame(frameIndex));
$('point-size').addEventListener('input', () => {
  $('point-size-value').textContent = Number($('point-size').value).toFixed(3);
  for (const object of [layer, context]) if (object) object.material.size = Number($('point-size').value);
});
for (const id of ['fit', 'top', 'front', 'side']) $(id).addEventListener('click', () => fit(id));
$('screenshot').addEventListener('click', () => {
  renderer.render(scene, camera);
  const link = document.createElement('a'); link.download = `mid360-${report?.id || 'review'}-${region?.key || ''}.png`;
  link.href = canvas.toDataURL('image/png'); link.click();
});
function resize() { renderer.setSize(innerWidth, innerHeight, false); fit(); }
window.addEventListener('resize', resize);
function render(now) {
  if (playing) {
    playTime += Math.min((now - lastTick) / 1000, .25) * Number($('speed').value);
    lastTick = now;
    seek(playTime);
    if (playTime >= frames.at(-1).seconds) stop();
  }
  controls.update(); renderer.render(scene, camera); requestAnimationFrame(render);
}
if ([...$('recording').options].some((option) => option.value === query.get('bag'))) $('recording').value = query.get('bag');
lucide.createIcons(); resize(); requestAnimationFrame(render); loadRecording(true).catch(fail);

// T-cam frame layout: where the car's parts sit in the onboard frame, for calibrating the camera
// against real onboard footage.
//
//   node tools/tcam-measure.mjs <outdir> [W=1280] [H=720] [cam=tcam]
//
// Renders the onboard camera (car standing on the start straight, level) at W x H and writes:
//  * <cam>_W x H_mask.png : every car part group in one flat colour (sky / track black), and the
//    projected feature points as crosses - for side-by-side and overlay with a reference frame;
//  * <cam>_W x H_frame.png: the normal render without the HUD;
//  * a JSON report (stdout and .json): per group the screen bounding box and coverage (% of the
//    frame), and the pixel positions of identifiable 3-D feature points measured on the model
//    geometry itself (halo apex and its widest points, the pillar foot, the mirror housings'
//    outer tips, the tops of the front tyres, the helmet crown, the nose at the front axle, and
//    the horizon line). The same features read off a real onboard frame give the 2-D side of
//    a camera-pose fit (position, angles and field of view of the real camera).
import { createRequire } from 'module';
const require = createRequire(import.meta.url);
const { chromium } = require('playwright');
import { spawn } from 'child_process';
import path from 'path';
import fs from 'fs';

const out = process.argv[2] || '.';
const W = +(process.argv[3] || 1280), H = +(process.argv[4] || 720), CAM = process.argv[5] || 'tcam';
const PORT = +(process.env.PORT || 8797);
fs.mkdirSync(out, { recursive: true });
const srv = spawn(process.execPath, [path.join(path.dirname(new URL(import.meta.url).pathname), 'serve.mjs'), String(PORT)], { stdio: 'ignore' });
await new Promise(r => setTimeout(r, 400));
const browser = await chromium.launch({ args: ['--use-gl=angle', '--use-angle=swiftshader', '--enable-unsafe-swiftshader'] });
const page = await browser.newPage({ viewport: { width: W, height: H }, deviceScaleFactor: 1 });
const errs = [];
page.on('pageerror', e => errs.push(e.message));
await page.goto(`http://localhost:${PORT}/?start=drive&noaudio=1&q=high&cam=${CAM}${process.env.EXTRA ? '&' + process.env.EXTRA : ''}`);
await page.waitForFunction(() => window.__ready === true, null, { timeout: 180000 });

const rep = await page.evaluate(async ([W, H, CAM]) => {
  const a = window.__app, THREE = a.THREE, T = a.track;
  // the car standing still on the start straight, the onboard camera as in the game
  a.__autoOverride = false; a.controls.enabled = false; a.controls.clear();
  a.veh.reset(T.spawnS, 0, { speed: 0 }); a.timing.teleport(a.veh.s);
  a.rig.setMode(CAM);
  await new Promise(r => { let n = 0; const f = () => { if (++n > 20) r(); else requestAnimationFrame(f); }; requestAnimationFrame(f); });
  const cam = a.camera, car = a.car;
  car.root.updateMatrixWorld(true); cam.updateMatrixWorld(true);
  const chassis = car.chassis, Minv = new THREE.Matrix4().copy(chassis.matrixWorld).invert();

  // part groups by mesh name / ancestor name
  const GROUPS = [
    ['helmet', ['helmet', 'driver']], ['halo', ['halo']], ['mirrors', ['mirrorL', 'mirrorR']],
    ['tyresFront', ['spinFL', 'spinFR', 'hubFL', 'hubFR', 'steerFL', 'steerFR']], ['tyresRear', ['spinRL', 'spinRR', 'hubRL', 'hubRR', 'steerRL', 'steerRR']],
    ['frontWing', ['frontWing', 'fwL', 'fwR']], ['nose', ['nose']], ['suspension', ['suspension']], ['sidepods', ['sidepodL', 'sidepodR']],
    ['tcamPod', ['tcamPod']], ['wheel', ['steeringWheel', 'driverArms']], ['body', ['chassis']],
  ];
  const COL = { helmet: 0xffffff, halo: 0xff2020, mirrors: 0x20ff20, tyresFront: 0x2060ff, tyresRear: 0x00ffff, frontWing: 0xffa000, nose: 0xff00ff, suspension: 0xffff00, sidepods: 0x8040ff, tcamPod: 0x804000, wheel: 0x00a060, body: 0x808080 };
  const groupOf = (o) => {
    for (let p = o; p && p !== car.root; p = p.parent) for (const [g, names] of GROUPS) if (g !== 'body' && names.includes(p.name)) return g;
    return 'body';
  };
  const meshes = [];
  car.root.traverse((o) => { if (o.isMesh && o.visible && o.name !== 'contactShadow' && !(o.material && o.material.transparent && o.material.opacity < 0.05)) meshes.push(o); });

  // ---- feature points from the geometry (chassis frame), projected to pixels ----
  const proj = (wp) => { const v = wp.clone().project(cam); return [(v.x * 0.5 + 0.5) * W, (1 - (v.y * 0.5 + 0.5)) * H, v.z]; };
  const vtx = (g) => { const pts = []; for (const m of meshes.filter(m => groupOf(m) === g)) { const pa = m.geometry.attributes.position; m.updateMatrixWorld(true); for (let i = 0; i < pa.count; i += 1) pts.push(new THREE.Vector3().fromBufferAttribute(pa, i).applyMatrix4(m.matrixWorld)); } return pts; };
  const inChassis = (p) => p.clone().applyMatrix4(Minv);
  const ext = (pts, score) => { let best = null, bs = -Infinity; for (const p of pts) { const s = score(inChassis(p), p); if (s > bs) { bs = s; best = p; } } return best; };
  const F = {};
  const halo = vtx('halo');
  if (halo.length) {
    F.haloApex = ext(halo, (c) => c.y - 4 * Math.abs(c.x));                 // top of the hoop on the centre line
    F.haloLeft = ext(halo, (c) => -c.x + 0.2 * c.y);                         // widest points
    F.haloRight = ext(halo, (c) => c.x + 0.2 * c.y);
    F.haloFrontMost = ext(halo, (c) => -c.z - 3 * Math.abs(c.x));             // pillar foot / leading point
  }
  const mir = vtx('mirrors');
  if (mir.length) { F.mirrorLeftTip = ext(mir, (c) => -c.x); F.mirrorRightTip = ext(mir, (c) => c.x); }
  // the rotating wheels only (tyre + rim, not the uprights and brake ducts)
  const spinVtx = (names) => { const pts = []; for (const m of meshes) { let p = m; while (p && !names.includes(p.name)) p = p.parent; if (!p) continue; const pa = m.geometry.attributes.position; for (let i = 0; i < pa.count; i++) pts.push(new THREE.Vector3().fromBufferAttribute(pa, i).applyMatrix4(m.matrixWorld)); } return pts; };
  const tl = spinVtx(['spinFL']), tr = spinVtx(['spinFR']);
  if (tl.length && tr.length) {
    F.tyreFLTop = ext(tl, (c) => c.y); F.tyreFRTop = ext(tr, (c) => c.y);
    F.tyreFLInnerTop = ext(tl, (c) => c.x + 0.5 * c.y); F.tyreFRInnerTop = ext(tr, (c) => -c.x + 0.5 * c.y);
  }
  const hel = vtx('helmet');
  if (hel.length) F.helmetCrown = ext(hel, (c) => c.y);
  const nose = vtx('nose');
  if (nose.length) F.noseTip = ext(nose, (c) => -c.z);
  const pts = {};
  for (const [k, p] of Object.entries(F)) if (p) { const c = inChassis(p); pts[k] = { px: proj(p).slice(0, 2).map(v => +v.toFixed(1)), chassis: [c.x, c.y, c.z].map(v => +v.toFixed(3)) }; }
  // horizon: a point far ahead along the car's heading at the camera's height
  const fwd = new THREE.Vector3(0, 0, -1).applyQuaternion(chassis.getWorldQuaternion(new THREE.Quaternion())); fwd.y = 0; fwd.normalize();
  const far = cam.position.clone().addScaledVector(fwd, 5000);
  const hz = proj(far);
  // camera in the chassis frame, its pitch / FOV
  const cc = inChassis(cam.getWorldPosition(new THREE.Vector3()));
  const camInfo = { posChassis: [cc.x, cc.y, cc.z].map(v => +v.toFixed(3)), vfov: +cam.fov.toFixed(2), hfov: +(2 * Math.atan(Math.tan(cam.fov * Math.PI / 360) * cam.aspect) * 180 / Math.PI).toFixed(2), aspect: +cam.aspect.toFixed(4) };

  // ---- part-ID render into a target ----
  const rt = new THREE.WebGLRenderTarget(W, H, { samples: 0 });
  const saved = meshes.map(m => [m, m.material]);
  const mats = {};
  // (colours set as linear values: a render target stores them unconverted, so they read back exactly)
  const linCol = (c) => new THREE.Color().setRGB(((c >> 16) & 255) / 255, ((c >> 8) & 255) / 255, (c & 255) / 255, THREE.LinearSRGBColorSpace);
  for (const m of meshes) { const g = groupOf(m); mats[g] = mats[g] || new THREE.MeshBasicMaterial({ color: linCol(COL[g]), side: THREE.DoubleSide, toneMapped: false }); m.material = mats[g]; }
  const hide = []; a.scene.traverse(o => { if (o.isMesh && !meshes.includes(o) && o.visible) { o.visible = false; hide.push(o); } });
  const bg = a.scene.background, fog = a.scene.fog; a.scene.background = new THREE.Color(0x000000); a.scene.fog = null;
  a.renderer.setRenderTarget(rt); a.renderer.clear(); a.renderer.render(a.scene, cam); a.renderer.setRenderTarget(null);
  const px = new Uint8Array(W * H * 4); a.renderer.readRenderTargetPixels(rt, 0, 0, W, H, px);
  for (const [m, mt] of saved) m.material = mt; for (const o of hide) o.visible = true; a.scene.background = bg; a.scene.fog = fog;
  // stats per group (colour match), image rows flipped (GL origin bottom-left)
  const groups = {}, CL = Object.entries(COL).map(([g, c]) => [g, (c >> 16) & 255, (c >> 8) & 255, c & 255]);
  const match = (r, gg, b) => { for (const [g, cr, cg, cb] of CL) if (Math.abs(r - cr) <= 4 && Math.abs(gg - cg) <= 4 && Math.abs(b - cb) <= 4) return g; return null; };
  const cv = document.createElement('canvas'); cv.width = W; cv.height = H; const cx = cv.getContext('2d'); const id = cx.createImageData(W, H);
  for (let y = 0; y < H; y++) for (let x = 0; x < W; x++) {
    const i = ((H - 1 - y) * W + x) * 4, o = (y * W + x) * 4;
    id.data[o] = px[i]; id.data[o + 1] = px[i + 1]; id.data[o + 2] = px[i + 2]; id.data[o + 3] = 255;
    const g = match(px[i], px[i + 1], px[i + 2]); if (!g) continue;
    const s = groups[g] || (groups[g] = { n: 0, x0: W, y0: H, x1: 0, y1: 0 });
    s.n++; if (x < s.x0) s.x0 = x; if (x > s.x1) s.x1 = x; if (y < s.y0) s.y0 = y; if (y > s.y1) s.y1 = y;
  }
  cx.putImageData(id, 0, 0);
  // feature crosses
  cx.strokeStyle = '#000'; cx.lineWidth = 3; const cross = (x, y) => { cx.beginPath(); cx.moveTo(x - 9, y); cx.lineTo(x + 9, y); cx.moveTo(x, y - 9); cx.lineTo(x, y + 9); cx.stroke(); };
  for (const v of Object.values(pts)) cross(v.px[0], v.px[1]);
  cx.strokeStyle = '#00ff80'; cx.lineWidth = 1; cx.beginPath(); cx.moveTo(0, hz[1]); cx.lineTo(W, hz[1]); cx.stroke();
  const maskURL = cv.toDataURL('image/png');
  const gs = {}; for (const [g, s] of Object.entries(groups)) gs[g] = { coverPct: +(100 * s.n / (W * H)).toFixed(2), bboxPct: [s.x0 / W, s.y0 / H, s.x1 / W, s.y1 / H].map(v => +(100 * v).toFixed(1)) };
  return { W, H, cam: camInfo, horizonY: +hz[1].toFixed(1), horizonPct: +(100 * hz[1] / H).toFixed(1), features: pts, groups: gs, maskURL };
}, [W, H, CAM]);

const tag = `${CAM}_${W}x${H}`;
fs.writeFileSync(path.join(out, `${tag}_mask.png`), Buffer.from(rep.maskURL.split(',')[1], 'base64'));
delete rep.maskURL;
fs.writeFileSync(path.join(out, `${tag}.json`), JSON.stringify(rep, null, 1));
console.log(JSON.stringify(rep, null, 1));
// the normal frame without the HUD (software GL: a frame can take seconds)
await page.evaluate(() => { for (const id of ['hud', 'controls']) { const e = document.getElementById(id); if (e) e.style.visibility = 'hidden'; } });
await page.waitForTimeout(1500);
await page.screenshot({ path: path.join(out, `${tag}_frame.png`), timeout: 180000 });
if (errs.length) console.log('page errors:', errs.join(' | '));
await browser.close(); srv.kill();

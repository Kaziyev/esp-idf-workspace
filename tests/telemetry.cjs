const fs = require('node:fs');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const path = require('node:path');
const source = fs.readFileSync(path.join(__dirname, '../web/app.js'), 'utf8');
const elements = new Map();
let clock = 1000;
function element(id) {
  if (!elements.has(id)) {
    const calls = [];
    const context = new Proxy({ calls, createRadialGradient: () => ({ addColorStop() {} }) }, {
      get: (o, k) => k in o ? o[k] : (...args) => calls.push([k, ...args]),
    });
    elements.set(id, {
      textContent: '', style: {}, classList: { toggle() {}, add() {}, remove() {} },
      children: Array.from({ length: 9 }, () => ({ textContent: '' })),
      setAttribute() {}, addEventListener() {}, getContext: () => context,
      clientWidth: 640, clientHeight: 400, width: 0, height: 0, value: '115200', context,
    });
  }
  return elements.get(id);
}
let exportedBlob;
const sandbox = {
  document: { getElementById: element, createElement: () => ({ click() {} }) },
  window: { matchMedia: () => ({ matches: false }), devicePixelRatio: 1 },
  navigator: {}, performance: { now: () => clock }, requestAnimationFrame() {},
  getComputedStyle: () => ({ fontFamily: 'Arial' }), setTimeout() {},
  TextDecoder, TextEncoder, Blob,
  URL: { createObjectURL: b => { exportedBlob = b; return 'blob:test'; }, revokeObjectURL() {} },
};
vm.createContext(sandbox);
vm.runInContext(source + '\nthis.api={normalizePacket,TelemetryParser,BrowserAHRS,state,setMode,receive,updateUI,drawScene,drawCharts,acceptSerialBytes,rawUnits,togglePause,resetBrowserView,setCharts};', sandbox);
const a = sandbox.api, plain = v => JSON.parse(JSON.stringify(v));
const sample = { ax: .1911, ay: -.1360, az: .9778, gx: .183, gy: -.183, gz: -.183, mx: -11, my: 39, mz: 32 };
const close = (actual, expected, tolerance = 1e-6) => assert(Math.abs(actual - expected) < tolerance, `${actual} != ${expected}`);
const level = { ax: 0, ay: 0, az: 1, gx: 0, gy: 0, gz: 0, mx: 25, my: 0, mz: -40 };
function filtered(filter, data, t, timeBasis = 'device') {
  const p = a.normalizePacket(data); p.t = t; p.timeBasis = timeBasis; filter.update(p); return p;
}

// The reported sensor-only schema must be accepted without fabricating firmware q.
const raw = a.normalizePacket(sample);
assert.equal(raw.q, null); assert.equal(raw.estimated, true);
assert.deepEqual(plain(raw.a), [.1911, -.136, .9778]);
assert.deepEqual(plain(raw.g), [.183, -.183, -.183]);
assert.deepEqual(plain(raw.m), [-11, 39, 32]);
assert.equal(a.normalizePacket({ ax: null, ay: '', az: true }), null);
assert.equal(a.normalizePacket({ arbitrary: 12 }), null);
const external = a.normalizePacket({ ...sample, q: [1, 0, 0, 0], dt_s: 'bad' });
assert.equal(external.estimated, false); assert.equal(external.dt, null); assert(external.warnings.length);
assert.deepEqual(plain(a.normalizePacket({ q: { w: '1', x: 0, y: 0, z: 0 } }).q), [1, 0, 0, 0]);

// Static tilt signs and magnetic yaw under the stated right-handed body convention.
let f = new a.BrowserAHRS();
const tilt = filtered(f, sample, 0);
close(tilt.rpy[0], Math.atan2(sample.ay, sample.az) * 180 / Math.PI);
close(tilt.rpy[1], Math.atan2(-sample.ax, Math.hypot(sample.ay, sample.az)) * 180 / Math.PI);
close(tilt.rpy[2], 0);
f = new a.BrowserAHRS(); filtered(f, level, 0);
let yaw;
for (let i = 1; i <= 500; i++) yaw = filtered(f, { ...level, mx: 0, my: -25 }, i * .02);
close(yaw.rpy[2], 90, .001);
f = new a.BrowserAHRS(); filtered(f, level, 0);
let roll;
for (let i = 1; i <= 500; i++) roll = filtered(f, { ...level, ay: 1, az: 0, my: -40, mz: 0 }, i * .02);
close(roll.rpy[0], 90, .001); close(roll.rpy[1], 0, .001);

// Gyro must contribute even when acceleration and magnetic data cannot correct it.
f = new a.BrowserAHRS(); filtered(f, level, 0);
let gyro;
for (let i = 1; i <= 50; i++) gyro = filtered(f, { ...level, az: 2, gx: 90, mx: 0, my: 0, mz: 0 }, i * .02);
close(gyro.rpy[0], 90, .001); assert(gyro.filterGyro); assert(!gyro.filterAcc); assert(!gyro.filterMag);
const duplicate = filtered(f, { ...level, az: 2, gx: 900, mx: 0, my: 0, mz: 0 }, 1);
close(duplicate.rpy[0], 90, .001); assert.equal(duplicate.filterDt, null);
const gap = filtered(f, level, 3); assert.equal(gap.filterDt, null); close(gap.rpy[0], 0, .001);

// A log with no measurement timestamps never integrates a guessed gyro interval.
f = new a.BrowserAHRS(); filtered(f, level, 0, 'log-assumed');
const untimed = filtered(f, { ...level, gx: 10000 }, 1, 'log-assumed');
close(untimed.rpy[0], 0); assert.equal(untimed.filterDt, null);

// Units are explicit choices; canonical keys are independent of those choices.
a.rawUnits.acc = 'ms2'; a.rawUnits.gyro = 'rads'; a.rawUnits.mag = 'counts';
const converted = a.normalizePacket({ ...level, az: 9.80665, gx: Math.PI });
close(converted.a[2], 1); close(converted.g[0], 180); assert.equal(converted.magUnit, 'counts');
close(a.normalizePacket({ a_g: [0, 0, 1], g_dps: [10, 0, 0] }).g[0], 10);
a.rawUnits.acc = 'g'; a.rawUnits.gyro = 'dps'; a.rawUnits.mag = 'uT';

// Replay the actual attached diagnostic when supplied, including its header and last line without LF.
const text = process.argv[2] ? fs.readFileSync(process.argv[2], 'utf8') : Array.from({ length: 120 }, () => JSON.stringify(sample)).join('\r\n');
const expected = text.split(/\r?\n/).filter(s => s.startsWith('{')).length;
const packets = [], errors = [];
const parser = new a.TelemetryParser(p => packets.push(p), e => errors.push(e));
for (let i = 0; i < text.length; i += 13) parser.feed(text.slice(i, i + 13));
parser.flush(); assert.equal(packets.length, expected); assert.equal(errors.length, 0);
a.setMode('serial'); a.state.port = {};
packets.forEach((p, i) => a.receive(p, { hostNow: 1000 + i * 20 }));
a.updateUI(1000 + expected * 20); a.drawScene(clock); a.drawCharts();
assert.equal(a.state.count, expected); assert(a.state.display.q.every(Number.isFinite));
assert.equal(element('fusion-badge').textContent, 'AHRS браузера');
assert.equal(element('a0').textContent, packets.at(-1).a[0].toFixed(3));
assert.equal(element('serial-frames').textContent, expected);
assert.equal(element('det').textContent, 'det(R) = 1.000000');
assert(element('chart-0').context.calls.filter(c => c[0] === 'lineTo').length > 30);
const modelBefore = JSON.stringify(element('scene').context.calls);
a.receive(a.normalizePacket({ ...level, q: [Math.SQRT1_2, Math.SQRT1_2, 0, 0] }), { hostNow: 4000 });
element('scene').context.calls.length = 0; a.drawScene(clock); a.updateUI(clock);
assert.notEqual(modelBefore, JSON.stringify(element('scene').context.calls));
assert.equal(element('roll').textContent, '90.0°'); assert.equal(element('fusion-badge').textContent, 'Из прошивки');
a.togglePause(); const frozen = a.state.display;
a.receive(a.normalizePacket(level), { hostNow: 4100 }); assert.equal(a.state.display, frozen);
a.togglePause(); assert.equal(a.state.display, a.state.latest);

// Sensor graphs, matrix placeholders and CSV must also work without usable attitude.
a.setMode('serial'); a.state.port = {}; a.receive(a.normalizePacket({ gx: 1, gy: 2, gz: 3 }));
a.updateUI(clock); a.drawCharts(); a.drawScene(clock);
assert.equal(element('roll').textContent, '—'); assert.equal(element('g0').textContent, '1.00');
assert.equal(element('matrix').children[0].textContent, '—'); element('export').onclick(); assert(exportedBlob);
// Live decoder path: chunked frames at changing receive times.
a.setMode('serial'); const decoder = new TextDecoder();
for (let i = 0; i < 10; i++) { clock += 20; const bytes = new TextEncoder().encode(JSON.stringify(sample) + '\r\n'); a.acceptSerialBytes(bytes.slice(0, 20), decoder); a.acceptSerialBytes(bytes.slice(20), decoder); }
assert.equal(a.state.count, 10); assert.equal(a.state.errors, 0); assert.equal(a.state.display.timeBasis, 'usb-arrival');
console.log(`PASS: ${expected} diagnostic frames, 0 rejected; raw/firmware paths, tilt/yaw/gyro physics, timing gaps, units, model/matrix/charts, pause, CSV, chunked Serial.`);

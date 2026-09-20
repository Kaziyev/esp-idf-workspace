const fs = require('node:fs');
const vm = require('node:vm');
const assert = require('node:assert/strict');
const path = require('node:path');
const source = fs.readFileSync(path.join(__dirname, '../web/app.js'), 'utf8');
const html = fs.readFileSync(path.join(__dirname, '../web/index.html'), 'utf8');
const htmlIds = [...html.matchAll(/\bid="([^"]+)"/g)].map(match => match[1]);
assert.equal(new Set(htmlIds).size, htmlIds.length, 'HTML IDs must be unique');
for (const [, id] of source.matchAll(/\$\('([^']+)'\)/g)) assert(htmlIds.includes(id), `Missing UI element: ${id}`);
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
// The original generic-frame tests explicitly use native IMU axes and aligned MAG.
const storage = new Map([
  ['qav250.imu-mount.v1', JSON.stringify({ version: 1, source: 'native', sensorToBody: [1, 0, 0, 0] })],
  ['qav250.legacy-mag-axes.v1', 'aligned'],
]);
const sandbox = {
  document: { getElementById: element, createElement: () => ({ click() {} }) },
  window: { matchMedia: () => ({ matches: false }), devicePixelRatio: 1 },
  navigator: {}, performance: { now: () => clock }, requestAnimationFrame() {},
  getComputedStyle: () => ({ fontFamily: 'Arial' }), setTimeout() {},
  TextDecoder, TextEncoder, Blob,
  localStorage: { getItem: key => storage.get(key) ?? null, setItem: (key, value) => storage.set(key, value), removeItem: key => storage.delete(key) },
  URL: { createObjectURL: b => { exportedBlob = b; return 'blob:test'; }, revokeObjectURL() {} },
};
vm.createContext(sandbox);
vm.runInContext(source + '\nthis.api={normalizePacket,TelemetryParser,BrowserAHRS,state,setMode,receive,updateUI,drawScene,drawCharts,acceptSerialBytes,rawUnits,togglePause,resetBrowserView,setCharts,IMUFrame,imuFrame,quatEuler,qmul,qconj,rotation,mv,mtv,determinant,captureStablePose,zeroHeading,magAxes,photoMountQ,demoPacket};', sandbox);
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
const text = process.argv[2] && process.argv[2] !== '--barometer-exe' ? fs.readFileSync(process.argv[2], 'utf8') : Array.from({ length: 120 }, () => JSON.stringify(sample)).join('\r\n');
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
// Native sensor S can be rotated sideways, tilted 10 degrees, or mounted upside down.
// Build a physical board orientation independently of the measured sensor attitude.
const radians = Math.PI / 180;
const closeVector = (actual, target, tolerance = 1e-6) => target.forEach((v, i) => close(actual[i], v, tolerance));
function sensorPacket(mount, rpy, heading = 0, firmware = true) {
  const boardQ = a.quatEuler(rpy), boardR = a.rotation(boardQ), mountR = a.rotation(mount);
  const sensorVector = v => a.mtv(mountR, v);
  const packet = { mag_frame: 'bmi270', a_g: sensorVector(a.mtv(boardR, [0, 0, 1])),
    g_dps: sensorVector([0, 0, 0]), m_uT: sensorVector(a.mtv(boardR, [25, 0, -40])),
    bias_dps: sensorVector([.1, -.2, .3]) };
  if (firmware) packet.q = a.qmul(a.quatEuler([0, 0, heading]), a.qmul(boardQ, mount));
  return packet;
}
for (const angles of [[10, 0, 90], [-10, 0, -90], [10, -7, 100], [0, 10, 0], [180, 0, 0], [0, 180, 0], [0, 0, 180]]) {
  const mount = a.quatEuler(angles), neutral = sensorPacket(mount, [0, 0, 0], 57), forward = sensorPacket(mount, [0, 30, 0], 57);
  const frame = new a.IMUFrame(); frame.setMount(a.IMUFrame.fromPoses(neutral.a_g, forward.a_g)); frame.reference(neutral.q);
  close(a.determinant(a.rotation(frame.sensorToBody)), 1);
  for (const rpy of [[0, 0, 0], [0, 35, 0], [0, -35, 0], [25, 0, 0], [-25, 0, 0], [0, 0, 40], [17, -23, 61]]) {
    const input = sensorPacket(mount, rpy, 57), p = a.normalizePacket(input);
    const bodyG = [1, 2, 3]; p.g = a.mtv(a.rotation(mount), bodyG);
    p.mode = 'serial'; frame.apply(p); const firstQ = p.q.slice();
    closeVector(p.rpy, rpy); closeVector(p.g, bodyG); closeVector(p.b, [.1, -.2, .3]);
    closeVector(a.mv(a.rotation(p.q), p.a), [0, 0, 1]);
    closeVector(a.mv(a.rotation(p.q), p.m), [25, 0, -40]);
    closeVector(p.sensor.a, input.a_g); assert.equal(p.coordinateFrame, 'body_calibrated');
    frame.apply(p); closeVector(p.q, firstQ); // No double correction on redraw/history.
    const front = a.mv(a.rotation(p.q), [1, 0, 0]);
    if (rpy[0] === 0 && rpy[2] === 0) close(front[2], -Math.sin(rpy[1] * radians));
  }
}
assert.throws(() => a.IMUFrame.fromPoses([0, 0, 1], [0, 0, 1]), /15–60/);
assert.throws(() => a.IMUFrame.fromPoses([0, 0, 1], [1, 0, 0]), /15–60/);
assert.throws(() => a.IMUFrame.fromPoses([0, 0, 2], [0, 0, 1]), /1 g/);
assert.throws(() => new a.IMUFrame().setMount([0, 0, 0, 0]), /Некорректная/);

// Exercise the actual two-button workflow with a 90-degree sideways / 10-degree tilt mount.
const tiltedMount = a.quatEuler([10, 0, 90]);
a.setMode('serial'); a.state.port = {};
function holdPose(rpy, options = {}) {
  for (let i = 0; i < 15; i++) {
    clock += 100;
    const input = sensorPacket(tiltedMount, rpy, 37, options.firmware !== false);
    if (options.moving) input.g_dps = [20, 0, 0];
    if (options.firmware === false) delete input.bias_dps;
    a.receive(a.normalizePacket(input), { hostNow: clock });
  }
}
holdPose([0, 0, 0]); element('calibrate-level').onclick();
assert.match(element('frame-note').textContent, /Шаг 1 готов/);
holdPose([0, 30, 0]); element('calibrate-forward').onclick();
assert(a.imuFrame.calibrated); assert(storage.has('qav250.imu-mount.v1'));
// A fresh page loads the profile, while malformed browser storage cannot break startup.
function reloadFrame() {
  const reload = { ...sandbox, document: { ...sandbox.document, getElementById: id => element('reload-' + id) } };
  vm.createContext(reload); vm.runInContext(source + '\nthis.reloadedFrame=imuFrame;', reload);
  return reload.reloadedFrame;
}
const profile = storage.get('qav250.imu-mount.v1');
closeVector(reloadFrame().sensorToBody, a.imuFrame.sensorToBody);
storage.set('qav250.imu-mount.v1', '{broken'); assert.equal(reloadFrame().mountSource, 'photo_usb');
storage.set('qav250.imu-mount.v1', profile);
closeVector(a.state.display.rpy, [0, 30, 0]); closeVector(a.state.history[0].rpy, [0, 0, 0]);
assert(a.state.history.every(p => p.coordinateFrame === 'body_calibrated'));
holdPose([0, 0, 0]); a.updateUI(clock);
assert.equal(element('pitch').textContent, '0.0°');
close(a.state.display.a[0], 0); close(a.state.display.a[1], 0); close(a.state.display.a[2], 1);
assert.equal(element('packet-frame').textContent, 'BODY');

// An actual calibrated pitch must move the nose down, and feed the same matrix/graphs/CSV.
holdPose([0, 30, 0]); a.updateUI(clock); a.drawScene(clock); a.drawCharts();
assert.equal(element('pitch').textContent, '30.0°');
close(Number(element('matrix').children[6].textContent), -.5);
close(Number(element('a0').textContent), -.5);
assert(element('scene').context.calls.some(c => c[0] === 'fillText' && c[1] === 'FRONT'));
element('export').onclick(); const calibratedExport = exportedBlob;

// Heading zero is a rotation of world around Z; it must not flatten roll/pitch or rotate sensors.
holdPose([12, -20, 45]); const preZeroA = plain(a.state.display.a), preZeroG = plain(a.state.display.g);
element('zero-heading').onclick();
closeVector(a.state.display.rpy, [12, -20, 0]); closeVector(a.state.display.a, preZeroA); closeVector(a.state.display.g, preZeroG);
const storedMount = plain(a.imuFrame.sensorToBody);
a.resetBrowserView(); closeVector(a.imuFrame.sensorToBody, storedMount);

// Raw AHRS remains in native axes; the correction applies once after filtering.
a.setMode('serial'); a.state.port = {};
holdPose([0, 0, 0], { firmware: false });
for (let i = 0; i < 6; i++) holdPose([0, 30, 0], { firmware: false });
closeVector(a.state.display.rpy, [0, 30, 0], .01);
assert(a.state.display.estimated); closeVector(a.state.display.a, [-.5, 0, Math.sqrt(.75)]);

// Reject stale/moving calibration; demo and logs cannot overwrite a physical mounting profile.
holdPose([0, 30, 0], { moving: true }); assert.throws(() => a.captureStablePose(), /движется/);
clock += 2000; assert.throws(() => a.captureStablePose(), /свежих/);
a.setMode('demo');
const demo = a.normalizePacket({ q: a.quatEuler([3, 6, 9]), a_g: [.1, .2, .97] }); a.receive(demo);
closeVector(demo.rpy, [3, 6, 9]); closeVector(demo.a, [.1, .2, .97]);
assert.throws(() => a.captureStablePose(), /USB/); assert(a.imuFrame.calibrated);
a.setMode('log'); assert.throws(() => a.captureStablePose(), /USB/);

// Missing signals remain unknown after calibration, even with no orientation to display.
a.setMode('serial'); a.state.port = {};
a.receive(a.normalizePacket({ g_dps: a.mtv(a.rotation(tiltedMount), [1, 2, 3]) }));
assert.equal(a.state.display.q, null); closeVector(a.state.display.g, [1, 2, 3]);
assert.deepEqual(plain(a.state.display.a), [null, null, null]);
element('calibrate-reset').onclick(); assert(!a.imuFrame.calibrated); assert.equal(JSON.parse(storage.get('qav250.imu-mount.v1')).source, 'native');
assert(!reloadFrame().calibrated);
closeVector(a.state.display.g, a.mtv(a.rotation(tiltedMount), [1, 2, 3]));

// Photo profile: forward = toward USB (-IMU Y), left = IMU X, up = IMU Z.
storage.delete('qav250.imu-mount.v1');
const photoFrame = reloadFrame(); assert.equal(photoFrame.mountSource, 'photo_usb');
closeVector(a.mv(a.rotation(photoFrame.sensorToBody), [0, -1, 0]), [1, 0, 0]);
closeVector(a.mv(a.rotation(photoFrame.sensorToBody), [1, 0, 0]), [0, 1, 0]);
closeVector(a.mv(a.rotation(photoFrame.sensorToBody), [0, 0, 1]), [0, 0, 1]);

// Distinct magnetometer frame must be handled BEFORE either browser tilt/heading fusion.
a.magAxes.legacy = 'photo';
const magnetic = { a_g: [0, 0, 1], m_uT: [13, -27, 42] };
let magneticPacket = a.normalizePacket(magnetic);
closeVector(magneticPacket.m, [-13, -27, -42]);
assert.equal(magneticPacket.magMapping, 'photo_y180');
closeVector(magnetic.m_uT, [13, -27, 42]); // untouched source for replays/diagnostics
closeVector(a.normalizePacket({ ...magnetic, mag_frame: 'bmi270' }).m, [13, -27, 42]); // not twice
a.magAxes.legacy = 'aligned';
closeVector(a.normalizePacket({ ...magnetic, mag_frame: 'bmm150' }).m, [-13, -27, -42]);
closeVector(a.normalizePacket(magnetic).m, [13, -27, 42]);
assert.equal(a.normalizePacket({ m_uT: [13, -27, 42], mag_frame: 'unknown' }), null);
assert.deepEqual(plain(a.normalizePacket({ ...magnetic, mag_frame: 'unknown' }).m), [null, null, null]);
a.magAxes.legacy = 'photo';
assert(a.normalizePacket({ ...magnetic, q: [1, 0, 0, 0] }).warnings.some(s => s.includes('MEKF')));
assert.equal(a.normalizePacket({ ...magnetic, q: [1, 0, 0, 0], mag_frame: 'bmi270' }).warnings.length, 0);

// End-to-end: same physical board motion through old raw BMM axes and updated firmware q.
element('calibrate-photo').onclick();
assert.equal(a.imuFrame.mountSource, 'photo_usb');
assert.equal(JSON.parse(storage.get('qav250.imu-mount.v1')).source, 'photo_usb');
for (const firmware of [false, true]) {
  a.setMode('serial'); a.state.port = {};
  for (const rpy of [[0, 0, 0], [0, 30, 0], [0, -30, 0], [25, 0, 0], [-25, 0, 0], [0, 0, 45], [15, -20, 55]]) {
    for (let i = 0; i < 100; i++) {
      const input = sensorPacket(a.photoMountQ, rpy, 37, firmware);
      if (!firmware) { // Old firmware emits native BMM axes, without a frame tag.
        delete input.mag_frame; delete input.bias_dps;
        input.m_uT = [-input.m_uT[0], input.m_uT[1], -input.m_uT[2]];
      }
      clock += 100; a.receive(a.normalizePacket(input), { hostNow: clock });
    }
    closeVector(a.state.display.rpy, rpy, .01);
    closeVector(a.mv(a.rotation(a.state.display.q), a.state.display.a), [0, 0, 1], .001);
    closeVector(a.mv(a.rotation(a.state.display.q), a.state.display.m), [25, 0, -40], .01);
    assert.equal(a.state.display.magMapping, firmware ? 'identity' : 'photo_y180');
    assert.equal(a.state.display.coordinateFrame, 'body_photo');
  }
}
a.updateUI(clock); assert.equal(element('frame-status').textContent, 'По фото · вперёд к USB');
assert.match(element('mag-alignment-note').textContent, /повторное преобразование отключено/);
const demoPhoto = a.demoPacket(0); closeVector(demoPhoto.m, a.mtv(a.rotation(a.quatEuler([0, 10 * Math.sin(.4), 0])), [24, 0, -39]));
assert.equal(demoPhoto.magMapping, 'identity');

// BMP3xx scalar fields must survive raw/MEKF, mounting, pause, CSV and log paths.
const baroFields = { pressure_pa: 100025, temperature_c: 25, baro_valid: true, baro_age_ms: 1, baro_model: 'BMP390', baro_address: 119 };
for (const firmware of [false, true]) {
  a.setMode('serial'); a.state.port = {};
  for (let i = 0; i < 20; i++) {
    clock += 100;
    a.receive(a.normalizePacket({ ...level, ...(firmware ? { q: [1, 0, 0, 0] } : {}), ...baroFields, pressure_pa: 100025 + i, t_us: i * 100000, seq: i }), { hostNow: clock });
  }
  a.updateUI(clock); a.drawCharts();
  assert.equal(element('chart-value-3').textContent, '1000.44 hPa');
  assert.equal(element('chart-value-4').textContent, '25.00 °C');
  assert.match(element('baro-status').textContent, /BMP390 · ДАННЫЕ/);
  assert.match(element('baro-note').textContent, /0x77/);
  for (const mode of ['sensors', 'attitude']) {
    a.setCharts(mode);
    for (const id of ['chart-3', 'chart-4']) element(id).context.calls.length = 0;
    a.drawCharts();
    for (const id of ['chart-3', 'chart-4']) assert(element(id).context.calls.filter(c => c[0] === 'lineTo').length > 15);
  }
  // Mount and heading changes cannot rotate or otherwise alter scalar data.
  element('calibrate-photo').onclick(); a.updateUI(clock);
  assert.equal(a.state.display.pressurePa, 100044); assert.equal(a.state.display.temperatureC, 25);
  a.togglePause(); const pressureFrozen = a.state.display.pressurePa;
  clock += 100; a.receive(a.normalizePacket({ ...level, ...baroFields, pressure_pa: 99000 }), { hostNow: clock });
  a.updateUI(clock + 10000); assert.equal(a.state.display.pressurePa, pressureFrozen);
  assert.equal(element('chart-value-3').textContent, '1000.44 hPa');
  a.togglePause(); a.updateUI(clock); assert.equal(element('chart-value-3').textContent, '990.00 hPa');
  a.updateUI(clock + 501); assert.equal(element('chart-value-3').textContent, '—');
  assert.match(element('baro-status').textContent, /НЕТ СВЕЖИХ ДАННЫХ/);
  a.state.port = null; a.updateUI(clock); assert.equal(element('chart-value-4').textContent, '—');
}
for (const invalid of [null, '', true, false, 'NaN', Infinity, -1, 0, 29999, 125001]) {
  const p = a.normalizePacket({ ...level, ...baroFields, pressure_pa: invalid });
  assert.equal(p.pressurePa, null); assert.equal(p.temperatureC, 25);
}
for (const invalid of [null, '', true, 'NaN', Infinity, -41, 86]) {
  assert.equal(a.normalizePacket({ ...level, ...baroFields, temperature_c: invalid }).temperatureC, null);
}
for (const metadata of [{ baro_valid: false }, { baro_valid: 'bad' }, { baro_age_ms: 501 }, { baro_age_ms: -1 }, { baro_age_ms: 'bad' }]) {
  const p = a.normalizePacket({ ...level, ...baroFields, ...metadata });
  assert.equal(p.pressurePa, null); assert.equal(p.temperatureC, null);
}
assert.equal(a.normalizePacket({ ...level, pressure_pa: '100025', temperature_c: '0' }).temperatureC, 0);
assert.equal(a.normalizePacket({ ...level, ...baroFields, baro_age_ms: 500 }).pressurePa, 100025);
assert.equal(a.normalizePacket(baroFields).q, null); // pressure alone invents no orientation
assert.equal(a.normalizePacket(level).pressurePa, null);
assert.equal(a.normalizePacket(level).baroReported, false);
const baroParsed = [], baroErrors = [], baroParser = new a.TelemetryParser(p => baroParsed.push(p), e => baroErrors.push(e));
baroParser.feed(JSON.stringify({ ...level, ...baroFields }) + '\r\n');
baroParser.flush(); assert.equal(baroErrors.length, 0); assert.equal(baroParsed[0].pressurePa, 100025);
a.setMode('log'); a.receive(baroParsed[0], { hostNow: 0, logTime: 0 }); a.updateUI(clock + 100000);
assert.equal(element('chart-value-3').textContent, '1000.25 hPa'); // log is not live
assert.match(element('baro-status').textContent, /ЖУРНАЛ/);
element('export').onclick(); const baroExport = exportedBlob;
baroExport.text().then(csv => {
  const [header, row] = csv.trim().split('\n').map(line => line.split(','));
  assert.equal(header.length, row.length);
  assert.equal(row[header.indexOf('pressure_pa')], '100025');
  assert.equal(row[header.indexOf('temperature_c')], '25');
  assert.equal(row[header.indexOf('baro_valid')], '1');
  assert.equal(row[header.indexOf('baro_model')], 'BMP390');
}).catch(error => { console.error(error); process.exitCode = 1; });
// Optional real host-C adapter output: node tests/telemetry.cjs --barometer-exe <path>.
const baroExeIndex = process.argv.indexOf('--barometer-exe');
if (baroExeIndex >= 0) {
  const output = require('node:child_process').execFileSync(process.argv[baroExeIndex + 1], { encoding: 'utf8' });
  const payloads = output.split(/\r?\n/).filter(line => line.startsWith('{')).map(JSON.parse);
  assert.equal(payloads.length, 3);
  for (const payload of payloads) {
    const p = a.normalizePacket({ ...level, ...payload });
    assert.equal(p.pressurePa, payload.pressure_pa);
    assert.equal(p.temperatureC, payload.temperature_c);
    assert.equal(p.baroValid, payload.baro_valid);
  }
}
a.setMode('serial'); a.state.port = {};
a.receive(a.normalizePacket(level), { hostNow: clock }); a.updateUI(clock); a.drawCharts();
assert.equal(element('chart-value-3').textContent, '—');
assert.match(element('baro-note').textContent, /прошивку/);
assert.equal(a.demoPacket(1).baroValid, true);
assert(Number.isFinite(a.demoPacket(1).pressurePa));

calibratedExport.text().then(csv => {
  const rows = csv.trim().split('\n').map(row => row.split(',')), header = rows.shift(), last = rows.at(-1);
  assert.equal(last.length, header.length); assert.equal(last[header.indexOf('coordinate_frame')], 'body_calibrated');
  close(Number(last[header.indexOf('pitch_deg')]), 30); close(Number(last[header.indexOf('ax_g')]), -.5);
  assert(header.includes('mount_qw')); assert(header.includes('heading_zero_deg'));
  console.log(`PASS: ${expected} telemetry frames; 7 mounts and 10-degree skew, photo USB-forward profile, BMM->BMI alignment, raw/MEKF, BMP3xx Pa/hPa/C, scalar charts, stale/missing/errors, pause/log/CSV and optional C output.`);
}).catch(error => { console.error(error); process.exitCode = 1; });

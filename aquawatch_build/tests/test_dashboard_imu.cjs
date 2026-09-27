const fs = require('fs');
const path = require('path');
const vm = require('vm');
const assert = require('assert/strict');
const source = fs.readFileSync(path.join(__dirname, '../aquawatch_build.ino'), 'utf8');
const html = source.match(/R"HTML\(([\s\S]*?)\)HTML"/)[1];
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];
const nodes = {};
const element = () => ({textContent:'', innerHTML:'', style:{}, querySelector(){return this;}});
for (const match of html.matchAll(/id="([^"]+)"/g)) nodes[match[1]] = element();
const context = {
  document: {
    getElementById(id) { assert(nodes[id], `missing HTML element: ${id}`); return nodes[id]; },
    createElement: element
  },
  XMLHttpRequest: function() { this.open = this.send = () => {}; },
  setTimeout() {}
};
vm.createContext(context);
vm.runInContext(script, context);
const data = {
  uptime:'0:00:15', gps:{valid:false}, sim:{ready:false}, rotationX:45, rotationY:-30,
  imuStatus:'BMI160 detected; waiting for first sample',
  imu:{calibrated:false, calibrationSamples:0, rawAge:null, readErrors:0, lastReadError:0, gyro:[0,0,0], accel:[0,0,0]}
};
context.render(data);
assert.equal(nodes.rx.textContent, '-- °');
assert.equal(nodes.ry.textContent, '-- °');
assert(nodes.imudiagnostic.textContent.includes('Last successful sensor read: none'));
assert(!nodes.imudiagnostic.textContent.includes('Raw gyro'));
data.imuStatus = 'BMI160 receiving data; calibrating: movement detected; hold still';
Object.assign(data.imu, {calibrationSamples:12, rawAge:40, gyro:[.2,-.1,.3], accel:[1,0,0]});
context.render(data);
assert(nodes.sensorstatus.textContent.includes('receiving data; calibrating'));
assert(nodes.imudiagnostic.textContent.includes('1.000 / 0.000 / 0.000'));
assert(nodes.imudiagnostic.textContent.includes('Acceleration magnitude (g): 1.000'));
assert.equal(nodes.rx.textContent, '-- °');
data.imuStatus = 'BMI160 I2C read timeout; last error -1';
Object.assign(data.imu, {rawAge:5000, readErrors:3, lastReadError:-1});
context.render(data);
assert(nodes.sensorstatus.textContent.includes('I2C read timeout'));
assert(nodes.imudiagnostic.textContent.includes('5.0 s ago'));
assert(nodes.imudiagnostic.textContent.includes('Read errors: 3 | Last error code: -1'));
data.imuStatus = 'BMI160 receiving data; tilt ready';
Object.assign(data.imu, {calibrated:true, rawAge:0});
context.render(data);
assert.equal(nodes.rx.textContent, '45.0 °');
assert.equal(nodes.ry.textContent, '-30.0 °');
assert(nodes.imudiagnostic.textContent.includes('Tilt calibrated: yes'));
console.log('PASS: real dashboard script renders waiting, calibrating, raw vertical readings, read timeout, and calibrated angles.');

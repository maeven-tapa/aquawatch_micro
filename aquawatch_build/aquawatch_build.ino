/*
 * AquaWatch - NodeMCU (ESP8266)
 *
 * Pin map (GPIO numbers are used deliberately):
 * D0/GPIO16: active-high SOS button, external pulldown
 * D1/GPIO5: BMI160 SCL       D2/GPIO4: BMI160 SDA
 * D3/GPIO0: active-high strobe relay
 * D4/GPIO2: active-low siren relay
 * D5/GPIO14: NEO-8M TX       D6/GPIO12: NEO-8M RX
 * D7/GPIO13: SIM800L TX      D8/GPIO15: SIM800L RX
 *
 * Hold the button for 1.5 seconds to start SOS. Hold for 3 seconds
 * while active to stop SOS. SOS sends immediately and every 60 seconds.
 * The ESP8266 creates an open access point named "aquawatch_dev".
 *
 * Libraries: DFRobot_BMI160, TinyGPSPlus (install from Library Manager).
 */
#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <TinyGPSPlus.h>
#include <DFRobot_BMI160.h>
#include <math.h>
#include "aquawatch_logo.h"

// ----------------------------- Hardware -----------------------------
const uint8_t BUTTON_PIN = D0;
const uint8_t STROBE_RELAY_PIN = D3; // active HIGH
const uint8_t SIREN_RELAY_PIN = D4;  // active LOW
const uint8_t GPS_RX_PIN = D5;       // NEO-8M TX -> ESP RX
const uint8_t GPS_TX_PIN = D6;       // NEO-8M RX <- ESP TX
const uint8_t SIM_RX_PIN = D7;       // SIM800L TX -> ESP RX
const uint8_t SIM_TX_PIN = D8;       // SIM800L RX <- ESP TX

const uint8_t STROBE_ON = HIGH;
const uint8_t STROBE_OFF = LOW;
const uint8_t SIREN_ON = LOW;
const uint8_t SIREN_OFF = HIGH;

SoftwareSerial gpsSerial(GPS_RX_PIN, GPS_TX_PIN); // RX, TX
SoftwareSerial simSerial(SIM_RX_PIN, SIM_TX_PIN); // RX, TX
TinyGPSPlus gps;
DFRobot_BMI160 bmi160;
ESP8266WebServer server(80);

// ----------------------------- Configuration -----------------------------
String deviceName = "Spike";
const char *AP_NAME = "aquawatch_dev";
const char *AP_PASSWORD = nullptr; // open AP
String receiverNumber = "09925283361"; // compatibility alias for the first contact
const uint8_t MAX_CONTACTS = 6;
struct Contact { String name; String number; };
Contact contacts[MAX_CONTACTS] = {{"Primary", "09925283361"}};
uint8_t contactCount = 1;
const unsigned long START_HOLD_MS = 1500;
const unsigned long STOP_HOLD_MS = 3000;
const unsigned long SOS_REPEAT_MS = 60000;
const unsigned long STROBE_PERIOD_MS = 500;
const float GRAVITY = 9.80665f;
const float GYRO_SCALE = 16.4f; // BMI160 default +/-2000 dps

// ----------------------------- State -----------------------------
bool alarmActive = false;
bool buttonWasDown = false;
bool buttonActionTaken = false;
unsigned long buttonDownAt = 0;
unsigned long lastStrobeAt = 0;
unsigned long lastSosAt = 0;
bool strobeState = false;
bool bmiReady = false;
bool simReady = false;
String simModel = "Not checked";
String simIMEI = "Not checked";
String simNetwork = "Unknown";
String lastSosStatus = "Not sent";

float rotationX = 0.0f;
float rotationY = 0.0f;
float linearAcceleration = 0.0f;
float gyroOffsetX = 0.0f;
float gyroOffsetY = 0.0f;
unsigned long lastGyroMicros = 0;
unsigned long lastSensorAt = 0;
unsigned long lastSensorSuccess = 0;
bool tiltArmed = true;
const float AUTO_TRIGGER_DEG = 50.0f;
const float AUTO_REARM_DEG = 45.0f;
String gpsRaw;
unsigned long gpsRawAt = 0;
bool gpsReceived = false;
bool smsPending = false;
bool smsBusy = false;
bool signalPending = false;
unsigned long signalRequestAt = 0;
unsigned long signalUpdatedAt = 0;
bool signalReceived = false;
int signalRssi = 99;
String signalResponse;
String blastMessage;

// ----------------------------- HTML -----------------------------
const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>AquaWatch</title>
<style>
:root{color-scheme:dark;--bg:#07131f;--card:#102538;--line:#1f4760;--cyan:#54d8e8;--green:#44e39a;--red:#ff6678}
*{box-sizing:border-box}body{margin:0;background:linear-gradient(145deg,#06101b,#0b2233);font-family:system-ui,Arial;color:#eef9fc}
main{max-width:960px;margin:auto;padding:24px 16px 42px}header{display:flex;justify-content:space-between;gap:12px;align-items:end;margin-bottom:20px}h1{font-size:clamp(28px,6vw,52px);letter-spacing:.08em;margin:0}h2{font-size:16px;color:var(--cyan);margin:0 0 12px;text-transform:uppercase;letter-spacing:.12em}.muted{color:#91adba;font-size:13px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(190px,1fr));gap:12px}.card{background:rgba(16,37,56,.9);border:1px solid var(--line);border-radius:16px;padding:17px;box-shadow:0 8px 24px #0003}.value{font-size:27px;font-weight:700;margin-top:7px}.unit{font-size:13px;color:#9ab8c2;font-weight:400}.wide{grid-column:1/-1}.status{display:inline-flex;align-items:center;gap:8px;font-weight:700}.dot{width:11px;height:11px;border-radius:99px;background:var(--red);box-shadow:0 0 12px currentColor}.ok{color:var(--green)}.alarm{color:var(--red)}a{color:var(--cyan)}input{width:100%;padding:11px;border-radius:9px;border:1px solid var(--line);background:#071722;color:#fff;font-size:16px}button{padding:11px 15px;border:0;border-radius:9px;background:#176b83;color:#fff;font-weight:700;cursor:pointer;margin-top:8px}.map{font-size:18px;word-break:break-word}.bar{height:8px;background:#071722;border-radius:9px;overflow:hidden;margin-top:12px}.bar i{display:block;height:100%;width:0;background:linear-gradient(90deg,var(--cyan),var(--green));transition:width .3s}

:root{--orange:#ff863e;--line:#34414d}
body{background:radial-gradient(ellipse at top,#332013 0%,#07131f 48%)}
header{align-items:center;flex-wrap:wrap;padding-bottom:18px}
.brand{display:flex;align-items:center;gap:18px}.logo{display:block;width:240px;max-width:100%;height:auto;border:0;border-radius:0;background:transparent}
h2{color:#ffac77}button{background:var(--orange);color:#19120d}button:hover{background:#ffac77}a,summary{color:#ffac77}
button:focus-visible,summary:focus-visible{outline:3px solid #54d8e8;outline-offset:3px}
.card{min-width:0}.motion-grid{grid-column:1/-1;display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:12px}
.motion-grid .card{border-top:3px solid var(--orange)}.bar i{background:linear-gradient(90deg,#ff863e,#ffbf69)}
.contact-heading{display:flex;align-items:center;justify-content:space-between;gap:10px}.contact-heading h2{margin:0}
.add-contact{width:42px;height:42px;padding:0;margin:0;font-size:26px;line-height:1;border-radius:50%}
#contactform{margin-top:16px}#contactform[hidden]{display:none}#contactform label{display:block;margin:10px 0 4px;font-size:13px;color:#c2cfd5}
.dot{background:currentColor}summary{cursor:pointer;margin-top:14px}#contacts{overflow-wrap:anywhere}
@media(max-width:520px){main{padding:16px 10px 30px}.brand{width:100%;justify-content:center;flex-direction:column;gap:10px}.logo{width:220px}.brand .muted{text-align:center}header{justify-content:center}.motion-grid{gap:6px}.motion-grid .card{padding:12px 7px}.motion-grid h2{font-size:10px;letter-spacing:.03em;min-height:26px}.motion-grid .value{font-size:clamp(12px,3.7vw,20px);overflow-wrap:anywhere}.grid{gap:10px}}
</style></head><body><main><header><div class="brand"><img class="logo" src="/logo-transparent.png" width="600" height="400" alt="AquaWatch logo"><div class="muted">AquaWatch Access Control</div></div></header><div class="card" style="margin-bottom:16px"><div class="contact-heading"><div><span class="muted">DEVICE NAME</span><div id="devicename" class="value">Spike</div></div><button id="nameedit" type="button" aria-controls="nameform" aria-expanded="false" onclick="editDeviceName(true)">Edit</button></div><form id="nameform" hidden style="margin-top:12px"><label for="nameinput">Device name</label><input id="nameinput" name="name" maxlength="16" required><p class="muted">Up to 16 letters, numbers, spaces, dots, underscores or hyphens. Included in SOS texts.</p><button type="submit">Save</button> <button type="button" onclick="editDeviceName(false)">Cancel</button><p id="nameerror" role="status"></p></form></div>
<div id="alarm" class="status" role="status" aria-live="polite" style="margin:4px 0 16px"><i class="dot"></i><span>Alarm: waiting for device</span></div><p id="connection" hidden role="status" aria-live="polite">Connecting to AquaWatch...</p><p id="sensorstatus" class="muted"></p><section class="grid"><div class="motion-grid"><article class="card"><h2>Rotation X</h2><div id="rx" class="value">--<span class="unit"> °</span></div></article><article class="card"><h2>Rotation Y</h2><div id="ry" class="value">--<span class="unit"> °</span></div></article><article class="card"><h2>Linear acceleration</h2><div id="acc" class="value">--<span class="unit"> m/s²</span></div><div class="bar"><i id="accbar"></i></div></article></div><article class="card"><h2>Device uptime</h2><div id="up" class="value">--</div></article>
<article class="card wide"><h2>GPS location</h2><div id="gps" class="map">Waiting for NEO-8M fix…</div><div id="gpsmeta" class="muted"></div><details id="gpsdetails"><summary>View raw GPS data (NMEA)</summary><p id="gpsrawage" class="muted"></p><pre id="gpsraw" style="white-space:pre-wrap;overflow-wrap:anywhere;max-height:240px;overflow:auto">Waiting for GPS serial data...</pre></details></article>
<article class="card"><h2>SIM800L</h2><div id="sim" class="status"><i class="dot"></i><span>Unknown</span></div><p id="signal">Signal: checking...</p><p class="muted" id="siminfo">--</p><p id="smsstatus" class="muted"></p></article><article class="card"><div class="contact-heading"><h2>SMS contacts</h2><button class="add-contact" type="button" id="contacttoggle" aria-label="Add SMS contact" aria-controls="contactform" aria-expanded="false" onclick="toggleContactForm()">+</button></div><form id="contactform" action="/add-contact" method="get" hidden><label for="contactname">Person name</label><input id="contactname" name="name" placeholder="Person name" maxlength="24" required><label for="contactnumber">Phone number</label><input id="contactnumber" name="number" type="tel" placeholder="Contact number" maxlength="20" required><button>Add contact</button></form><div id="contacts" class="muted">Loading contacts…</div><p class="muted">Every saved contact receives the SOS blast.</p></article>
<article class="card"><h2>Alarm control</h2><button onclick="fetch('/alarm/on')">Trigger SOS</button> <button onclick="fetch('/alarm/off')">Cancel SOS</button><p class="muted">Physical button: hold 1.5 s to start, 3 s to stop. Automatic SOS at X or Y rotation of +50° or -50°. After cancellation, both axes must return within ±45° to re-arm.</p></article></section>
<p class="muted" style="margin-top:18px">Refreshes every second · Wi-Fi: aquawatch_dev · http://192.168.4.1</p></main>
<script>
var $=function(id){return document.getElementById(id);};
function toggleContactForm(){
  var form=$('contactform'),button=$('contacttoggle');
  form.hidden=!form.hidden;button.setAttribute('aria-expanded',String(!form.hidden));
  button.setAttribute('aria-label',form.hidden?'Add SMS contact':'Close contact form');
  button.textContent=form.hidden?'+':'−';if(!form.hidden)$('contactname').focus();
}
function editDeviceName(open){
  $('nameform').hidden=!open;$('nameedit').setAttribute('aria-expanded',String(open));$('nameerror').textContent='';
  if(open){$('nameinput').value=$('devicename').textContent;$('nameinput').focus();}
}
function saveDeviceName(event){
  event.preventDefault();var name=$('nameinput').value.trim();
  if(!/^[A-Za-z0-9 ._-]{1,16}$/.test(name)){$('nameerror').textContent='Enter a valid name (1–16 characters).';return;}
  var xhr=new XMLHttpRequest();xhr.open('POST','/device-name',true);xhr.timeout=5000;
  xhr.setRequestHeader('Content-Type','application/x-www-form-urlencoded');
  xhr.onload=function(){if(xhr.status===200){$('devicename').textContent=name;editDeviceName(false);}else{$('nameerror').textContent=xhr.responseText||'Save failed. Try again.';}};
  xhr.onerror=xhr.ontimeout=function(){$('nameerror').textContent='Could not confirm save. Check the connection and retry.';};
  xhr.send('name='+encodeURIComponent(name));
}
$('nameform').onsubmit=saveDeviceName;
function fmt(n,d){return typeof n==='number' && isFinite(n)?n.toFixed(d===undefined?1:d):'--';}
function esc(s){var el=document.createElement('span');el.textContent=String(s);return el.innerHTML;}
function render(d){
  if(!d || typeof d.uptime!=='string' || !d.gps || !d.sim)throw new Error('Incomplete telemetry response');
  $('up').textContent=d.uptime;
  if(d.deviceName)$('devicename').textContent=d.deviceName;
  $('rx').textContent=fmt(d.rotationX)+' °';
  $('ry').textContent=fmt(d.rotationY)+' °';
  $('acc').textContent=fmt(d.acceleration,2)+' m/s²';
  $('accbar').style.width=Math.min(100,Math.max(0,Number(d.acceleration)||0)*10)+'%';
  $('signal').textContent='Signal: '+(d.signal||'Waiting for modem');
  $('smsstatus').textContent=d.smsStatus||'';
  $('sensorstatus').textContent=(d.imuStatus||'')+' · GPS received: '+(d.gpsChars||0)+' characters';
  $('gpsraw').textContent=d.gpsRaw||'No serial data received from GPS yet.';
  $('gpsrawage').textContent=d.gpsRawAge==null?'Waiting for NEO-8M data':'Last received '+(d.gpsRawAge/1000).toFixed(1)+' s ago';
  $('alarm').className='status '+(d.alarm?'alarm':'ok');
  $('alarm').querySelector('span').textContent=d.alarm?'Alarm: ACTIVE':'Alarm: OFF';
  if(d.gps.valid){
    var lat=Number(d.gps.lat),lng=Number(d.gps.lng);
    $('gps').innerHTML=lat.toFixed(6)+', '+lng.toFixed(6)+' · <a target="_blank" rel="noopener" href="https://maps.google.com/?q='+lat+','+lng+'">Open map</a>';
  }else{$('gps').textContent=d.gpsChars?'GPS receiving data; waiting for position fix':'No GPS data received yet';}
  $('gpsmeta').textContent='Satellites: '+(d.gps.sats||0)+(d.gps.valid?' · Fix age: '+d.gps.age+' ms':'');
  $('sim').className='status '+(d.sim.ready?'ok':'alarm');
  $('sim').querySelector('span').textContent=d.sim.status||'Modem not responding';
  $('siminfo').textContent=(d.sim.model||'Unknown model')+' · IMEI '+(d.sim.imei||'Unknown');
  $('contacts').innerHTML=(d.contacts||[]).map(function(c,i){return '<div style="margin:8px 0"><b>'+esc(c.name)+'</b><br>'+esc(c.number)+' <a href="/remove-contact?i='+i+'">remove</a></div>';}).join('');
  $('connection').textContent=(d.firmware||'AquaWatch')+' · Live · '+new Date().toLocaleTimeString();
  $('connection').className='ok';$('connection').hidden=true;
}
function update(){
  // One request at a time, with a bounded timeout and a visible failure reason.
  var xhr=new XMLHttpRequest();
  xhr.open('GET','/api?t='+Date.now(),true);xhr.timeout=5000;
  function failed(reason){$('connection').hidden=false;$('connection').textContent='Live update failed: '+reason+'. Reconnect to aquawatch_dev; retrying...';$('connection').className='alarm';}
  xhr.onload=function(){
    try{if(xhr.status!==200)throw new Error('HTTP '+xhr.status);render(JSON.parse(xhr.responseText));}
    catch(e){failed(e.message);}
  };
  xhr.onerror=function(){failed('device connection lost');};
  xhr.ontimeout=function(){failed('device did not respond within 5 seconds');};
  xhr.onloadend=function(){setTimeout(update,1000);};
  xhr.send();
}
update();
</script></body></html>
)HTML";

// ----------------------------- Contact storage -----------------------------
const int EEPROM_BYTES = 512;
const int CONTACT_SLOT_BYTES = 44;
void writeContactText(int address, const String &value, int width) { for (int i = 0; i < width; i++) EEPROM.write(address + i, i < value.length() ? value[i] : 0); }
String readContactText(int address, int width) { char buffer[45]; int n = 0; for (; n < width && EEPROM.read(address + n); n++) buffer[n] = EEPROM.read(address + n); buffer[n] = 0; return String(buffer); }
void saveContacts() { EEPROM.write(0, contactCount); for (uint8_t i = 0; i < MAX_CONTACTS; i++) { writeContactText(1 + i * CONTACT_SLOT_BYTES, contacts[i].name, 24); writeContactText(1 + i * CONTACT_SLOT_BYTES + 24, contacts[i].number, 20); } EEPROM.commit(); }
void loadContacts() { uint8_t saved = EEPROM.read(0); if (saved > 0 && saved <= MAX_CONTACTS) { contactCount = saved; for (uint8_t i = 0; i < contactCount; i++) { contacts[i].name = readContactText(1 + i * CONTACT_SLOT_BYTES, 24); contacts[i].number = readContactText(1 + i * CONTACT_SLOT_BYTES + 24, 20); } } receiverNumber = contacts[0].number; }
// Device name occupies a separate EEPROM area; existing contacts stay intact.
const int DEVICE_NAME_MARKER = 300;
const int DEVICE_NAME_LENGTH = 16;
bool validDeviceName(const String &name) {
  if (!name.length() || name.length() > DEVICE_NAME_LENGTH) return false;
  for (unsigned int i=0; i<name.length(); i++) {
    char c=name[i];
    if (!((c>='A' && c<='Z') || (c>='a' && c<='z') || (c>='0' && c<='9') || c==' ' || c=='.' || c=='_' || c=='-')) return false;
  }
  return true;
}
void loadDeviceName() {
  if (EEPROM.read(DEVICE_NAME_MARKER)!=0xA7 || EEPROM.read(DEVICE_NAME_MARKER+1)!=0x51) return;
  String saved=readContactText(DEVICE_NAME_MARKER+2,DEVICE_NAME_LENGTH); saved.trim();
  if (validDeviceName(saved)) deviceName=saved;
}
void handleDeviceName() {
  String name=server.arg("name"); name.trim();
  if (!validDeviceName(name)) { server.send(400,"text/plain","Use 1-16 letters, numbers, spaces, dots, underscores or hyphens."); return; }
  if (name!=deviceName) {
    EEPROM.write(DEVICE_NAME_MARKER,0xA7); EEPROM.write(DEVICE_NAME_MARKER+1,0x51);
    writeContactText(DEVICE_NAME_MARKER+2,name,DEVICE_NAME_LENGTH);
    if (!EEPROM.commit()) { server.send(500,"text/plain","Unable to save device name. Try again."); return; }
    deviceName=name;
  }
  server.send(200,"text/plain","Saved");
}
// ----------------------------- Utility -----------------------------
String jsonEscape(const String &s) {
  // Escape every JSON control byte, including unsolicited modem/EEPROM data.
  String out; out.reserve(s.length() + 16);
  for (unsigned int i = 0; i < s.length(); i++) {
    uint8_t c = static_cast<uint8_t>(s[i]);
    if (c == '"' || c == '\\') { out += '\\'; out += char(c); }
    else if (c < 0x20) { char escaped[7]; snprintf(escaped, sizeof(escaped), "\\u%04x", c); out += escaped; }
    else out += char(c);
  }
  return out;
}
String uptimeText() { unsigned long s = millis() / 1000; char b[24]; snprintf(b, sizeof(b), "%lu:%02lu:%02lu", s / 3600, (s / 60) % 60, s % 60); return String(b); }
void feedGPS() {
  while (gpsSerial.available()) {
    char c = gpsSerial.read(); gps.encode(c);
    if ((c >= 32 && c <= 126) || c == '\n') {
      if (gpsRaw.length() >= 1536) gpsRaw.remove(0, 256);
      gpsRaw += c; gpsRawAt = millis(); gpsReceived = true;
    }
  }
}
String gpsText() { return gps.location.isValid() ? String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6) : "No fix"; }

// ----------------------------- BMI160 -----------------------------
void calibrateGyro() {
  int16_t data[6]; float sx = 0, sy = 0; int n = 0;
  for (int i = 0; i < 150; i++) { if (bmi160.getAccelGyroData(data) == BMI160_OK) { sx += -(data[2] / GYRO_SCALE); sy += data[1] / GYRO_SCALE; n++; } delay(5); }
  if (n) { gyroOffsetX = sx / n; gyroOffsetY = sy / n; }
  lastGyroMicros = micros();
}
void updateSensor() {
  if (!bmiReady || millis() - lastSensorAt < 40) return;
  lastSensorAt = millis(); int16_t data[6]; if (bmi160.getAccelGyroData(data) != BMI160_OK) return;
  lastSensorSuccess = millis();
  float gx = -(data[2] / GYRO_SCALE) - gyroOffsetX, gy = (data[1] / GYRO_SCALE) - gyroOffsetY;
  if (fabs(gx) < .5) gx = 0; if (fabs(gy) < .5) gy = 0;
  unsigned long now = micros(); float dt = (now - lastGyroMicros) / 1000000.0f; lastGyroMicros = now;
  if (dt > 0 && dt <= .1) { rotationX += gx * dt; rotationY += gy * dt; }
  if (rotationX >= 360) rotationX -= 360; if (rotationX < -360) rotationX += 360;
  if (rotationY >= 360) rotationY -= 360; if (rotationY < -360) rotationY += 360;
  float ax = data[3] / 16384.0f * GRAVITY, ay = data[4] / 16384.0f * GRAVITY, az = data[5] / 16384.0f * GRAVITY;
  float total = sqrt(ax*ax + ay*ay + az*az); linearAcceleration = 0;
  if (total >= .1) { float lx = ax - ax / total * GRAVITY, ly = ay - ay / total * GRAVITY, lz = az - az / total * GRAVITY; linearAcceleration = sqrt(lx*lx + ly*ly + lz*lz); if (linearAcceleration < .20) linearAcceleration = 0; }
}
void setupBMI() {
  Wire.begin(D2, D1); int8_t result = bmi160.I2cInit(0x69); if (result != BMI160_OK) result = bmi160.I2cInit(0x68);
  bmiReady = result == BMI160_OK; if (bmiReady) { calibrateGyro(); Serial.println(F("BMI160 ready")); } else Serial.println(F("BMI160 not detected"));
}

// ----------------------------- SIM800L -----------------------------
void serviceStrobe();
void updateAlarm();
void serviceLive() {
  feedGPS(); updateSensor(); updateAlarm(); server.handleClient(); yield();
}
String atCommand(const String &cmd, unsigned long timeout) {
  simSerial.listen(); while (simSerial.available()) simSerial.read(); simSerial.println(cmd); String out; unsigned long start = millis();
  while (millis() - start < timeout) { if (smsBusy) serviceLive(); else serviceStrobe(); while (simSerial.available()) { char c = simSerial.read(); if (out.length() < 512) out += c; } yield(); }
  return out;
}
void setupSIM() {
  simSerial.begin(9600); delay(300);
  atCommand("AT", 1200); atCommand("ATE0", 800); atCommand("AT+CMGF=1", 1000);
  String model = atCommand("AT+CGMM", 1500), imei = atCommand("AT+GSN", 1500);
  model.replace("\r", ""); model.replace("\n", " "); model.replace("OK", ""); model.trim();
  imei.replace("\r", ""); imei.replace("\n", " "); imei.replace("OK", ""); imei.trim();
  simModel = model.length() ? model : "Unknown"; simIMEI = imei.length() ? imei : "Unknown";
  simReady = atCommand("AT+CPIN?", 1500).indexOf("READY") >= 0;
  simNetwork = simReady ? "SIM ready" : "SIM not ready";
}
// SIM800 AT+CSQ: 99 means unknown. Endpoints are bounds, not exact readings.
String signalText() {
  if (!signalReceived || millis() - signalUpdatedAt > 30000) return "Unavailable / stale";
  if (signalRssi == 99) return "Unknown (CSQ 99)";
  if (signalRssi == 0) return "<= -115 dBm (CSQ 0)";
  if (signalRssi == 31) return ">= -52 dBm (CSQ 31)";
  int dbm = signalRssi == 1 ? -111 : -114 + 2 * signalRssi;
  return String(dbm) + " dBm (CSQ " + String(signalRssi) + ")";
}
void updateSignal() {
  if (smsBusy) return;
  if (!signalPending) {
    if (millis() - signalRequestAt < 10000) return;
    while (simSerial.available()) simSerial.read();
    signalResponse = ""; signalRequestAt = millis(); signalPending = true;
    simSerial.println("AT+CSQ");
  }
  while (simSerial.available()) {
    char c = simSerial.read(); if (signalResponse.length() < 256) signalResponse += c;
  }
  bool done = signalResponse.indexOf("\nOK") >= 0 || signalResponse.indexOf("ERROR") >= 0;
  if (!done && millis() - signalRequestAt < 1500) return;
  signalRssi = 99; signalReceived = false;
  int pos = signalResponse.indexOf("+CSQ:"); int value = -1, ber = -1;
  if (pos >= 0 && sscanf(signalResponse.c_str() + pos, "+CSQ: %d,%d", &value, &ber) == 2 &&
      ((value >= 0 && value <= 31) || value == 99)) {
    signalRssi = value; signalReceived = true; signalUpdatedAt = millis();
  }
  signalPending = false;
}
String smsData() {
  String msg = "AQUAWATCH DISTRESS MESSAGE\nDevice: " + deviceName + "\nlat; long: ";
  msg += gps.location.isValid() && gps.location.age() < 10000 ? gpsText() : "No fresh fix";
  msg += "\nGyro X: "; msg += String(rotationX, 1); msg += " deg Y: "; msg += String(rotationY, 1); msg += " deg";
  msg += "\nAcceleration: "; msg += String(linearAcceleration, 2); msg += " m/s2";
  return msg;
}
bool sendSmsTo(const String &number) {
  if (!number.length() || !alarmActive) return false;
  while (simSerial.available()) simSerial.read();
  simSerial.print("AT+CMGS=\""); simSerial.print(number); simSerial.println("\"");
  unsigned long start = millis(); String prompt;
  while (millis() - start < 5000) {
    serviceLive();
    while (simSerial.available()) { char c = simSerial.read(); if (prompt.length() < 256) prompt += c; }
    if (!alarmActive) { simSerial.write(27); return false; }
    if (prompt.indexOf('>') >= 0 || prompt.indexOf("ERROR") >= 0) break;
  }
  if (prompt.indexOf('>') < 0) { simSerial.write(27); return false; }
  simSerial.print(blastMessage); simSerial.write(26);
  // Submitted SMS cannot be recalled. Finish its response before the next AT command.
  String result; start = millis();
  while (millis() - start < 60000) {
    serviceLive();
    while (simSerial.available()) { char c = simSerial.read(); if (result.length() < 512) result += c; }
    if (result.indexOf("ERROR") >= 0) return false;
    if (result.indexOf("+CMGS:") >= 0 && result.indexOf("\nOK") >= 0) return true;
  }
  return false;
}
void sendSOS() {
  if (!contactCount) { lastSosStatus = "No SMS contacts"; return; }
  // Snapshot recipients and sensor values once for this blast.
  String numbers[MAX_CONTACTS]; uint8_t total = contactCount;
  for (uint8_t i = 0; i < total; i++) numbers[i] = contacts[i].number;
  blastMessage = smsData(); smsBusy = true;
  String reg = atCommand("AT+CREG?", 1500);
  simReady = reg.indexOf(",1") >= 0 || reg.indexOf(",5") >= 0;
  simNetwork = simReady ? "Network registered" : "Network not registered";
  if (!simReady) { lastSosStatus = "GSM unavailable"; smsBusy = false; return; }
  uint8_t sent = 0;
  for (uint8_t i = 0; i < total && alarmActive; i++) {
    lastSosStatus = "Sending " + String(i + 1) + "/" + String(total);
    if (sendSmsTo(numbers[i])) sent++;
  }
  lastSosStatus = String(sent) + "/" + String(total) + " accepted by modem" + (alarmActive ? "" : " (cancelled)");
  smsBusy = false;
}
// ----------------------------- Alarm -----------------------------
void setAlarm(bool on) {
  if (on == alarmActive) return;
  alarmActive = on;
  if (on) {
    digitalWrite(SIREN_RELAY_PIN, SIREN_ON); strobeState = true;
    digitalWrite(STROBE_RELAY_PIN, STROBE_ON); lastStrobeAt = millis(); smsPending = true;
  } else {
    digitalWrite(SIREN_RELAY_PIN, SIREN_OFF); digitalWrite(STROBE_RELAY_PIN, STROBE_OFF);
    strobeState = false; smsPending = false; tiltArmed = false;
  }
}
void serviceStrobe() {
  if (alarmActive && millis() - lastStrobeAt >= STROBE_PERIOD_MS) {
    strobeState = !strobeState; digitalWrite(STROBE_RELAY_PIN, strobeState ? STROBE_ON : STROBE_OFF); lastStrobeAt = millis();
  }
}
void updateAlarm() {
  bool down = digitalRead(BUTTON_PIN) == HIGH; unsigned long now = millis();
  if (down && !buttonWasDown) { buttonDownAt = now; buttonActionTaken = false; }
  if (down && !buttonActionTaken && now - buttonDownAt >= (alarmActive ? STOP_HOLD_MS : START_HOLD_MS)) {
    setAlarm(!alarmActive); buttonActionTaken = true;
  }
  if (!down) { buttonDownAt = 0; buttonActionTaken = false; }
  buttonWasDown = down;
  if (bmiReady && lastSensorSuccess && now - lastSensorSuccess < 250) {
    if (fabs(rotationX) < AUTO_REARM_DEG && fabs(rotationY) < AUTO_REARM_DEG) tiltArmed = true;
    if (tiltArmed && (fabs(rotationX) >= AUTO_TRIGGER_DEG || fabs(rotationY) >= AUTO_TRIGGER_DEG)) {
      tiltArmed = false; setAlarm(true);
    }
  }
  serviceStrobe();
}

// ----------------------------- Web API -----------------------------
void handleApi() {
  // Do not let a cached response or a bad peripheral value freeze the dashboard.
  server.sendHeader("Cache-Control", "no-store, max-age=0");
  String out = "{\"rotationX\":" + String(rotationX, 2) + ",\"rotationY\":" + String(rotationY, 2) + ",\"acceleration\":" + String(linearAcceleration, 2) + ",\"uptime\":\"" + uptimeText() + "\",\"alarm\":" + String(alarmActive ? "true" : "false") + ",\"number\":\"" + jsonEscape(receiverNumber) + "\",\"gps\":{";
  out += "\"valid\":" + String(gps.location.isValid() ? "true" : "false") + ",\"lat\":" + String(gps.location.lat(), 6) + ",\"lng\":" + String(gps.location.lng(), 6) + ",\"sats\":" + String(gps.satellites.isValid() ? gps.satellites.value() : 0) + ",\"age\":" + String(gps.location.isValid() ? gps.location.age() : 0) + "},\"sim\":{\"ready\":" + String(simReady ? "true" : "false") + ",\"status\":\"" + jsonEscape(simNetwork) + "\",\"model\":\"" + jsonEscape(simModel) + "\",\"imei\":\"" + jsonEscape(simIMEI) + "\"}}";
  out.remove(out.length() - 1); out += ",\"contacts\":[";
  for (uint8_t i = 0; i < contactCount; i++) { if (i) out += ","; out += "{\"name\":\"" + jsonEscape(contacts[i].name) + "\",\"number\":\"" + jsonEscape(contacts[i].number) + "\"}"; }
  out += "],\"signal\":\"" + jsonEscape(signalText()) + "\",\"smsStatus\":\"" + jsonEscape(lastSosStatus) + "\",\"gpsRaw\":\"" + jsonEscape(gpsRaw) + "\",\"gpsRawAge\":" + (gpsReceived ? String(millis() - gpsRawAt) : String("null"));
  out += ",\"imuStatus\":\"" + String(!bmiReady ? "BMI160 not detected (check I2C wiring/address)" : (!lastSensorSuccess || millis() - lastSensorSuccess > 1000) ? "BMI160 read timeout" : "BMI160 receiving data") + "\",\"gpsChars\":" + String(gps.charsProcessed()) + ",\"firmware\":\"AquaWatch 1.3.1\"}";
  out.remove(out.length()-1); out += ",\"deviceName\":\"" + jsonEscape(deviceName) + "\"}";
  server.send(200, "application/json", out);
}
void handleSetNumber() { if (server.hasArg("number")) { String n = server.arg("number"); n.trim(); if (n.length() >= 5 && n.length() <= 20) { receiverNumber = n; contacts[0].number = n; saveContacts(); } } server.sendHeader("Location", "/"); server.send(303); }
void handleAddContact() { String name = server.arg("name"); String number = server.arg("number"); name.trim(); number.trim(); if (contactCount < MAX_CONTACTS && name.length() && number.length() >= 5 && number.length() <= 20) { contacts[contactCount].name = name; contacts[contactCount].number = number; contactCount++; saveContacts(); } receiverNumber = contacts[0].number; server.sendHeader("Location", "/"); server.send(303); }
void handleRemoveContact() { int index = server.hasArg("i") ? server.arg("i").toInt() : -1; if (index >= 0 && index < contactCount && contactCount > 1) { for (uint8_t i = index; i + 1 < contactCount; i++) contacts[i] = contacts[i + 1]; contactCount--; saveContacts(); } receiverNumber = contacts[0].number; server.sendHeader("Location", "/"); server.send(303); }void setupWeb() { WiFi.mode(WIFI_AP); WiFi.softAP(AP_NAME); server.on("/", [](){ server.sendHeader("Cache-Control", "no-store, max-age=0"); server.send_P(200, "text/html", INDEX_HTML); }); server.on("/logo-transparent.png", [](){ server.sendHeader("Cache-Control", "public, max-age=86400"); server.send_P(200, "image/png", reinterpret_cast<const char *>(AQUAWATCH_LOGO), sizeof(AQUAWATCH_LOGO)); }); server.on("/api", handleApi); server.on("/device-name", HTTP_POST, handleDeviceName); server.on("/set-number", handleSetNumber); server.on("/add-contact", handleAddContact); server.on("/remove-contact", handleRemoveContact); server.on("/alarm/on", [](){ setAlarm(true); server.send(200, "text/plain", "ON"); }); server.on("/alarm/off", [](){ setAlarm(false); server.send(200, "text/plain", "OFF"); }); server.begin(); }

void setup() { Serial.begin(115200); delay(100); EEPROM.begin(EEPROM_BYTES); loadContacts(); loadDeviceName(); pinMode(BUTTON_PIN, INPUT_PULLDOWN_16); pinMode(STROBE_RELAY_PIN, OUTPUT); pinMode(SIREN_RELAY_PIN, OUTPUT); digitalWrite(STROBE_RELAY_PIN, STROBE_OFF); digitalWrite(SIREN_RELAY_PIN, SIREN_OFF); gpsSerial.begin(9600); setupBMI(); setupSIM(); setupWeb(); Serial.println(F("AquaWatch ready at http://192.168.4.1")); }
void loop() {
  serviceLive(); updateSignal();
  if (alarmActive && !smsBusy && !signalPending && (smsPending || millis() - lastSosAt >= SOS_REPEAT_MS)) {
    smsPending = false; lastSosAt = millis(); sendSOS();
  }
}

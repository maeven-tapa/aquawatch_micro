/*
 * AquaWatch - ESP32 NodeMCU (38-pin, ESP32-WROOM)
 *
 * Pin map (GPIO numbers are used deliberately):
 * GPIO25: active-high SOS button, pulldown (button connects to 3.3 V)
 * GPIO22: BMI160 SCL        GPIO23: BMI160 SDA
 * GPIO4: active-high strobe relay
 * GPIO27: active-low siren relay
 * GPIO18: NEO-M8U TX -> ESP RX   GPIO19: NEO-M8U RX <- ESP TX
 * GPIO16: SIM800L TX -> ESP RX   GPIO17: SIM800L RX <- ESP TX
 *
 * Hold the button for 1.5 seconds to start SOS. Hold for 3 seconds
 * while active to stop SOS. SOS SMS waits for a fresh GPS fix; app-contact
 * status texts start when GSM is ready and repeat every 3 minutes while powered on.
 * The ESP32 creates an open access point named "aquawatch_dev".
 *
 * Libraries: DFRobot_BMI160, TinyGPSPlus (install from Library Manager).
 * Board: ESP32 Dev Module; 4 MB flash; Huge APP (3MB No OTA/1MB SPIFFS).
 */
#include <Arduino.h>
#include <EEPROM.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <driver/gpio.h>
#include <WiFi.h>
#include <WebServer.h>
#include <LittleFS.h>
#include <TinyGPSPlus.h>
#include <DFRobot_BMI160.h>
#include <math.h>
#include <vector>
#include "aquawatch_logo.h"
#include "tilt_estimator.h"
#include "imu_calibration.h"
#include "app_status_schedule.h"

// ----------------------------- Hardware -----------------------------
const uint8_t BUTTON_PIN = 25;
const uint8_t BMI_SCL_PIN = 22;
const uint8_t BMI_SDA_PIN = 23;
const uint8_t STROBE_RELAY_PIN = 4;  // active HIGH
const uint8_t SIREN_RELAY_PIN = 27;  // active LOW
const uint8_t GPS_RX_PIN = 18;      // NEO-M8U TX -> ESP RX
const uint8_t GPS_TX_PIN = 19;      // NEO-M8U RX <- ESP TX
const uint8_t SIM_RX_PIN = 16;      // SIM800L TX -> ESP RX
const uint8_t SIM_TX_PIN = 17;      // SIM800L RX <- ESP TX

const uint8_t STROBE_ON = HIGH;
const uint8_t STROBE_OFF = LOW;
const uint8_t SIREN_ON = LOW;
const uint8_t SIREN_OFF = HIGH;

HardwareSerial gpsSerial(1);
HardwareSerial simSerial(2);
TinyGPSPlus gps;
DFRobot_BMI160 bmi160;
WebServer server(80);

// ----------------------------- Configuration -----------------------------
String deviceName = "Spike";
const char *AP_NAME = "aquawatch_dev";
const char *AP_PASSWORD = nullptr; // open AP
String receiverNumber = "09925283361"; // compatibility alias for the first contact
struct Contact { String name; String number; };
std::vector<Contact> smsContacts;
std::vector<Contact> appContacts;
bool contactsFsReady = false;
const unsigned long START_HOLD_MS = 1500;
const unsigned long STOP_HOLD_MS = 3000;
const unsigned long GPS_FRESH_MS = 10000;
const unsigned long STROBE_PERIOD_MS = 250;
const unsigned long DISTRESS_CONFIRM_MS = 10000;
const float DISTRESS_UPPER_DEG = 90.0f;
const float GRAVITY = 9.80665f;
const float GYRO_SCALE = 16.4f; // BMI160 default +/-2000 dps

// ----------------------------- State -----------------------------
bool alarmActive = false;
bool buttonWasDown = false;
bool buttonActionTaken = false;
unsigned long buttonDownAt = 0;
unsigned long lastStrobeAt = 0;
AppStatusSchedule appStatusSchedule;
std::vector<Contact> appStatusRecipients;
size_t appStatusNext = 0, appStatusSent = 0;
String appStatusMessage;
bool strobeState = false;
bool bmiReady = false;
bool attitudeReady = false;
bool bmiHasRead = false;
unsigned long lastBmiReadAt = 0;
int8_t lastBmiReadError = BMI160_OK;
uint32_t bmiReadErrors = 0;
float bmiRawValues[6] = {};
String bmiInitStatus = "BMI160 not initialized";
ImuCalibration imuCalibration;
bool simReady = false;
String simModel = "Not checked";
String simIMEI = "Not checked";
String simNumber = "Unavailable";
String simNetwork = "Unknown";
String lastSosStatus = "Not sent";

float rotationX = 0.0f;
float rotationY = 0.0f;
float linearAcceleration = 0.0f;
float gyroOffsetX = 0.0f;
float gyroOffsetY = 0.0f;
float gyroOffsetZ = 0.0f;
TiltEstimator tilt;
unsigned long lastGyroMicros = 0;
unsigned long lastSensorAt = 0;
unsigned long lastSensorSuccess = 0;
struct BoatProfile { const char *id; const char *label; float normalX; float normalY; };
const BoatProfile BOAT_PROFILES[] = {
  {"fishing", "Fishing boat", 50.0f, 50.0f},
  {"robo", "Robo boat", 30.0f, 30.0f},
  {"pump", "Pump boat", 70.0f, 70.0f}
};
const uint8_t BOAT_PROFILE_COUNT = sizeof(BOAT_PROFILES) / sizeof(BOAT_PROFILES[0]);
uint8_t boatProfileIndex = 0;
enum AlarmCause { ALARM_CAUSE_NONE, ALARM_CAUSE_MANUAL_SOS, ALARM_CAUSE_TILT_DISTRESS, ALARM_CAUSE_ALARM_ONLY };
AlarmCause alarmCause = ALARM_CAUSE_NONE;
String modemLine;
bool modemSmsBody = false;
String modemSmsSender;
String inboxMessages[10];
uint8_t inboxCount = 0;
bool replyPending = false;
String replyToNumber;
String replyText;
String statusSmsData();
String eventLog[15];
uint8_t eventLogCount = 0;
unsigned long lastStatusLogAt = 0;
String uptimeText();
void addEvent(const String &message) {
  String entry = "[" + uptimeText() + "] " + message;
  if (eventLogCount < 15) eventLog[eventLogCount++] = entry;
  else { for (uint8_t i=1; i<15; i++) eventLog[i-1]=eventLog[i]; eventLog[14]=entry; }
}
void addInboxMessage(const String &sender, const String &body) {
  String item = "[" + uptimeText() + "] From " + (sender.length() ? sender : "unknown number") + ": " + body;
  if (inboxCount < 10) inboxMessages[inboxCount++] = item;
  else { for (uint8_t i=1; i<10; i++) inboxMessages[i-1]=inboxMessages[i]; inboxMessages[9]=item; }
}
bool distressCounting = false;
unsigned long distressStartedAt = 0;
float linearAccelX = 0.0f;
float linearAccelY = 0.0f;
String directionGuess = "Indeterminate";
String gpsDirectionGuess = "Waiting for GPS movement";
bool gpsDirectionActive = false;
bool gpsDirectionAnchorSet = false;
double gpsAnchorLat = 0.0, gpsAnchorLng = 0.0;
unsigned long gpsAnchorAt = 0;
String gpsRaw;
unsigned long gpsRawAt = 0;
bool gpsReceived = false;
bool odometerReady = false;
uint32_t tripDistanceMeters = 0;
uint32_t totalDistanceMeters = 0;
uint32_t odometerAccuracyMeters = 0;
unsigned long lastOdometerAt = 0;
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
header{align-items:center;justify-content:center;flex-wrap:wrap;padding-bottom:18px}
.brand{display:flex;align-items:center;justify-content:center;flex-direction:column;gap:8px;text-align:center;width:100%}.logo{display:block;width:360px;max-width:100%;height:auto;border:0;border-radius:0;background:transparent}
h2{color:#ffac77}button{background:var(--orange);color:#19120d}button:hover{background:#ffac77}a,summary{color:#ffac77}
button:focus-visible,summary:focus-visible{outline:3px solid #54d8e8;outline-offset:3px}
.card{min-width:0}.motion-grid{grid-column:1/-1;display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:12px}
.motion-grid .card{border-top:3px solid var(--orange)}.bar i{background:linear-gradient(90deg,#ff863e,#ffbf69)}
.contact-heading{display:flex;align-items:center;justify-content:space-between;gap:10px}.contact-heading h2{margin:0}
.add-contact{width:42px;height:42px;padding:0;margin:0;font-size:26px;line-height:1;border-radius:50%}
#contactform{margin-top:16px}#contactform[hidden]{display:none}#contactform label{display:block;margin:10px 0 4px;font-size:13px;color:#c2cfd5}
#nameedit{margin-left:auto}
.dot{background:currentColor}summary{cursor:pointer;margin-top:14px}#contacts{overflow-wrap:anywhere}select{width:100%;padding:11px;border-radius:9px;border:1px solid var(--line);background:#071722;color:#fff;font-size:16px}
@media(max-width:520px){main{padding:16px 10px 30px}.brand{width:100%;justify-content:center;flex-direction:column;gap:10px}.logo{width:220px}.brand .muted{text-align:center}header{justify-content:center}.motion-grid{gap:6px}.motion-grid .card{padding:12px 7px}.motion-grid h2{font-size:10px;letter-spacing:.03em;min-height:26px}.motion-grid .value{font-size:clamp(12px,3.7vw,20px);overflow-wrap:anywhere}.grid{gap:10px}}
</style></head><body><main><header><div class="brand"><img class="logo" src="/logo-transparent.png" width="600" height="400" alt="AquaWatch logo"><div class="muted">AquaWatch Access Control</div></div></header><div class="card" style="margin-bottom:16px"><div class="contact-heading"><div><div id="devicename" class="value">Spike</div></div><button id="nameedit" type="button" aria-controls="nameform" aria-expanded="false" onclick="editDeviceName(true)">Edit</button><button type="button" onclick="location.href='/logs'">Logs</button></div><form id="nameform" hidden style="margin-top:12px"><label for="nameinput">Device name</label><input id="nameinput" name="name" maxlength="16" required><p class="muted">Up to 16 letters, numbers, spaces, dots, underscores or hyphens. Included in SOS texts.</p><button type="submit">Save</button> <button type="button" onclick="editDeviceName(false)">Cancel</button><p id="nameerror" role="status"></p></form></div>
<article class="card" style="margin-bottom:16px"><h2>Boat type &amp; tilt alarm</h2><form action="/boat-type" method="get"><label for="boattype" class="muted">Boat type</label><select id="boattype" name="type"><option value="fishing">Fishing boat — normal limit 50°</option><option value="robo">Robo boat — normal limit 30°</option><option value="pump">Pump boat — normal limit 70°</option></select><button type="submit">Save boat type</button></form><p id="boatmeta" class="muted">Upper limit: ±90° for 10 seconds. Returning to the normal limit cancels automatic distress.</p></article>
<div id="alarm" class="status" role="status" aria-live="polite" style="margin:4px 0 16px"><i class="dot"></i><span>Alarm: waiting for device</span></div><p id="connection" hidden role="status" aria-live="polite">Connecting to AquaWatch...</p><p id="sensorstatus" class="muted"></p><details style="margin-bottom:16px"><summary>BMI160 diagnostics</summary><pre id="imudiagnostic" style="white-space:pre-wrap;overflow-wrap:anywhere">Waiting for sensor diagnostics...</pre></details><section class="grid"><div class="motion-grid"><article class="card"><h2>Rotation X</h2><div id="rx" class="value">--<span class="unit"> °</span></div></article><article class="card"><h2>Rotation Y</h2><div id="ry" class="value">--<span class="unit"> °</span></div></article><article class="card"><h2>Linear acceleration</h2><div id="acc" class="value">--<span class="unit"> m/s²</span></div><div class="bar"><i id="accbar"></i></div></article><article class="card"><h2>Trip distance</h2><div id="trip" class="value">--<span class="unit"> m</span></div><p class="muted" id="tripmeta">NEO-M8U odometer</p></article></div><article id="distresscard" class="card" hidden><h2>Distress countdown</h2><div id="distresssecs" class="value">0 / 10 s</div><p id="distressmeta" class="muted">Upper limit reached.</p></article><article class="card"><h2>GPS course / backup</h2><div id="direction" class="value" style="font-size:20px">--</div><p class="muted">GPS course from the last 3 seconds; motion estimate is the backup.</p></article><article class="card"><h2>Device uptime</h2><div id="up" class="value">--</div></article>
<article class="card wide"><h2>GPS location</h2><div id="gps" class="map">Waiting for NEO-M8U fix…</div><div id="gpsmeta" class="muted"></div><details id="gpsdetails"><summary>View raw GPS data (NMEA)</summary><p id="gpsrawage" class="muted"></p><pre id="gpsraw" style="white-space:pre-wrap;overflow-wrap:anywhere;max-height:240px;overflow:auto">Waiting for GPS serial data...</pre></details></article>
<article class="card"><h2>SIM800L</h2><div id="sim" class="status"><i class="dot"></i><span>Unknown</span></div><p id="signal">Signal: checking...</p><p class="muted">SIM number: <span id="simnumber">Checking…</span></p><p class="muted" id="siminfo">--</p><p id="smsstatus" class="muted"></p><button type="button" id="inboxtoggle" onclick="toggleInbox()">Inbox</button><div id="inboxpanel" hidden style="margin-top:10px;max-height:240px;overflow:auto;overflow-wrap:anywhere"></div><p class="muted">Text ALARM ONLY to activate sound and lights, or STATUS to request device status without activating them.</p></article><article class="card"><div style="padding-bottom:14px;margin-bottom:14px;border-bottom:1px solid var(--line)"><div class="contact-heading"><h2>App contacts</h2><button class="add-contact" type="button" id="appcontacttoggle" aria-label="Add app contact" aria-controls="appcontactform" aria-expanded="false" onclick="toggleAppContactForm()">+</button></div><p class="muted">These people receive routine status texts after startup, once GSM is registered, and every 3 minutes while powered on, even without distress. SOS contacts below receive distress alerts.</p><form id="appcontactform" action="/add-contact" method="get" hidden><input type="hidden" name="type" value="app"><label for="appcontactname">Person name</label><input id="appcontactname" name="name" maxlength="24" required><label for="appcontactnumber">Phone number</label><input id="appcontactnumber" name="number" type="tel" maxlength="20" required><button>Add app contact</button></form><div id="appcontacts" class="muted">Loading app contacts…</div></div><div class="contact-heading"><h2>SMS contacts</h2><button class="add-contact" type="button" id="contacttoggle" aria-label="Add SMS contact" aria-controls="contactform" aria-expanded="false" onclick="toggleContactForm()">+</button></div><form id="contactform" action="/add-contact" method="get" hidden><label for="contactname">Person name</label><input id="contactname" name="name" placeholder="Person name" maxlength="24" required><label for="contactnumber">Phone number</label><input id="contactnumber" name="number" type="tel" placeholder="Contact number" maxlength="20" required><button>Add contact</button></form><div id="contacts" class="muted">Loading contacts…</div><p class="muted">SMS contacts receive the immediate SOS alert.</p></article>
<article class="card"><h2>Alarm control</h2><button onclick="fetch('/alarm/on')">Trigger SOS</button> <button onclick="fetch('/alarm/local')">Alarm only</button> <button onclick="fetch('/alarm/off')">Cancel</button><p class="muted">Alarm only activates siren and strobe without SOS texts. Routine app status texts continue. Text ALARM ONLY to the SIM800L to activate it. Manual SOS: hold the physical button 1.5 s to start, 3 s to stop.</p></article></section>
<p class="muted" style="margin-top:18px">Refreshes every second · Wi-Fi: aquawatch_dev · http://192.168.4.1</p></main>
<script>
var $=function(id){return document.getElementById(id);};
function toggleContactForm(){
  var form=$('contactform'),button=$('contacttoggle');
  form.hidden=!form.hidden;button.setAttribute('aria-expanded',String(!form.hidden));
  button.setAttribute('aria-label',form.hidden?'Add SMS contact':'Close contact form');
  button.textContent=form.hidden?'+':'−';if(!form.hidden)$('contactname').focus();
}
function toggleAppContactForm(){var form=$('appcontactform'),button=$('appcontacttoggle');form.hidden=!form.hidden;button.setAttribute('aria-expanded',String(!form.hidden));button.setAttribute('aria-label',form.hidden?'Add app contact':'Close app contact form');button.textContent=form.hidden?'+':'−';if(!form.hidden)$('appcontactname').focus();}
function editDeviceName(open){
  $('nameform').hidden=!open;$('nameedit').setAttribute('aria-expanded',String(open));$('nameerror').textContent='';
  if(open){$('nameinput').value=$('devicename').textContent;$('nameinput').focus();}
}
function toggleInbox(){var panel=$('inboxpanel');panel.hidden=!panel.hidden;$('inboxtoggle').textContent=panel.hidden?'Inbox':'Close inbox';}
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
  $('rx').textContent=fmt(d.imu && !d.imu.calibrated ? null : d.rotationX)+' °';
  $('ry').textContent=fmt(d.imu && !d.imu.calibrated ? null : d.rotationY)+' °';
  $('acc').textContent=fmt(d.acceleration,2)+' m/s²';
  $('accbar').style.width=Math.min(100,Math.max(0,Number(d.acceleration)||0)*10)+'%';
  var odo=d.odometer||{};
  $('trip').textContent=odo.available?fmt(odo.tripMeters,1)+' m':'-- m';
  $('tripmeta').textContent=odo.available?'Accuracy ±'+fmt(odo.accuracyMeters,1)+' m':'Waiting for NEO-M8U odometer';
  $('direction').textContent=d.direction||'Indeterminate';
  if(d.boat){$('boattype').value=d.boat.id||'fishing';$('boatmeta').textContent=(d.boat.label||'Boat')+' normal limit: ±'+fmt(d.boat.normalX)+'° X / ±'+fmt(d.boat.normalY)+'° Y. Upper limit: ±90° for 10 seconds.';}
  var distress=d.distress||{};$('distresscard').hidden=!distress.visible;
  if(distress.visible){$('distresssecs').textContent=(distress.seconds||0)+' / 10 s';$('distressmeta').textContent=distress.active?'Emergency Distress active. Return to the normal limit to stop it.':'Upper limit reached — counting before Emergency Distress.';}
  $('signal').textContent='Signal: '+(d.signal||'Waiting for modem');
  $('smsstatus').textContent=d.smsStatus||'';
  $('sensorstatus').textContent=(d.imuStatus||'')+' · GPS received: '+(d.gpsChars||0)+' characters';
  var imu=d.imu;
  if(imu){
    $('imudiagnostic').textContent='Tilt calibrated: '+(imu.calibrated?'yes':'no')+' | Calibration samples: '+imu.calibrationSamples+
      '\nLast successful sensor read: '+(imu.rawAge==null?'none':fmt(imu.rawAge/1000,1)+' s ago')+
      '\nRead errors: '+imu.readErrors+' | Last error code: '+imu.lastReadError+
      (imu.rawAge==null?'':'\nRaw gyro X/Y/Z (deg/s): '+imu.gyro.map(function(v){return fmt(v,2);}).join(' / ')+
      '\nAcceleration X/Y/Z (g): '+imu.accel.map(function(v){return fmt(v,3);}).join(' / ')+
      '\nAcceleration magnitude (g): '+fmt(Math.sqrt(imu.accel.reduce(function(sum,v){return sum+v*v;},0)),3));
  }
  $('gpsraw').textContent=d.gpsRaw||'No serial data received from GPS yet.';
  $('gpsrawage').textContent=d.gpsRawAge==null?'Waiting for NEO-M8U data':'Last received '+(d.gpsRawAge/1000).toFixed(1)+' s ago';
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
  $('simnumber').textContent=d.sim.number&&d.sim.number!=='Unavailable'?d.sim.number:'Unavailable (SIM did not report it)';
  $('inboxpanel').innerHTML=(d.inbox||[]).slice().reverse().map(function(item){return '<div style="padding:7px 0;border-bottom:1px solid #34414d">'+esc(item)+'</div>';}).join('')||'<span class="muted">No received messages yet</span>';
  $('appcontacts').innerHTML=(d.appContacts||[]).map(function(c,i){return '<div style="margin:8px 0"><b>'+esc(c.name)+'</b><br>'+esc(c.number)+' <a href="/remove-contact?type=app&i='+i+'">remove</a></div>';}).join('')||'No app contacts yet';
  $('contacts').innerHTML=(d.contacts||[]).map(function(c,i){return '<div style="margin:8px 0"><b>'+esc(c.name)+'</b><br>'+esc(c.number)+' <a href="/remove-contact?type=sms&i='+i+'">remove</a></div>';}).join('')||'No SMS contacts yet';
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

const char LOGS_HTML[] PROGMEM = R"LOGHTML(
<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>AquaWatch device logs</title>
<style>:root{color-scheme:dark}body{margin:0;padding:20px;background:#07131f;color:#eef9fc;font:16px system-ui,Arial}main{max-width:900px;margin:auto}.terminal{height:65vh;min-height:320px;max-height:640px;overflow-y:auto;overflow-x:hidden;background:#020907;border:1px solid #245b35;border-radius:8px;padding:10px 16px;color:#7cff91;font:13px/1.5 Consolas,"Cascadia Mono",monospace;box-shadow:inset 0 0 18px #000}h1{font-size:25px}button{background:#ff863e;color:#19120d;border:0;border-radius:8px;padding:10px 15px;font-weight:700;cursor:pointer}.muted{color:#91adba}.row{padding:7px 0;border-bottom:1px solid #173521;overflow-wrap:anywhere;white-space:pre-wrap}.row:before{content:"> ";color:#35d95c}</style></head><body><main><button onclick="location.href='/'">← Dashboard</button><h1>AquaWatch device logs</h1><p id="live" class="muted">Connecting…</p><section class="terminal"><div id="logs">Waiting for device log…</div></section></main>
<script>function refresh(){fetch('/api?t='+Date.now()).then(function(r){if(!r.ok)throw Error('HTTP '+r.status);return r.json();}).then(function(d){var g=d.gps.valid?'GPS fix '+d.gps.lat.toFixed(5)+', '+d.gps.lng.toFixed(5):'GPS no fix';document.getElementById('live').textContent='Mode: '+d.alarmMode+' · '+d.sim.status+' · SIM signal '+d.signal+' · '+g+' · '+d.imuStatus;var box=document.getElementById('logs'),oldTop=box.scrollTop,wasAtTop=oldTop<10;box.replaceChildren();(d.log||[]).slice().reverse().forEach(function(item){var row=document.createElement('div');row.className='row';row.textContent=item;box.appendChild(row);});box.scrollTop=wasAtTop?0:oldTop;}).catch(function(e){document.getElementById('live').textContent='Log update failed: '+e.message;}).finally(function(){setTimeout(refresh,2000);});}refresh();</script></body></html>
)LOGHTML";

// ----------------------------- Contact storage -----------------------------
const int EEPROM_BYTES = 512;
const int CONTACT_SLOT_BYTES = 44;
void writeContactText(int address, const String &value, int width) { for (int i = 0; i < width; i++) EEPROM.write(address + i, i < value.length() ? value[i] : 0); }
String readContactText(int address, int width) { char buffer[45]; int n = 0; for (; n < width && EEPROM.read(address + n); n++) buffer[n] = EEPROM.read(address + n); buffer[n] = 0; return String(buffer); }
bool saveContacts() {
  if (!contactsFsReady) return false;
  File file = LittleFS.open("/contacts.txt", "w");
  if (!file) return false;
  for (const Contact &c : smsContacts) file.println(String("S\t") + c.name + "\t" + c.number);
  for (const Contact &c : appContacts) file.println(String("A\t") + c.name + "\t" + c.number);
  file.close();
  receiverNumber = smsContacts.empty() ? "" : smsContacts[0].number;
  return true;
}
void loadContacts() {
  smsContacts.clear(); appContacts.clear();
  if (contactsFsReady && LittleFS.exists("/contacts.txt")) {
    File file = LittleFS.open("/contacts.txt", "r");
    while (file && file.available()) {
      String line = file.readStringUntil('\n'); line.trim();
      int first = line.indexOf('\t'), second = first >= 0 ? line.indexOf('\t', first + 1) : -1;
      if (first == 1 && second > first + 1) {
        Contact c{line.substring(first + 1, second), line.substring(second + 1)};
        if (c.number.length() >= 5 && c.number.length() <= 20) {
          if (line[0] == 'A') appContacts.push_back(c); else if (line[0] == 'S') smsContacts.push_back(c);
        }
      }
    }
    if (file) file.close();
  } else {
    // Import contacts saved by older firmware, then move future changes to flash storage.
    uint8_t saved = EEPROM.read(0);
    if (saved > 0 && saved <= 6) {
      for (uint8_t i=0; i<saved; i++) {
        Contact c{readContactText(1 + i * CONTACT_SLOT_BYTES, 24), readContactText(1 + i * CONTACT_SLOT_BYTES + 24, 20)};
        if (c.name.length() && c.number.length() >= 5) smsContacts.push_back(c);
      }
    }
    if (smsContacts.empty()) smsContacts.push_back({"Primary", "09925283361"});
    saveContacts();
  }
  receiverNumber = smsContacts.empty() ? "" : smsContacts[0].number;
}
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
const int BOAT_PROFILE_MARKER = 330;
const BoatProfile &boatProfile() { return BOAT_PROFILES[boatProfileIndex]; }
bool atNormalLimit() { return fabs(rotationX) <= boatProfile().normalX && fabs(rotationY) <= boatProfile().normalY; }
bool atUpperLimit() { return fabs(rotationX) >= DISTRESS_UPPER_DEG || fabs(rotationY) >= DISTRESS_UPPER_DEG; }
void loadBoatProfile() {
  if (EEPROM.read(BOAT_PROFILE_MARKER) == 0xB4) {
    uint8_t saved = EEPROM.read(BOAT_PROFILE_MARKER + 1);
    if (saved < BOAT_PROFILE_COUNT) boatProfileIndex = saved;
  }
}
void handleBoatType() {
  String requested = server.arg("type");
  for (uint8_t i = 0; i < BOAT_PROFILE_COUNT; i++) {
    if (requested == BOAT_PROFILES[i].id) {
      boatProfileIndex = i;
      EEPROM.write(BOAT_PROFILE_MARKER, 0xB4); EEPROM.write(BOAT_PROFILE_MARKER + 1, i); EEPROM.commit();
      distressCounting = false;
      break;
    }
  }
  server.sendHeader("Location", "/"); server.send(303);
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
struct UbxParserState {
  uint8_t state = 0;
  uint8_t msgClass = 0, msgId = 0;
  uint16_t length = 0, index = 0;
  uint8_t payload[64] = {};
  uint8_t checksumA = 0, checksumB = 0;
  uint8_t receivedChecksumA = 0;
  bool dropPayload = false;
} ubxParser;

void processUbxPacket() {
  if (ubxParser.msgClass != 0x01 || ubxParser.msgId != 0x09 || ubxParser.length != 20 || ubxParser.dropPayload || ubxParser.payload[0] != 0) return;
  auto readU32 = [](const uint8_t *p) -> uint32_t {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
  };
  tripDistanceMeters = readU32(ubxParser.payload + 8);
  totalDistanceMeters = readU32(ubxParser.payload + 12);
  odometerAccuracyMeters = readU32(ubxParser.payload + 16);
  odometerReady = true;
  lastOdometerAt = millis();
}

void parseUbxByte(uint8_t value) {
  switch (ubxParser.state) {
    case 0: ubxParser.state = value == 0xB5 ? 1 : 0; return;
    case 1: ubxParser.state = value == 0x62 ? 2 : (value == 0xB5 ? 1 : 0); return;
    case 2: ubxParser.msgClass = value; ubxParser.checksumA = value; ubxParser.checksumB = value; ubxParser.state = 3; return;
    case 3: ubxParser.msgId = value; ubxParser.checksumA += value; ubxParser.checksumB += ubxParser.checksumA; ubxParser.state = 4; return;
    case 4: ubxParser.length = value; ubxParser.checksumA += value; ubxParser.checksumB += ubxParser.checksumA; ubxParser.state = 5; return;
    case 5:
      ubxParser.length |= (uint16_t)value << 8;
      ubxParser.checksumA += value; ubxParser.checksumB += ubxParser.checksumA;
      ubxParser.index = 0; ubxParser.dropPayload = ubxParser.length > sizeof(ubxParser.payload);
      ubxParser.state = ubxParser.length ? 6 : 7;
      return;
    case 6:
      ubxParser.checksumA += value; ubxParser.checksumB += ubxParser.checksumA;
      if (!ubxParser.dropPayload) ubxParser.payload[ubxParser.index] = value;
      if (++ubxParser.index >= ubxParser.length) ubxParser.state = 7;
      return;
    case 7:
      ubxParser.receivedChecksumA = value;
      ubxParser.state = 8; return;
    case 8: {
      bool checksumOk = ubxParser.receivedChecksumA == ubxParser.checksumA && ubxParser.checksumB == value;
      if (checksumOk) processUbxPacket();
      ubxParser.state = 0;
      return;
    }
    default: ubxParser.state = 0; return;
  }
}

void sendUbx(uint8_t msgClass, uint8_t msgId, const uint8_t *payload, uint16_t length) {
  uint8_t checksumA = 0, checksumB = 0;
  auto add = [&](uint8_t value) { checksumA += value; checksumB += checksumA; };
  gpsSerial.write(0xB5); gpsSerial.write(0x62);
  gpsSerial.write(msgClass); add(msgClass);
  gpsSerial.write(msgId); add(msgId);
  gpsSerial.write((uint8_t)(length & 0xFF)); add((uint8_t)(length & 0xFF));
  gpsSerial.write((uint8_t)(length >> 8)); add((uint8_t)(length >> 8));
  for (uint16_t i = 0; i < length; i++) { gpsSerial.write(payload[i]); add(payload[i]); }
  gpsSerial.write(checksumA); gpsSerial.write(checksumB);
}

void configureGpsOdometer() {
  // UBX-CFG-ODO: enable the receiver's trip odometer (useODO flag, bit 0).
  uint8_t odoConfig[20] = {};
  odoConfig[4] = 0x01;
  sendUbx(0x06, 0x1E, odoConfig, sizeof(odoConfig));
  delay(50);
  // UBX-CFG-MSG: output UBX-NAV-ODO once per navigation epoch on this UART.
  const uint8_t navOdoRate[] = {0x01, 0x09, 0x01};
  sendUbx(0x06, 0x01, navOdoRate, sizeof(navOdoRate));
}

void feedGPS() {
  while (gpsSerial.available()) {
    uint8_t value = (uint8_t)gpsSerial.read();
    char c = (char)value; gps.encode(c); parseUbxByte(value);
    if ((c >= 32 && c <= 126) || c == '\n') {
      if (gpsRaw.length() >= 1536) gpsRaw.remove(0, 256);
      gpsRaw += c; gpsRawAt = millis(); gpsReceived = true;
    }
  }
}
String gpsText() { return gps.location.isValid() ? String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6) : "No fix"; }
bool hasFreshGpsFix() { return gps.location.isValid() && gps.location.age() < GPS_FRESH_MS; }
void updateGpsDirection() {
  if (!gps.location.isValid() || gps.location.age() >= GPS_FRESH_MS) {
    gpsDirectionActive = false; gpsDirectionAnchorSet = false; return;
  }
  if (!gps.location.isUpdated()) return;
  double lat = gps.location.lat(), lng = gps.location.lng();
  unsigned long now = millis();
  if (!gpsDirectionAnchorSet) {
    gpsAnchorLat = lat; gpsAnchorLng = lng; gpsAnchorAt = now; gpsDirectionAnchorSet = true; return;
  }
  if (now - gpsAnchorAt < 3000) return;
  const double rad = PI / 180.0;
  double lat1 = gpsAnchorLat * rad, lat2 = lat * rad;
  double dLat = (lat - gpsAnchorLat) * rad, dLng = (lng - gpsAnchorLng) * rad;
  double h = sin(dLat/2)*sin(dLat/2) + cos(lat1)*cos(lat2)*sin(dLng/2)*sin(dLng/2);
  if (h > 1.0) h = 1.0;
  double distance = 6371000.0 * 2.0 * atan2(sqrt(h), sqrt(1.0-h));
  if (distance < 4.0) {
    gpsDirectionGuess = "Stationary (GPS)";
    gpsDirectionActive = true;
  } else {
    double y = sin(dLng) * cos(lat2);
    double x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dLng);
    double bearing = atan2(y, x) * 180.0 / PI;
    if (bearing < 0) bearing += 360.0;
    static const char *HEADINGS[] = {"North", "North-east", "East", "South-east", "South", "South-west", "West", "North-west"};
    uint8_t sector = (uint8_t)(((int)(bearing + 22.5) / 45) % 8);
    gpsDirectionGuess = String(HEADINGS[sector]) + " (GPS course)";
    gpsDirectionActive = true;
  }
  gpsAnchorLat = lat; gpsAnchorLng = lng; gpsAnchorAt = now;
}
String currentDirection() { return gpsDirectionActive ? gpsDirectionGuess : directionGuess; }

// ----------------------------- BMI160 -----------------------------
String imuStatusText() {
  if (!bmiReady) return bmiInitStatus;
  if (!bmiHasRead) return bmiReadErrors ? "BMI160 detected; read failed, code " + String(lastBmiReadError) : "BMI160 detected; waiting for first sample";
  if (millis() - lastBmiReadAt > 1000) return "BMI160 I2C read timeout; last error " + String(lastBmiReadError);
  if (!attitudeReady) {
    String reason = "hold still in the normal operating position";
    if (imuCalibration.status() == ImuCalibration::MOTION) reason = "movement detected; hold still";
    else if (imuCalibration.status() == ImuCalibration::HIGH_RATE) reason = "gyro rate too high; check raw readings";
    else if (imuCalibration.status() == ImuCalibration::INVALID_GRAVITY) reason = "acceleration not near 1 g; check raw readings";
    return "BMI160 receiving data; calibrating: " + reason;
  }
  if (!lastSensorSuccess || millis() - lastSensorSuccess > 1000) return "BMI160 receiving data; tilt estimate unavailable";
  return "BMI160 receiving data; tilt ready";
}
void updateSensor() {
  if (!bmiReady || millis() - lastSensorAt < 40) return;
  lastSensorAt = millis(); int16_t data[6];
  int8_t result = bmi160.getAccelGyroData(data);
  if (result != BMI160_OK) { lastBmiReadError = result; bmiReadErrors++; return; }
  // Read health is independent of calibration and tilt-estimator acceptance.
  bmiHasRead = true; lastBmiReadAt = millis();
  for (int i = 0; i < 6; i++) bmiRawValues[i] = data[i] / (i < 3 ? GYRO_SCALE : 16384.0f);
  unsigned long now = micros();
  if (!attitudeReady) {
    if (!imuCalibration.addSample(bmiRawValues, lastBmiReadAt)) return;
    if (!tilt.begin({imuCalibration.mean(3), imuCalibration.mean(4), imuCalibration.mean(5)})) {
      imuCalibration.reject(ImuCalibration::INVALID_GRAVITY);
      return;
    }
    gyroOffsetX = imuCalibration.mean(0); gyroOffsetY = imuCalibration.mean(1); gyroOffsetZ = imuCalibration.mean(2);
    rotationX = rotationY = 0;
    attitudeReady = true; lastGyroMicros = now;
    Serial.println(F("BMI160 calibrated; normal operating position saved"));
    addEvent("BMI160 calibrated; tilt monitoring ready");
  } else {
    float dt = (now - lastGyroMicros) / 1000000.0f; lastGyroMicros = now;
    // Keep both sensors in their native frame; tilt maps to the startup frame.
    TiltEstimator::Vector acceleration = {data[3] / 16384.0f, data[4] / 16384.0f, data[5] / 16384.0f};
    TiltEstimator::Vector gyro = {data[0] / GYRO_SCALE - gyroOffsetX,
                                data[1] / GYRO_SCALE - gyroOffsetY,
                                data[2] / GYRO_SCALE - gyroOffsetZ};
    if (!tilt.update(acceleration, gyro, dt)) return;
    rotationX = tilt.x(); rotationY = tilt.y();
  }
  lastSensorSuccess = millis();
  float ax = data[3] / 16384.0f * GRAVITY, ay = data[4] / 16384.0f * GRAVITY, az = data[5] / 16384.0f * GRAVITY;
  float total = sqrt(ax*ax + ay*ay + az*az);
  linearAcceleration = 0; linearAccelX = 0; linearAccelY = 0;
  if (total >= .1) {
    float lx = ax - ax / total * GRAVITY, ly = ay - ay / total * GRAVITY, lz = az - az / total * GRAVITY;
    linearAccelX = lx; linearAccelY = ly;
    linearAcceleration = sqrt(lx*lx + ly*ly + lz*lz);
    if (linearAcceleration < .20) { linearAcceleration = 0; linearAccelX = 0; linearAccelY = 0; }
  }
  // This is device-relative: without a compass, north/south cannot be determined.
  float forwardScore = linearAccelX + rotationY * 0.08f;
  float sideScore = linearAccelY + rotationX * 0.08f;
  if (fabs(forwardScore) < 0.35f && fabs(sideScore) < 0.35f) directionGuess = "Stationary / indeterminate";
  else if (fabs(forwardScore) >= fabs(sideScore)) directionGuess = forwardScore >= 0 ? "Likely forward" : "Likely backward";
  else directionGuess = sideScore >= 0 ? "Likely starboard" : "Likely port";
}
void setupBMI() {
  // DFRobot_BMI160's constructor starts Wire on the default pins. Restart
  // it here so ESP32 actually uses GPIO23 for SDA instead of GPIO21.
  Wire.end();
  if (!Wire.begin(BMI_SDA_PIN, BMI_SCL_PIN)) {
    bmiReady = false;
    bmiInitStatus = "BMI160 I2C bus failed to start";
    Serial.println(bmiInitStatus);
    return;
  }
  int8_t result = bmi160.I2cInit(0x69); if (result != BMI160_OK) result = bmi160.I2cInit(0x68);
  bmiReady = result == BMI160_OK;
  if (!bmiReady) { bmiInitStatus = "BMI160 initialization failed, code " + String(result); Serial.println(bmiInitStatus); return; }
  // Calibration runs in updateSensor and retries until stationary. Its failure
  // must never mark a detected sensor unavailable or stop further I2C reads.
  Serial.println(F("BMI160 detected; hold still in normal operating position (flat or vertical)"));
}

// ----------------------------- SIM800L -----------------------------
void serviceStrobe();
void updateAlarm();
void setAlarm(bool on, AlarmCause cause);
void serviceLive() {
  feedGPS(); updateGpsDirection(); updateSensor(); updateAlarm(); server.handleClient(); yield();
}
String atCommand(const String &cmd, unsigned long timeout) {
  while (simSerial.available()) simSerial.read(); simSerial.println(cmd); String out; unsigned long start = millis();
  while (millis() - start < timeout) { if (smsBusy) serviceLive(); else { feedGPS(); updateSensor(); serviceStrobe(); } while (simSerial.available()) { char c = simSerial.read(); if (out.length() < 512) out += c; } yield(); }
  return out;
}
void setupSIM() {
  simSerial.setRxBufferSize(1024);
  simSerial.begin(9600, SERIAL_8N1, SIM_RX_PIN, SIM_TX_PIN); delay(300);
  atCommand("AT", 1200); atCommand("ATE0", 800); atCommand("AT+CMGF=1", 1000); atCommand("AT+CNMI=2,2,0,0,0", 1000);
  String model = atCommand("AT+CGMM", 1500), imei = atCommand("AT+GSN", 1500);
  model.replace("\r", ""); model.replace("\n", " "); model.replace("OK", ""); model.trim();
  imei.replace("\r", ""); imei.replace("\n", " "); imei.replace("OK", ""); imei.trim();
  simModel = model.length() ? model : "Unknown"; simIMEI = imei.length() ? imei : "Unknown";
  String numberReply = atCommand("AT+CNUM", 2000);
  int comma = numberReply.indexOf(',');
  int numberOpen = comma >= 0 ? numberReply.indexOf('\"', comma) : -1;
  int numberClose = numberOpen >= 0 ? numberReply.indexOf('\"', numberOpen + 1) : -1;
  if (numberOpen >= 0 && numberClose > numberOpen) {
    String reportedNumber = numberReply.substring(numberOpen + 1, numberClose);
    reportedNumber.trim();
    if (reportedNumber.length() >= 5 && reportedNumber.length() <= 20) simNumber = reportedNumber;
  }
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
  while (simSerial.available()) {
    char c=simSerial.read();
    if (signalPending && signalResponse.length()<256) signalResponse+=c;
    if (c=='\r') continue;
    if (c=='\n') {
      modemLine.trim();
      if (modemLine.startsWith("+CMT:")) {
        modemSmsBody=true; modemSmsSender="";
        int q1=modemLine.indexOf('\"'), q2=q1>=0?modemLine.indexOf('\"',q1+1):-1;
        if(q1>=0 && q2>q1) modemSmsSender=modemLine.substring(q1+1,q2);
      }
      else if (modemSmsBody && modemLine.length()==0) {
        // Some SIM800L firmware inserts a blank line between the SMS header and body.
      }
      else if (modemSmsBody) {
        String body=modemLine; body.trim();
        String command=body; command.toUpperCase();
        addInboxMessage(modemSmsSender, body);
        addEvent("SMS received from " + (modemSmsSender.length()?modemSmsSender:"unknown number"));
        if (command=="ALARM ONLY" || command=="ALARM" || command=="SOS ALARM") {
          setAlarm(true, ALARM_CAUSE_ALARM_ONLY);
          lastSosStatus="Alarm only activated by SMS; no SMS blast sent";
          addEvent("ALARM ONLY command accepted");
          replyToNumber=modemSmsSender;
          replyText="AquaWatch received your ALARM ONLY command. Siren and lights are active; no SOS blast was sent.";
          replyPending=replyToNumber.length()>0;
        } else if (command=="STATUS" || command=="GET STATUS" || command=="STATUS NOW") {
          addEvent("STATUS command accepted; alarm outputs unchanged");
          replyToNumber=modemSmsSender; replyText="__STATUS__"; replyPending=replyToNumber.length()>0;
        } else {
          addEvent("SMS text received (no matching command)");
        }
        modemSmsBody=false;
      }
      modemLine="";
    } else if (modemLine.length()<160) modemLine+=c;
  }
  if (!signalPending) {
    if (millis() - signalRequestAt < 10000) return;
    while (simSerial.available()) simSerial.read();
    signalResponse = ""; signalRequestAt = millis(); signalPending = true;
    simSerial.println("AT+CSQ");
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
  String msg = "AquaWatch " + deviceName;
  msg += alarmCause == ALARM_CAUSE_MANUAL_SOS ? " SOS button triggered" : " Emergency Distress";
  msg += "\nMode: " + String(alarmCause == ALARM_CAUSE_TILT_DISTRESS ? "Automatic distress" : "SOS");
  msg += "\nUptime: " + uptimeText();
  msg += "\nSIM800L: " + simNetwork + " | " + simNumber + " | " + signalText();
  msg += "\nLocation: " + gpsText();
  msg += " · " + String(gps.satellites.isValid() ? gps.satellites.value() : 0) + " satellites";
  msg += "\nGyro X: "; msg += String(rotationX, 1); msg += " deg Y: "; msg += String(rotationY, 1); msg += " deg";
  msg += "\nAcceleration: "; msg += String(linearAcceleration, 2); msg += " m/s2";
  msg += "\nSensor: "; msg += imuStatusText();
  msg += "\nDirection: "; msg += currentDirection();
  return msg;
}
String statusSmsData() {
  String msg = "AquaWatch " + deviceName + " STATUS";
  msg += "\nMode: " + String(!alarmActive ? "Monitoring" : alarmCause == ALARM_CAUSE_ALARM_ONLY ? "Alarm only" : alarmCause == ALARM_CAUSE_TILT_DISTRESS ? "Automatic distress" : "SOS");
  msg += "\nUptime: " + uptimeText();
  msg += "\nSIM800L: " + simNetwork + " | " + simNumber + " | " + signalText();
  msg += "\nGPS: " + gpsText() + " | " + String(gps.satellites.isValid() ? gps.satellites.value() : 0) + " satellites";
  msg += "\nGyro X/Y: " + String(rotationX, 1) + " / " + String(rotationY, 1) + " deg";
  msg += "\nAcceleration: " + String(linearAcceleration, 2) + " m/s2";
  msg += "\nSensor: "; msg += imuStatusText();
  msg += "\nDirection: " + currentDirection();
  return msg;
}
bool sendSmsTo(const String &number, bool requireAlarm) {
  if (!number.length() || (requireAlarm && !alarmActive)) return false;
  while (simSerial.available()) simSerial.read();
  simSerial.print("AT+CMGS=\""); simSerial.print(number); simSerial.println("\"");
  unsigned long start = millis(); String prompt;
  while (millis() - start < 5000) {
    serviceLive();
    while (simSerial.available()) { char c = simSerial.read(); if (prompt.length() < 256) prompt += c; }
    if (requireAlarm && !alarmActive) { simSerial.write(27); return false; }
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
bool sendCommandReply(const String &number, const String &message) {
  if (!number.length() || !message.length()) return false;
  smsBusy=true;
  while(simSerial.available())simSerial.read();
  simSerial.print("AT+CMGS=\""); simSerial.print(number); simSerial.println("\"");
  unsigned long start=millis(); String response; bool prompted=false;
  while(millis()-start<5000) {
    serviceLive();
    while(simSerial.available()){char c=simSerial.read();if(response.length()<256)response+=c;}
    if(response.indexOf('>')>=0){prompted=true;break;}
    if(response.indexOf("ERROR")>=0)break;
  }
  if(prompted){
    simSerial.print(message); simSerial.write(26); response=""; start=millis();
    while(millis()-start<30000){
      serviceLive();
      while(simSerial.available()){char c=simSerial.read();if(response.length()<512)response+=c;}
      if(response.indexOf("ERROR")>=0)break;
      if(response.indexOf("+CMGS:")>=0 && response.indexOf("\nOK")>=0){smsBusy=false;return true;}
    }
  } else simSerial.write(27);
  smsBusy=false; return false;
}
void sendSOS() {
  // Never submit a text that says "No fix". Keep the alarm active and wait
  // until TinyGPS++ reports a valid location newer than GPS_FRESH_MS instead.
  if (!hasFreshGpsFix()) { lastSosStatus = "Waiting for a fresh GPS fix; SMS not sent"; return; }
  if (smsContacts.empty()) { lastSosStatus = "No SMS contacts"; return; }
  // Snapshot recipients and sensor values once for this blast.
  std::vector<Contact> recipients = smsContacts;
  size_t total = recipients.size();
  blastMessage = smsData(); smsBusy = true;
  String reg = atCommand("AT+CREG?", 1500);
  simReady = reg.indexOf(",1") >= 0 || reg.indexOf(",5") >= 0;
  simNetwork = simReady ? "Network registered" : "Network not registered";
  if (!simReady) { lastSosStatus = "GSM unavailable"; smsBusy = false; return; }
  if (!hasFreshGpsFix()) { lastSosStatus = "GPS fix became stale; SMS not sent"; smsPending = true; smsBusy = false; return; }
  size_t sent = 0;
  for (size_t i = 0; i < total && alarmActive; i++) {
    if (!hasFreshGpsFix()) {
      lastSosStatus = "GPS fix became stale; remaining SMS not sent";
      smsPending = true;
      break;
    }
    lastSosStatus = "Sending " + String(i + 1) + "/" + String(total);
    if (sendSmsTo(recipients[i].number, true)) sent++;
  }
  lastSosStatus = String(sent) + "/" + String(total) + " accepted by modem" + (alarmActive ? "" : " (cancelled)");
  smsBusy = false;
}
void sendAppStatus() {
  if (smsBusy || signalPending) return;
  // Let the main loop send a queued, GPS-ready SOS before the next routine text.
  if (alarmActive && alarmCause != ALARM_CAUSE_ALARM_ONLY && smsPending && hasFreshGpsFix()) return;
  if (appStatusRecipients.empty()) {
    if (appContacts.empty() || !appStatusSchedule.due(millis())) return;
    smsBusy = true;
    String reg = atCommand("AT+CREG?", 1500);
    simReady = reg.indexOf(",1") >= 0 || reg.indexOf(",5") >= 0;
    simNetwork = simReady ? "Network registered" : "Network not registered";
    smsBusy = false;
    if (!simReady) {
      lastSosStatus = "App status waiting for GSM registration";
      appStatusSchedule.networkUnavailable(millis());
      return;
    }
    // Contacts may have changed while serviceLive serviced the dashboard.
    if (appContacts.empty()) return;
    appStatusRecipients = appContacts;
    appStatusMessage = statusSmsData();
    appStatusNext = appStatusSent = 0;
    addEvent("Routine app status batch started");
  }
  if (alarmActive && alarmCause != ALARM_CAUSE_ALARM_ONLY && smsPending && hasFreshGpsFix()) return;
  // One recipient per loop allows SOS/replies to run between routine texts.
  // Keep a separate snapshot because an intervening SOS changes blastMessage.
  blastMessage = appStatusMessage;
  smsBusy = true;
  lastSosStatus = "Sending app status " + String((unsigned long)(appStatusNext+1)) + "/" + String((unsigned long)appStatusRecipients.size());
  if (sendSmsTo(appStatusRecipients[appStatusNext].number, false)) appStatusSent++;
  appStatusNext++;
  smsBusy = false;
  if (appStatusNext == appStatusRecipients.size()) {
    lastSosStatus = String((unsigned long)appStatusSent) + "/" + String((unsigned long)appStatusRecipients.size()) + " app status texts accepted by modem";
    addEvent(lastSosStatus);
    appStatusRecipients.clear(); appStatusMessage = "";
    appStatusSchedule.batchFinished(millis());
  }
}
// ----------------------------- Alarm -----------------------------
void setAlarm(bool on, AlarmCause cause = ALARM_CAUSE_NONE) {
  if (on == alarmActive && (!on || cause == alarmCause)) return;
  alarmActive = on;
  if (on) {
    alarmCause = cause;
    digitalWrite(SIREN_RELAY_PIN, SIREN_ON); strobeState = true;
    digitalWrite(STROBE_RELAY_PIN, STROBE_ON); lastStrobeAt = millis(); smsPending = cause != ALARM_CAUSE_ALARM_ONLY;
    addEvent(cause == ALARM_CAUSE_ALARM_ONLY ? "Alarm only ON (sound + lights; no SOS texts)" : cause == ALARM_CAUSE_TILT_DISTRESS ? "Automatic distress alarm ON" : "SOS alarm ON");
    lastSosStatus = cause == ALARM_CAUSE_ALARM_ONLY ? "Alarm only active; routine app status continues" : hasFreshGpsFix() ? "SMS pending" : "Waiting for a fresh GPS fix; SMS not sent";
  } else {
    digitalWrite(SIREN_RELAY_PIN, SIREN_OFF); digitalWrite(STROBE_RELAY_PIN, STROBE_OFF);
    strobeState = false; smsPending = false; alarmCause = ALARM_CAUSE_NONE; addEvent("Alarm OFF");
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
    setAlarm(!alarmActive, ALARM_CAUSE_MANUAL_SOS); buttonActionTaken = true;
  }
  if (!down) { buttonDownAt = 0; buttonActionTaken = false; }
  buttonWasDown = down;
  bool sensorFresh = bmiReady && attitudeReady && lastSensorSuccess && now - lastSensorSuccess < 250;
  if (!sensorFresh) {
    // Do not let a period without IMU readings count toward the 10 seconds.
    distressCounting = false;
  } else if (atNormalLimit()) {
    distressCounting = false;
    // Only automatic distress is cancelled by returning to normal; manual SOS stays active.
    if (alarmActive && alarmCause == ALARM_CAUSE_TILT_DISTRESS) setAlarm(false);
  } else if (!alarmActive) {
    if (atUpperLimit() && !distressCounting) {
      distressCounting = true;
      distressStartedAt = now;
    }
    // Once started, the 10-second count continues above the normal limit.
    if (distressCounting && now - distressStartedAt >= DISTRESS_CONFIRM_MS) {
      distressCounting = false;
      setAlarm(true, ALARM_CAUSE_TILT_DISTRESS);
    }
  }
  serviceStrobe();
}

// ----------------------------- Web API -----------------------------
void handleApi() {
  // Do not let a cached response or a bad peripheral value freeze the dashboard.
  server.sendHeader("Cache-Control", "no-store, max-age=0");
  bool automaticDistress = alarmActive && alarmCause == ALARM_CAUSE_TILT_DISTRESS;
  bool distressVisible = distressCounting || automaticDistress;
  unsigned long distressSeconds = automaticDistress ? 10 : (distressCounting ? min(10UL, (millis() - distressStartedAt) / 1000UL) : 0);
  String out = "{\"rotationX\":" + String(rotationX, 2) + ",\"rotationY\":" + String(rotationY, 2) + ",\"acceleration\":" + String(linearAcceleration, 2) + ",\"uptime\":\"" + uptimeText() + "\",\"alarm\":" + String(alarmActive ? "true" : "false") + ",\"number\":\"" + jsonEscape(receiverNumber) + "\",\"gps\":{";
  out += "\"valid\":" + String(gps.location.isValid() ? "true" : "false") + ",\"lat\":" + String(gps.location.lat(), 6) + ",\"lng\":" + String(gps.location.lng(), 6) + ",\"sats\":" + String(gps.satellites.isValid() ? gps.satellites.value() : 0) + ",\"age\":" + String(gps.location.isValid() ? gps.location.age() : 0) + "},\"sim\":{\"ready\":" + String(simReady ? "true" : "false") + ",\"status\":\"" + jsonEscape(simNetwork) + "\",\"model\":\"" + jsonEscape(simModel) + "\",\"imei\":\"" + jsonEscape(simIMEI) + "\",\"number\":\"" + jsonEscape(simNumber) + "\"}}";
  out.remove(out.length() - 1); out += ",\"contacts\":[";
  for (size_t i = 0; i < smsContacts.size(); i++) { if (i) out += ","; out += "{\"name\":\"" + jsonEscape(smsContacts[i].name) + "\",\"number\":\"" + jsonEscape(smsContacts[i].number) + "\"}"; }
  out += "],\"appContacts\":[";
  for (size_t i = 0; i < appContacts.size(); i++) { if (i) out += ","; out += "{\"name\":\"" + jsonEscape(appContacts[i].name) + "\",\"number\":\"" + jsonEscape(appContacts[i].number) + "\"}"; }
  out += "],\"inbox\":[";
  for(uint8_t i=0;i<inboxCount;i++){if(i)out+=",";out+="\""+jsonEscape(inboxMessages[i])+"\"";}
  out += "],\"signal\":\"" + jsonEscape(signalText()) + "\",\"smsStatus\":\"" + jsonEscape(lastSosStatus) + "\",\"gpsRaw\":\"" + jsonEscape(gpsRaw) + "\",\"gpsRawAge\":" + (gpsReceived ? String(millis() - gpsRawAt) : String("null"));
  out += ",\"imuStatus\":\"" + jsonEscape(imuStatusText()) + "\",\"gpsChars\":" + String(gps.charsProcessed()) + ",\"firmware\":\"AquaWatch 1.5.0\"}";
  out.remove(out.length()-1); out += ",\"deviceName\":\"" + jsonEscape(deviceName) + "\",\"direction\":\"" + jsonEscape(currentDirection()) + "\",\"odometer\":{\"available\":" + String(odometerReady && millis() - lastOdometerAt < 5000 ? "true" : "false") + ",\"tripMeters\":" + String(tripDistanceMeters) + ",\"totalMeters\":" + String(totalDistanceMeters) + ",\"accuracyMeters\":" + String(odometerAccuracyMeters) + ",\"age\":" + (odometerReady ? String(millis() - lastOdometerAt) : String("null")) + "},\"boat\":{\"id\":\"" + String(boatProfile().id) + "\",\"label\":\"" + String(boatProfile().label) + "\",\"normalX\":" + String(boatProfile().normalX, 0) + ",\"normalY\":" + String(boatProfile().normalY, 0) + "},\"distress\":{\"visible\":" + String(distressVisible ? "true" : "false") + ",\"active\":" + String(automaticDistress ? "true" : "false") + ",\"seconds\":" + String(distressSeconds) + "}}";
  out.remove(out.length()-1); out += ",\"alarmMode\":\"" + String(!alarmActive ? "Monitoring" : alarmCause == ALARM_CAUSE_ALARM_ONLY ? "Alarm only (sound + lights)" : alarmCause == ALARM_CAUSE_TILT_DISTRESS ? "Automatic distress" : "SOS") + "\",\"log\":[";
  for(uint8_t i=0;i<eventLogCount;i++){if(i)out+=",";out+="\""+jsonEscape(eventLog[i])+"\"";}
  out += "]}";
  out.remove(out.length()-1);
  out += ",\"imu\":{\"detected\":" + String(bmiReady ? "true" : "false") + ",\"calibrated\":" + String(attitudeReady ? "true" : "false");
  out += ",\"rawAge\":" + (bmiHasRead ? String(millis() - lastBmiReadAt) : String("null"));
  out += ",\"readErrors\":" + String(bmiReadErrors) + ",\"lastReadError\":" + String(lastBmiReadError);
  out += ",\"calibrationSamples\":" + String(imuCalibration.samples()) + ",\"gyro\":[";
  for (int i=0; i<3; i++) { if (i) out += ","; out += String(bmiRawValues[i], 3); }
  out += "],\"accel\":[";
  for (int i=3; i<6; i++) { if (i>3) out += ","; out += String(bmiRawValues[i], 4); }
  out += "]}}";
  server.send(200, "application/json", out);
}
void handleSetNumber() { if (server.hasArg("number") && !smsContacts.empty()) { String n = server.arg("number"); n.trim(); if (n.length() >= 5 && n.length() <= 20) { receiverNumber = n; smsContacts[0].number = n; saveContacts(); } } server.sendHeader("Location", "/"); server.send(303); }
void handleAddContact() {
  String name=server.arg("name"), number=server.arg("number"); name.trim(); number.trim();
  name.replace("\t"," "); name.replace("\r"," "); name.replace("\n"," "); name.replace("|"," ");
  number.replace("\t",""); number.replace("\r",""); number.replace("\n",""); number.replace("|","");
  if(name.length() && name.length()<=24 && number.length()>=5 && number.length()<=20) {
    Contact c{name,number};
    if(server.arg("type")=="app") appContacts.push_back(c); else smsContacts.push_back(c);
    saveContacts();
  }
  receiverNumber=smsContacts.empty()?"":smsContacts[0].number;
  server.sendHeader("Location", "/"); server.send(303);
}
void handleRemoveContact() {
  int index=server.hasArg("i")?server.arg("i").toInt():-1;
  if(server.arg("type")=="app") { if(index>=0 && (size_t)index<appContacts.size()) appContacts.erase(appContacts.begin()+index); }
  else { if(index>=0 && (size_t)index<smsContacts.size()) smsContacts.erase(smsContacts.begin()+index); }
  saveContacts(); receiverNumber=smsContacts.empty()?"":smsContacts[0].number;
  server.sendHeader("Location", "/"); server.send(303);
}void setupWeb() { WiFi.mode(WIFI_AP); WiFi.softAP(AP_NAME); server.on("/", [](){ server.sendHeader("Cache-Control", "no-store, max-age=0"); server.send_P(200, "text/html", INDEX_HTML); }); server.on("/logo-transparent.png", [](){ server.sendHeader("Cache-Control", "public, max-age=86400"); server.send_P(200, "image/png", reinterpret_cast<const char *>(AQUAWATCH_LOGO), sizeof(AQUAWATCH_LOGO)); }); server.on("/logs", [](){ server.sendHeader("Cache-Control", "no-store, max-age=0"); server.send_P(200, "text/html", LOGS_HTML); }); server.on("/api", handleApi); server.on("/device-name", HTTP_POST, handleDeviceName); server.on("/boat-type", handleBoatType); server.on("/set-number", handleSetNumber); server.on("/add-contact", handleAddContact); server.on("/remove-contact", handleRemoveContact); server.on("/alarm/on", [](){ setAlarm(true, ALARM_CAUSE_MANUAL_SOS); server.send(200, "text/plain", "ON"); }); server.on("/alarm/local", [](){ setAlarm(true, ALARM_CAUSE_ALARM_ONLY); server.send(200, "text/plain", "ON (alarm only)"); }); server.on("/alarm/off", [](){ setAlarm(false); server.send(200, "text/plain", "OFF"); }); server.begin(); }

void setup() {
  // Preload output latches before enabling outputs to avoid an active-low
  // siren pulse. Arduino-ESP32 digitalWrite requires pinMode first.
  gpio_set_level(static_cast<gpio_num_t>(SIREN_RELAY_PIN), SIREN_OFF);
  gpio_set_level(static_cast<gpio_num_t>(STROBE_RELAY_PIN), STROBE_OFF);
  pinMode(SIREN_RELAY_PIN, OUTPUT);
  digitalWrite(SIREN_RELAY_PIN, SIREN_OFF);
  pinMode(STROBE_RELAY_PIN, OUTPUT);
  digitalWrite(STROBE_RELAY_PIN, STROBE_OFF);
  pinMode(BUTTON_PIN, INPUT_PULLDOWN);

  Serial.begin(115200);
  delay(100);
  EEPROM.begin(EEPROM_BYTES);
  // Format an uninitialized filesystem on the new ESP32 when mounting fails.
  contactsFsReady = LittleFS.begin(true);
  if (!contactsFsReady) Serial.println(F("LittleFS unavailable; contact lists will not persist"));
  loadContacts(); loadDeviceName(); loadBoatProfile();
  gpsSerial.setRxBufferSize(1024);
  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  configureGpsOdometer();
  setupBMI(); setupSIM(); setupWeb();
  addEvent("Device ready; monitoring");
  Serial.println(F("AquaWatch ready at http://192.168.4.1"));
}
void loop() {
  serviceLive(); updateSignal();
  if (replyPending && !smsBusy && !signalPending) {
    String recipient=replyToNumber;
    String message=replyText=="__STATUS__" ? "AquaWatch received your STATUS command.\n" + statusSmsData() : replyText;
    replyPending=false; replyToNumber=""; replyText="";
    bool sent=sendCommandReply(recipient,message);
    addEvent(sent ? "SMS command reply accepted by modem" : "SMS command reply failed");
  }
  if (millis() - lastStatusLogAt >= 2000) {
    lastStatusLogAt = millis();
    String gpsStatus = gps.location.isValid()
      ? "GPS fix " + String(gps.location.lat(), 5) + "," + String(gps.location.lng(), 5) + " " + String(gps.satellites.isValid() ? gps.satellites.value() : 0) + " sats"
      : "GPS no fix (" + String(gps.charsProcessed()) + " chars)";
    String imuStatus = imuStatusText(); if (attitudeReady && lastSensorSuccess && millis() - lastSensorSuccess <= 1000) imuStatus += " | X=" + String(rotationX, 1) + " Y=" + String(rotationY, 1) + " deg";
    String mode = !alarmActive ? "Monitoring" : alarmCause == ALARM_CAUSE_ALARM_ONLY ? "Alarm only" : alarmCause == ALARM_CAUSE_TILT_DISTRESS ? "Auto distress" : "SOS";
    addEvent("STATUS | " + mode + " | " + gpsStatus + " | SIM800L " + simNetwork + ", " + signalText() + " | " + imuStatus + " | Direction " + currentDirection());
  }
  if (alarmActive && alarmCause != ALARM_CAUSE_ALARM_ONLY && !smsBusy && !signalPending) {
    if (smsPending) {
      if (smsContacts.empty()) { smsPending=false; lastSosStatus="No SMS contacts"; }
      else if (hasFreshGpsFix()) { smsPending=false; sendSOS(); }
      else lastSosStatus="Waiting for a fresh GPS fix; SMS alert not sent";
    }
  }
  sendAppStatus();
}

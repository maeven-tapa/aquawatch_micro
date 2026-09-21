// Host checks run the sketch's alarm function with mocked hardware and its real dashboard script.
const fs = require('fs'), vm = require('vm'), assert = require('assert/strict');
const src = fs.readFileSync('aquawatch.ino','utf8');
const body = src.slice(src.indexOf('void updateAlarm() {'), src.indexOf('// ----------------------------- Web API'))
  .replace('void updateAlarm()', 'function updateAlarm()').replace(/\b(bool|unsigned long)\s+/g,'let ');
const c = {now:1000, button:0, millis(){return this.now;}, digitalRead(){return this.button;},
  BUTTON_PIN:16,HIGH:1,buttonWasDown:false,buttonActionTaken:false,buttonDownAt:0,alarmActive:false,
  START_HOLD_MS:1500,STOP_HOLD_MS:3000,bmiReady:true,lastSensorSuccess:1000,tiltArmed:true,
  rotationX:0,rotationY:0,AUTO_REARM_DEG:45,AUTO_TRIGGER_DEG:50,fabs:Math.abs, serviceStrobe(){},
  setAlarm(on){ c.alarmActive=on; if(!on)c.tiltArmed=false; }};
c.millis=()=>c.now; c.digitalRead=()=>c.button;
vm.createContext(c);vm.runInContext(body,c);
function tick(x,y){c.rotationX=x;c.rotationY=y;c.lastSensorSuccess=c.now;c.updateAlarm();}
for(const [x,y] of [[50,0],[-50,0],[0,50],[0,-50],[55,-60]]){
  c.alarmActive=false;c.tiltArmed=true;tick(x,y);assert.equal(c.alarmActive,true);
  c.setAlarm(false);tick(x,y);assert.equal(c.alarmActive,false,'cancel must latch');
  tick(44,44);assert.equal(c.tiltArmed,true);tick(x,y);assert.equal(c.alarmActive,true);
}
c.alarmActive=false;c.tiltArmed=true;tick(49.9,-49.9);assert.equal(c.alarmActive,false);
c.rotationX=70;c.lastSensorSuccess=1;c.updateAlarm();assert.equal(c.alarmActive,false,'stale sensor');
c.bmiReady=false;c.button=1;c.updateAlarm();c.now+=1499;c.updateAlarm();assert.equal(c.alarmActive,false);
c.now++;c.updateAlarm();assert.equal(c.alarmActive,true);c.now+=4000;c.updateAlarm();assert.equal(c.alarmActive,true,'one action per hold');
c.button=0;c.updateAlarm();c.button=1;c.updateAlarm();c.now+=3000;c.updateAlarm();assert.equal(c.alarmActive,false);
const nodes={};const el=()=>({textContent:'',innerHTML:'',style:{},open:true,querySelector(){return this;}});
for (const match of src.matchAll(/id="([a-z]+)"/g)) nodes[match[1]]=el();
const data={rotationX:50,rotationY:-12,acceleration:1.23,uptime:'0:01:00',alarm:true,signal:'-90 dBm (CSQ 12)',smsStatus:'Sending 1/2',gpsRaw:'$GPGGA,raw\n$GPRMC,raw\n',gpsRawAge:123,gps:{valid:false},sim:{ready:true,status:'SIM ready',model:'SIM800',imei:'123'},number:'123',contacts:[{name:'<script>',number:'123'}]};
let requests=[],timers=[];
function Request(){requests.push(this);this.open=()=>{};this.send=()=>{};}
const ctx={document:{getElementById:id=>nodes[id]??=(el()),querySelector:id=>nodes[id.slice(1)]??=el(),createElement(){return {set textContent(s){this.innerHTML=s.replace(/&/g,'&amp;').replace(/</g,'&lt;').replace(/>/g,'&gt;');}};}},XMLHttpRequest:Request,setTimeout(fn){timers.push(fn);}};
vm.createContext(ctx);vm.runInContext(src.match(/<script>([\s\S]*?)<\/script>/)[1],ctx);
function reply(body){let r=requests.at(-1);r.status=200;r.responseText=body;r.onload();r.onloadend();}
reply(JSON.stringify(data));
  assert.equal(nodes.gpsraw.textContent,data.gpsRaw);assert.equal(nodes.signal.textContent,'Signal: '+data.signal);
  assert.equal(nodes.gpsdetails.open,true);assert(nodes.contacts.innerHTML.includes('&lt;script&gt;'));
assert.equal(nodes.up.textContent,'0:01:00');assert(nodes.connection.textContent.includes('Live'));
assert.equal(requests.length,1,'no overlapping requests');timers.shift()();
// A control byte in peripheral data previously broke JSON.parse and was silently swallowed.
reply(JSON.stringify(data).replace('SIM ready','SIM\x01ready'));
assert(nodes.connection.textContent.startsWith('Live update failed:'));
timers.shift()();data.uptime='0:01:02';reply(JSON.stringify(data));
assert.equal(nodes.up.textContent,'0:01:02');assert(nodes.connection.textContent.includes('Live'));
timers.shift()();requests.at(-1).ontimeout();requests.at(-1).onloadend();
assert(nodes.connection.textContent.includes('5 seconds'));assert.equal(timers.length,1);
timers.shift()();requests.at(-1).onerror();requests.at(-1).onloadend();assert(nodes.connection.textContent.includes('connection lost'));
// Escaped control bytes must decode without losing modem text or GPS line breaks.
timers.shift()();data.sim.model='SIM\u0000\u0001\b\f800';reply(JSON.stringify(data));
assert(nodes.siminfo.textContent.includes(data.sim.model));assert(nodes.connection.textContent.includes('Live'));
console.log('PASS: alarm boundaries/button holds; real dashboard rendering; invalid JSON, timeout and network error visibility; automatic recovery; control-byte and NMEA display.');

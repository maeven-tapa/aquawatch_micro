#include "../app_status_schedule.h"
#include <string>
#include <vector>
#include <functional>
#include <stdio.h>
#include <stdlib.h>

// Arduino/modem boundary mocks; the functions under test come from the sketch.
struct String : std::string {
  String() = default;
  using std::string::string;
  using std::string::operator=;
  String(const std::string &s) : std::string(s) {}
  String(unsigned long n) : std::string(std::to_string(n)) {}
  int indexOf(const char *s) const { auto i=find(s); return i==npos ? -1 : int(i); }
};
struct Contact { String name, number; };
enum AlarmCause { ALARM_CAUSE_NONE, ALARM_CAUSE_MANUAL_SOS, ALARM_CAUSE_TILT_DISTRESS, ALARM_CAUSE_ALARM_ONLY };
bool alarmActive=false, smsBusy=false, signalPending=false, smsPending=false, simReady=false, strobeState=false;
AlarmCause alarmCause=ALARM_CAUSE_NONE;
std::vector<Contact> appContacts, appStatusRecipients;
size_t appStatusNext=0, appStatusSent=0;
String appStatusMessage, blastMessage, simNetwork, lastSosStatus;
AppStatusSchedule appStatusSchedule;
unsigned long lastStrobeAt=0;
const int SIREN_RELAY_PIN=27, STROBE_RELAY_PIN=4, SIREN_ON=0, SIREN_OFF=1, STROBE_ON=1, STROBE_OFF=0;
uint32_t fakeNow=20000;
bool networkReady=true, freshGps=false, submitSucceeds=true;
int networkQueries=0, snapshots=0;
std::function<void()> duringRegistration;
struct Sent { String number, body; };
std::vector<Sent> sent;
uint32_t millis() { return fakeNow; }
bool hasFreshGpsFix() { return freshGps; }
void digitalWrite(int, int) {}
void addEvent(const String &) {}
void require(bool value, const char *message) {
  if (!value) { fprintf(stderr, "FAIL: %s\n", message); exit(1); }
}
String atCommand(const String &command, unsigned long timeout) {
  require(smsBusy, "registration poll owns modem");
  require(command=="AT+CREG?", "only query registration");
  networkQueries++; fakeNow+=uint32_t(timeout);
  if (duringRegistration) duringRegistration();
  return networkReady ? "+CREG: 0,1" : "+CREG: 0,0";
}
String statusSmsData() { return std::string("STATUS ")+std::to_string(++snapshots); }
bool sendSmsTo(const String &number, bool requireAlarm) {
  require(smsBusy, "status send owns modem");
  require(!requireAlarm, "routine status allowed without distress");
  sent.push_back({number, blastMessage}); fakeNow+=5000;
  return submitSucceeds;
}
#include "app_status_under_test.h"

void reset() {
  alarmActive=smsBusy=signalPending=smsPending=simReady=strobeState=false;
  alarmCause=ALARM_CAUSE_NONE;
  appContacts={{"First", "111"}};
  appStatusRecipients.clear(); appStatusNext=appStatusSent=0;
  appStatusMessage=""; blastMessage=""; sent.clear();
  appStatusSchedule=AppStatusSchedule(); fakeNow=20000;
  networkReady=submitSucceeds=true; freshGps=false;
  networkQueries=snapshots=0; duringRegistration=nullptr;
}
int main() {
  reset();
  sendAppStatus();
  require(sent.size()==1 && !alarmActive, "send at startup without distress or GPS fix");
  uint32_t completed=fakeNow;
  fakeNow=completed+3000; sendAppStatus();
  require(sent.size()==1, "must not repeat after three seconds");
  fakeNow=completed+179999; sendAppStatus();
  require(sent.size()==1, "wait full three minutes after batch");
  fakeNow=completed+180000; sendAppStatus();
  require(sent.size()==2, "repeat after three minutes");

  reset(); networkReady=false;
  sendAppStatus();
  require(sent.empty() && networkQueries==1, "no startup SMS without registration");
  completed=fakeNow;
  fakeNow=completed+9999; sendAppStatus();
  require(networkQueries==1, "registration retries are bounded");
  networkReady=true; fakeNow=completed+10000; sendAppStatus();
  require(sent.size()==1, "startup recovers when modem registers");

  reset(); appContacts.push_back({"Second", "222"});
  sendAppStatus();
  require(sent.size()==1 && !appStatusRecipients.empty(), "one routine recipient per loop");
  String snapshot=sent[0].body;
  setAlarm(true, ALARM_CAUSE_MANUAL_SOS); freshGps=true;
  sendAppStatus();
  require(sent.size()==1, "pending SOS takes priority between recipients");
  blastMessage="SOS body"; smsPending=false; // SOS handled by the main loop.
  setAlarm(false);
  sendAppStatus();
  require(sent.size()==2 && sent[1].number=="222", "cancelled alarm does not cancel routine status");
  require(sent[1].body==snapshot, "resume status with original body after SOS");
  require(appStatusRecipients.empty(), "batch completes after remaining recipient");
  completed=fakeNow;
  setAlarm(true, ALARM_CAUSE_ALARM_ONLY); sendAppStatus();
  require(sent.size()==2, "alarm transitions do not force extra routine texts");
  fakeNow=completed+180000; sendAppStatus();
  require(sent.size()==3, "routine status continues in alarm-only mode");

  reset();
  duringRegistration=[] { setAlarm(true, ALARM_CAUSE_MANUAL_SOS); freshGps=true; };
  sendAppStatus();
  require(sent.empty(), "SOS arriving during registration gets priority");
  duringRegistration=nullptr; smsPending=false;
  sendAppStatus();
  require(sent.size()==1 && networkQueries==1, "resume prepared batch after SOS");

  reset(); appContacts.clear(); sendAppStatus();
  require(networkQueries==0 && sent.empty(), "empty contact list does not use modem");
  appContacts.push_back({"Added", "333"});
  signalPending=true; sendAppStatus();
  require(networkQueries==0, "wait for outstanding modem query");
  signalPending=false; smsBusy=true; sendAppStatus();
  require(networkQueries==0, "wait for SMS transaction");
  smsBusy=false; submitSucceeds=false; sendAppStatus();
  completed=fakeNow; fakeNow+=3000; sendAppStatus();
  require(sent.size()==1, "failed submissions do not cause rapid retries");

  AppStatusSchedule schedule;
  require(schedule.due(0), "startup due even at zero uptime");
  const uint32_t nearWrap=UINT32_MAX-1000;
  schedule.batchFinished(nearWrap);
  require(!schedule.due(nearWrap+179999u), "interval survives timer wrap");
  require(schedule.due(nearWrap+180000u), "deadline survives timer wrap");
  schedule.batchFinished(5000000);
  require(!schedule.due(5000001), "no catch-up burst after long pause");
  puts("PASS: startup status; 3-minute cadence; no 3-second repeats; registration retry; SOS priority/resume; alarm cancellation/only; empty contacts; modem contention; failed submissions; timer wrap.");
}

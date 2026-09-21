#include <HardwareSerial.h>
#include <WiFi.h>
#include <WebServer.h>

#define BTN1 35
#define BTN2 36
#define BTN3 37

#define RELAY1 38
#define RELAY2 39

#define LED1 10
#define LED2 11
#define LED3 12
#define LED4 13
#define LED5 14

// SIM800L UART
HardwareSerial sim800(1);

// SIM800L pins
// SIM800L TX → ESP32-S3 GPIO 15
// SIM800L RX → ESP32-S3 GPIO 16
#define SIM800_RX 15
#define SIM800_TX 16

// Website server
WebServer server(80);

// Wi-Fi Access Point name
const char* apName = "aquawatch_webapp";

// Device info
String deviceTitle = "AGUAWEWATCH";
String deviceVersion = "v1.0.0";
String espModel = "";
String espSerialNumber = "";
String simModel = "Checking...";
String simIMEI = "Checking...";

// GSM receiver number
String phoneNumber = "09925283361";

// Fixed GPS location
String gpsLocation = "14.33207,120.95708";

// Change these if your relay module is ACTIVE LOW
#define RELAY_ON  HIGH
#define RELAY_OFF LOW

// Change to LOW if your LED wiring is ACTIVE LOW
#define LED_ON  HIGH
#define LED_OFF LOW

bool relay2State = false;
bool lastButtonState = LOW;

unsigned long lastDebounceTime = 0;
const unsigned long debounceDelay = 200;

// Relay 1 sequence control
bool sequenceRunning = false;
int sequenceStep = 0;
unsigned long stepStartTime = 0;

// LED random control
unsigned long lastLedRandomTime = 0;
const unsigned long ledRandomDelay = 300;

int ledPins[] = {LED1, LED2, LED3, LED4, LED5};
const int totalLeds = sizeof(ledPins) / sizeof(ledPins[0]);

// Relay 1 loop sequence:
// 1.5 sec ON → 0.5 sec OFF → 0.5 sec ON → 0.5 sec OFF → 0.5 sec ON → 0.5 sec OFF → repeat
bool relay1States[] = {
  true,
  false,
  true,
  false,
  true,
  false
};

unsigned long durations[] = {
  1500,
  500,
  500,
  500,
  500,
  500
};

const int totalSteps = sizeof(durations) / sizeof(durations[0]);

void setup() {
  Serial.begin(115200);
  sim800.begin(9600, SERIAL_8N1, SIM800_RX, SIM800_TX);

  delay(3000);

  randomSeed(micros());

  pinMode(BTN1, INPUT);
  pinMode(BTN2, INPUT);
  pinMode(BTN3, INPUT);

  pinMode(RELAY1, OUTPUT);
  pinMode(RELAY2, OUTPUT);

  for (int i = 0; i < totalLeds; i++) {
    pinMode(ledPins[i], OUTPUT);
    digitalWrite(ledPins[i], LED_OFF);
  }

  digitalWrite(RELAY1, RELAY_OFF);
  digitalWrite(RELAY2, RELAY_OFF);

  setupDeviceInfo();
  startWebsite();

  Serial.println("System ready.");
  Serial.println("Initializing SIM800L...");

  initSIM800();
  updateSIM800Info();

  Serial.println("Website ready.");
  Serial.println("Connect phone to Wi-Fi: aquawatch_webapp");
  Serial.println("Open: http://192.168.4.1");
}

void loop() {
  server.handleClient();

  bool buttonPressed =
    digitalRead(BTN1) == HIGH ||
    digitalRead(BTN2) == HIGH ||
    digitalRead(BTN3) == HIGH;

  if (buttonPressed && !lastButtonState) {
    if (millis() - lastDebounceTime > debounceDelay) {

      if (relay2State == false) {
        relay2State = true;
        digitalWrite(RELAY2, RELAY_ON);
        startRelay1Sequence();

        Serial.println("System ON. SMS loop started.");
      }
      else {
        relay2State = false;
        digitalWrite(RELAY2, RELAY_OFF);
        stopRelay1Sequence();

        Serial.println("System OFF. SMS loop stopped.");
      }

      lastDebounceTime = millis();
    }
  }

  lastButtonState = buttonPressed;

  if (sequenceRunning) {
    runRelay1Sequence();
  }

  runRandomLeds();

  // While system is ON, check GSM then send SMS repeatedly
  if (relay2State == true) {
    if (checkGSM()) {
      sendGPSMessage();
    } else {
      Serial.println("GSM check failed. Retrying...");
      delay(1000);
    }
  }
}

void setupDeviceInfo() {
  espModel = ESP.getChipModel();

  uint64_t chipid = ESP.getEfuseMac();
  char serialBuffer[20];
  sprintf(serialBuffer, "%04X%08X", (uint16_t)(chipid >> 32), (uint32_t)chipid);
  espSerialNumber = String(serialBuffer);
}

void startWebsite() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(apName); // Open Wi-Fi, no password

  IPAddress IP = WiFi.softAPIP();

  Serial.print("Access Point IP: ");
  Serial.println(IP);

  server.on("/", handleWebsite);
  server.on("/set-phone", handleSetPhone);
  server.on("/learn-us", handleLearnUs);
  server.on("/refresh-gsm", []() {
    updateSIM800Info();
    server.sendHeader("Location", "/");
    server.send(303);
  });

  server.begin();
}

void handleSetPhone() {
  String newPhone = server.arg("phone");

  if (newPhone.length() > 0) {
    phoneNumber = newPhone;
    Serial.print("Updated GSM receiver number to: ");
    Serial.println(phoneNumber);
  }

  server.sendHeader("Location", "/");
  server.send(303);
}

void handleLearnUs() {
  if (relay2State == false) {
    relay2State = true;
    digitalWrite(RELAY2, RELAY_ON);
    startRelay1Sequence();
    Serial.println("Learn Us pressed: System ON. SMS loop started.");
  } else {
    relay2State = false;
    digitalWrite(RELAY2, RELAY_OFF);
    stopRelay1Sequence();
    Serial.println("Learn Us pressed: System OFF. SMS loop stopped.");
  }

  server.sendHeader("Location", "/");
  server.send(303);
}

void handleWebsite() {
  String relay1Status = sequenceRunning ? "RUNNING LOOP SEQUENCE" : "OFF";
  String relay2Status = relay2State ? "ON" : "OFF";

  String html = "";

  html += "<!DOCTYPE html>";
  html += "<html>";
  html += "<head>";
  html += "<meta name='viewport' content='width=device-width, initial-scale=1.0'>";
  html += "<meta http-equiv='refresh' content='5'>";
  html += "<title>" + deviceTitle + "</title>";

  html += "<style>";
  html += "body{font-family:Arial;background:#020817;margin:0;padding:20px;color:#e5f6ff;}";
  html += ".box{max-width:650px;margin:auto;background:#071b33;padding:22px;border-radius:18px;box-shadow:0 6px 25px rgba(0,0,0,0.45);border:1px solid #12395c;}";
  html += "h1{text-align:center;color:#ffffff;margin-bottom:5px;letter-spacing:2px;}";
  html += ".sub{text-align:center;color:#8ecae6;margin-bottom:22px;font-size:14px;}";
  html += ".row{padding:13px;border-bottom:1px solid #12395c;}";
  html += ".label{font-weight:bold;color:#62d9ff;font-size:14px;}";
  html += ".value{margin-top:5px;font-size:16px;word-break:break-word;color:#f1faff;}";
  html += ".on{color:#38ff88;font-weight:bold;}";
  html += ".off{color:#ff5c5c;font-weight:bold;}";
  html += "a{color:#7ddcff;}";
  html += "a.btn, button.btn{display:block;text-align:center;background:#0b4f8a;color:white;text-decoration:none;padding:13px;border-radius:12px;margin-top:18px;font-weight:bold;border:none;cursor:pointer;}";
  html += "a.btn:hover, button.btn:hover{background:#1269b3;}";
  html += "input[type='tel']{width:100%;padding:10px;border-radius:10px;border:1px solid #12395c;background:#04172e;color:#f1faff;margin-top:5px;}";
  html += "</style>";

  html += "</head>";
  html += "<body>";
  html += "<div class='box'>";

  html += "<h1>" + deviceTitle + "</h1>";
  html += "<div class='sub'>AquaWatch Device Information Page</div>";

  html += "<div class='row'><div class='label'>Version</div><div class='value'>" + deviceVersion + "</div></div>";
  html += "<div class='row'><div class='label'>ESP32 Model</div><div class='value'>" + espModel + "</div></div>";
  html += "<div class='row'><div class='label'>ESP32 Serial Number</div><div class='value'>" + espSerialNumber + "</div></div>";
  html += "<div class='row'><div class='label'>SIM800L Model</div><div class='value'>" + simModel + "</div></div>";
  html += "<div class='row'><div class='label'>SIM800L IMEI</div><div class='value'>" + simIMEI + "</div></div>";
  html += "<div class='row'><div class='label'>GSM Receiver Number</div><div class='value'><form action='/set-phone' method='GET'><input type='tel' name='phone' value='" + phoneNumber + "' placeholder='Enter receiver number'><button type='submit' class='btn'>Update Receiver Number</button></form></div></div>";
  html += "<div class='row'><div class='label'>GPS Location</div><div class='value'>" + gpsLocation + "</div></div>";
  html += "<div class='row'><div class='label'>Google Maps</div><div class='value'><a href='https://maps.google.com/?q=" + gpsLocation + "'>Open Location</a></div></div>";

  html += "<div class='row'><div class='label'>Relay 1 Status</div><div class='value'>" + relay1Status + "</div></div>";

  html += "<div class='row'><div class='label'>Relay 2 Status</div><div class='value'>";
  if (relay2State) {
    html += "<span class='on'>ON</span>";
  } else {
    html += "<span class='off'>OFF</span>";
  }
  html += "</div></div>";

  html += "<a class='btn' href='/learn-us'>Learn Us</a>";
  html += "<a class='btn' href='/refresh-gsm'>Refresh GSM Info</a>";

  html += "</div>";
  html += "</body>";
  html += "</html>";

  server.send(200, "text/html", html);
}

void initSIM800() {
  sendATCommand("AT", 2000);
  sendATCommand("ATE0", 1000);              // Disable echo for cleaner response
  sendATCommand("AT+CMGF=1", 2000);         // SMS text mode
  sendATCommand("AT+CSCS=\"GSM\"", 2000);
  sendATCommand("AT+CNMI=1,2,0,0,0", 2000);

  Serial.println("SIM800L init done.");
}

void updateSIM800Info() {
  Serial.println("Reading SIM800L model and IMEI...");

  String modelResponse = sendATCommand("AT+CGMM", 3000);
  String imeiResponse = sendATCommand("AT+GSN", 3000);

  simModel = cleanATResponse(modelResponse);
  simIMEI = cleanATResponse(imeiResponse);

  if (simModel.length() == 0) {
    simModel = "Not detected";
  }

  if (simIMEI.length() == 0) {
    simIMEI = "Not detected";
  }

  Serial.print("SIM Model: ");
  Serial.println(simModel);

  Serial.print("SIM IMEI: ");
  Serial.println(simIMEI);
}

String cleanATResponse(String response) {
  response.replace("\r", "");
  response.replace("\n", " ");
  response.replace("OK", "");
  response.replace("ERROR", "");
  response.replace("AT+CGMM", "");
  response.replace("AT+GSN", "");
  response.trim();

  while (response.indexOf("  ") != -1) {
    response.replace("  ", " ");
  }

  return response;
}

bool checkGSM() {
  Serial.println("Checking SIM800L...");

  String at = sendATCommand("AT", 2000);
  if (at.indexOf("OK") == -1) {
    Serial.println("No response from SIM800L.");
    return false;
  }

  String sim = sendATCommand("AT+CPIN?", 3000);
  if (sim.indexOf("READY") == -1) {
    Serial.println("SIM not ready.");
    return false;
  }

  String signal = sendATCommand("AT+CSQ", 3000);
  Serial.println("Signal response:");
  Serial.println(signal);

  String reg = sendATCommand("AT+CREG?", 3000);
  Serial.println("Network response:");
  Serial.println(reg);

  if (reg.indexOf(",1") != -1 || reg.indexOf(",5") != -1) {
    Serial.println("GSM registered.");
    return true;
  }

  Serial.println("GSM not registered.");
  return false;
}

void sendGPSMessage() {
  Serial.println("Sending SMS...");

  sendATCommand("AT+CMGF=1", 2000);

  sim800.print("AT+CMGS=\"");
  sim800.print(phoneNumber);
  sim800.println("\"");

  String prompt = waitSIM800Response(5000);
  Serial.println(prompt);

  if (prompt.indexOf(">") == -1) {
    Serial.println("No SMS prompt > received.");
    return;
  }

  sim800.println("AQUAWATCH DISTRESS MESSAGE");
  sim800.print("long; lat: ");
  sim800.println(gpsLocation);
  sim800.println(getGyroData());
  sim800.println(getAccelData());

  sim800.write(26); // CTRL+Z

  String result = waitSIM800Response(20000);
  Serial.println(result);

  if (result.indexOf("+CMGS") != -1 || result.indexOf("OK") != -1) {
    Serial.println("SMS SENT SUCCESSFULLY.");
  } else {
    Serial.println("SMS FAILED.");
  }
}

String getGyroData() {
  float gyroX = random(-8, 9) * 0.1;
  float gyroY = random(-8, 9) * 0.1;
  float gyroZ = random(-8, 9) * 0.1;

  return String("Gyro X: ") + String(gyroX, 1) + "g Y: " + String(gyroY, 1) + "g Z: " + String(gyroZ, 1) + "g";
}

String getAccelData() {
  float accelX = random(-4, 5) * 0.1;
  float accelY = random(-4, 5) * 0.1;
  float accelZ = random(-4, 5) * 0.1;

  return String("Accel knots: X=") + String(accelX, 1) + " Y=" + String(accelY, 1) + " Z=" + String(accelZ, 1);
}

String sendATCommand(String command, unsigned long timeout) {
  clearSIM800Buffer();

  Serial.print("SEND: ");
  Serial.println(command);

  sim800.println(command);

  return waitSIM800Response(timeout);
}

String waitSIM800Response(unsigned long timeout) {
  String response = "";
  unsigned long startTime = millis();

  while (millis() - startTime < timeout) {
    server.handleClient();

    if (sequenceRunning) {
      runRelay1Sequence();
    }

    runRandomLeds();

    while (sim800.available()) {
      char c = sim800.read();
      response += c;
      Serial.write(c);
    }
  }

  Serial.println();
  return response;
}

void clearSIM800Buffer() {
  while (sim800.available()) {
    sim800.read();
  }
}

void startRelay1Sequence() {
  sequenceRunning = true;
  sequenceStep = 0;
  stepStartTime = millis();

  digitalWrite(RELAY1, relay1States[sequenceStep] ? RELAY_ON : RELAY_OFF);
}

void runRelay1Sequence() {
  if (millis() - stepStartTime >= durations[sequenceStep]) {
    sequenceStep++;

    if (sequenceStep >= totalSteps) {
      sequenceStep = 0;
    }

    digitalWrite(RELAY1, relay1States[sequenceStep] ? RELAY_ON : RELAY_OFF);
    stepStartTime = millis();
  }
}

void stopRelay1Sequence() {
  sequenceRunning = false;
  sequenceStep = 0;
  digitalWrite(RELAY1, RELAY_OFF);
}

void runRandomLeds() {
  if (millis() - lastLedRandomTime >= ledRandomDelay) {
    lastLedRandomTime = millis();

    for (int i = 0; i < totalLeds; i++) {
      int randomState = random(0, 2);
      digitalWrite(ledPins[i], randomState == 1 ? LED_ON : LED_OFF);
    }
  }
}

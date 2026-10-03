#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncTCP.h>
#include <ArduinoJson.h>
#include <ESPmDNS.h>
#include <pins.h>
#include <config.h>

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ---------------------- TOOLPATH ------------------------
struct Segment { float pitch_mm; float turns; };
Segment segments[MAX_SEGMENTS];
int segmentCount = 0;

float spindleRPM = 45.0;   // constant target spindle speed for the job, sent with the job upload

// ---------------------- STATE ------------------------
int currentSegment = 0;
unsigned long jobStartMillis = 0;
float cumulativeTurnsAtSegmentStart = 0;
unsigned long pausedAtMillis = 0;
float jobTimeSeconds = 0;

unsigned long lastStatusMillis = 0;
unsigned long homeBackoffStart = 0;
bool farLimitTriggered = false;
bool homeLimitTriggered = false;

enum SwitchDirs { FWD, REV, LEFT, RIGHT };
// state
enum MachineState { RUNNING, PAUSED, HOMING, IDLE, JOGGING, BACKING_OFF };

enum MachineActions { 
  START_JOB, 
  PAUSE_JOB, 
  RESUME_JOB, 
  STOP_JOB, 
  JOG_SPINDLE, 
  JOG_CARRIAGE,
  JOG_STOP,
  START_HOMING, 
  UPDATE_JOB,
  HANDLE_JOG_SWITCHES
};

MachineActions machineAction;
MachineState machineState = IDLE;

// ---------------------- FUNCTIONS ------------------------
void logPins() {
  int pins[] = {
    digitalRead(JOG_SPINDLE_FWD_PIN),
    digitalRead(JOG_SPINDLE_REV_PIN),
    digitalRead(JOG_CARRIAGE_LEFT_PIN),
    digitalRead(JOG_CARRIAGE_RIGHT_PIN),
    digitalRead(HOME_LIMIT_PIN),
    digitalRead(FAR_LIMIT_PIN)
  };

  Serial.printf("FWD_PIN: %d ", pins[0]);
  Serial.printf("REV_PIN: %d ", pins[1]);
  Serial.printf("LEFT_PIN: %d ", pins[2]);
  Serial.printf("RIGHT_PIN: %d ", pins[3]);
  Serial.printf("HLMT_PIN: %d ", pins[4]);
  Serial.printf("FLMT_PIN: %d\n", pins[5]);
}

bool state(MachineState queryState) {
  //Serial.printf("state: %d : %d\n", machineState, queryState);
  return machineState == queryState;
}

bool homeLimitHit() {
  return digitalRead(HOME_LIMIT_PIN) == LOW;
}

bool farLimitHit() {
  return digitalRead(FAR_LIMIT_PIN) == LOW;
}

bool canRun(MachineActions action) {
  bool canRun = true;
  switch (action) {
    case START_JOB:
    case JOG_SPINDLE:
    case JOG_CARRIAGE:
    case START_HOMING:
      if (
        !state(IDLE)
    ) canRun = false;
      break;
    case PAUSE_JOB:
      if (
        !state(RUNNING)
      ) canRun = false;
      break;
    case RESUME_JOB:
      if (
        !state(PAUSED)    
      ) canRun = false;
      break;
    case JOG_STOP:
      if (
        !state(JOGGING)
      ) canRun = false;
      break;
    case UPDATE_JOB:
      if (
        machineState != RUNNING || 
        machineState == PAUSED
      ) canRun = false;
      break;
    case HANDLE_JOG_SWITCHES:
      if (
        !state(IDLE) && 
        !state(JOGGING)
      ) canRun = false;
      break;
  }
  
  return canRun;
}

void setAxisRate(int channel, double freqHz) {
  //Serial.printf("setAxisRate ch=%d freq=%.1f\n", channel, freqHz);
  if (freqHz <= 0) {
    ledcWrite(channel, 0);  // force duty to 0 — actually stops pulses
  } else {
    ledcWriteTone(channel, freqHz);
  }
}

void applyLeadscrewRateForSegment(int segIdx) {
  float pitch = segments[segIdx].pitch_mm;
  double leadscrewOutputRPM = spindleRPM * (pitch / SCREW_LEAD_MM);
  double leadscrewMotorFreq = (leadscrewOutputRPM / 60.0) * LEADSCREW_GEAR_RATIO
                              * MOTOR_STEPS_PER_REV * MICROSTEPPING;
  
  digitalWrite(LEADSCREW_DIR_PIN, FAR_DIR);   // carriage advances one direction for the whole job
  setAxisRate(LEADSCREW_CH, leadscrewMotorFreq);

  Serial.printf("applyLeadscrewRateForSegment seg=%d pitch=%.3f leadscrewMotorFreq=%.1f\n",
                segIdx, pitch, leadscrewMotorFreq);
}

// ----------- JOBS -----------
void startJob() {
  if (!canRun(START_JOB)) return;
  Serial.println("starting job");
  if (segmentCount == 0 || state(RUNNING) || farLimitTriggered) return;
  ledcSetup(SPINDLE_CH, 1, 10);
  ledcSetup(LEADSCREW_CH, 1, 10);
  currentSegment = 0;
  cumulativeTurnsAtSegmentStart = 0;
  jobTimeSeconds = 0;
  jobStartMillis = millis();
  machineState = RUNNING;

  double spindleMotorFreq = (spindleRPM / 60.0) * SPINDLE_GEAR_RATIO
                            * MOTOR_STEPS_PER_REV * MICROSTEPPING;
  
  Serial.printf("spindleRPM=%.1f\n", spindleRPM);
  Serial.printf("spindleMotorFreq=%.1f\n", spindleMotorFreq);
  Serial.printf("leadscrewMotorFreq=%.1f\n", (spindleRPM * (segments[0].pitch_mm / SCREW_LEAD_MM) / 60.0) * LEADSCREW_GEAR_RATIO * MOTOR_STEPS_PER_REV * MICROSTEPPING);
  Serial.printf("segmentCount=%d\n", segmentCount);
  Serial.printf("machineState: %d\n", machineState);
  
  digitalWrite(SPINDLE_DIR_PIN, SPINDLE_CW);
  setAxisRate(SPINDLE_CH, spindleMotorFreq);

  applyLeadscrewRateForSegment(0);
}

void pauseJob() {
  if (!canRun(PAUSE_JOB)) return;
  machineState = PAUSED;
  pausedAtMillis = millis();
  setAxisRate(SPINDLE_CH, 0);
  setAxisRate(LEADSCREW_CH, 0);
  Serial.printf("job paused: %d\n", machineState);
}

void resumeJob() {
  if (!canRun(RESUME_JOB)) return;
  jobStartMillis += millis() - pausedAtMillis;  // shift the clock so elapsed-time math stays correct
  machineState = RUNNING;

  double spindleMotorFreq = (spindleRPM / 60.0) * SPINDLE_GEAR_RATIO
                            * MOTOR_STEPS_PER_REV * MICROSTEPPING;
  setAxisRate(SPINDLE_CH, spindleMotorFreq);
  applyLeadscrewRateForSegment(currentSegment);
  Serial.printf("job resumed: %d\n", machineState);
}

void stopJob() {
  if (!canRun(STOP_JOB)) return;
  machineState = IDLE;
  setAxisRate(SPINDLE_CH, 0);
  setAxisRate(LEADSCREW_CH, 0);
  Serial.printf("Job Stopped: %d\n", machineState);
}

void updateJob() {
  if (!canRun(UPDATE_JOB)) return;

  if (farLimitTriggered) {   // safety cutoff if wired
    Serial.println("stopJob: far limit triggered");
    farLimitTriggered = true;
    stopJob();
    return;
  }

  float elapsedSec = (millis() - jobStartMillis) / 1000.0;
  float totalTurnsElapsed = elapsedSec * (spindleRPM / 60.0);
  float turnsIntoSegment = totalTurnsElapsed - cumulativeTurnsAtSegmentStart;
  static unsigned long lastDbg = 0;
  if (millis() - lastDbg > 1000) {
    lastDbg = millis();
    Serial.printf("elapsed=%.2f totalTurns=%.2f intoSeg=%.2f target=%.2f seg=%d\n",
                  elapsedSec, totalTurnsElapsed, turnsIntoSegment,
                  segments[currentSegment].turns, currentSegment);
    jobTimeSeconds += elapsedSec;
  }
  if (turnsIntoSegment >= segments[currentSegment].turns) {
    cumulativeTurnsAtSegmentStart += segments[currentSegment].turns;
    currentSegment++;
    if (currentSegment >= segmentCount) {
      Serial.printf("Job complete: %.1f seconds elapsed\n", jobTimeSeconds);
      stopJob();
      return;
    }
    applyLeadscrewRateForSegment(currentSegment);
  }
}

// ----------- JOGGING -----------
void jogStop() {
  if (!canRun(JOG_STOP)) return;
  setAxisRate(SPINDLE_CH, 0);
  setAxisRate(LEADSCREW_CH, 0);
  machineState = IDLE;
  //Serial.printf("jogStop: %d\n", machineState);
}

void jogSpindle(bool forward) {
  if (!canRun(JOG_SPINDLE)) return;
  digitalWrite(SPINDLE_DIR_PIN, forward ? SPINDLE_CW : SPINDLE_CCW);
  setAxisRate(SPINDLE_CH, JOG_FREQ_HZ);
  machineState = JOGGING;
  Serial.printf("jogSpindle: %s at %.1f Hz : %d\n", forward ? "forward" : "reverse", JOG_FREQ_HZ, machineState);
}

void jogCarriage(bool rightward) {
  if (!canRun(JOG_CARRIAGE)) return;
  digitalWrite(LEADSCREW_DIR_PIN, rightward ? LEADSCREW_CW : LEADSCREW_CCW);
  setAxisRate(LEADSCREW_CH, JOG_FREQ_HZ);
  machineState = JOGGING;
  Serial.printf("jogCarriage: %s at %.1f Hz : %d\n", rightward ? "rightward" : "leftward", JOG_FREQ_HZ, machineState);
}

void handleJogSwitches() {
  if (!canRun(HANDLE_JOG_SWITCHES)) return;
  int swPins[4] = {
    digitalRead(JOG_SPINDLE_FWD_PIN),
    digitalRead(JOG_SPINDLE_REV_PIN),
    digitalRead(JOG_CARRIAGE_LEFT_PIN),
    digitalRead(JOG_CARRIAGE_RIGHT_PIN)
  };
  int switchStates[4] = {
    digitalRead(JOG_SPINDLE_FWD_PIN) == LOW ? 1 : 0,
    digitalRead(JOG_SPINDLE_REV_PIN) == LOW ? 1 : 0,
    digitalRead(JOG_CARRIAGE_LEFT_PIN) == LOW ? 1 : 0,
    digitalRead(JOG_CARRIAGE_RIGHT_PIN) == LOW ? 1 : 0
  };
  
  SwitchDirs switchIndexes[4] = {FWD, REV, LEFT, RIGHT};
  int pressedSum = 0;
  int pressedIndx = -1;
  for (int i = 0; i < 4; i++) {
    pressedSum += switchStates[i];
    if (switchStates[i] == 1) {
      pressedIndx = i;
    }
  }
  
  //Serial.printf("switchStates: %d %d %d %d\n", swPins[0], swPins[1], swPins[2], swPins[3]);
  SwitchDirs switchDir;
  if (pressedIndx >= 0) {
    switchDir = switchIndexes[pressedIndx];
  }
  if (
    pressedSum != 1 ||
    switchDir == RIGHT && farLimitTriggered
  ) {
    jogStop();
    return;  // ignore if multiple switches pressed at once or no buttons pressed
  }
  
  if (state(JOGGING)) return;
  switch (switchDir) {
    case FWD:
      jogSpindle(true);
      break;
    case REV:
      jogSpindle(false);
      break;
    case LEFT :
      jogCarriage(false);
      break;
    case RIGHT:
      jogCarriage(true);
      break;
    default:
      jogStop();
      break;
  }
}
// ----------- LIMIT SWITCHES -----------
void handleLimitSwitches() {
  if (farLimitHit() && !farLimitTriggered) {
    Serial.println("far limit switch triggered");
    farLimitTriggered = true;
  } else if (!farLimitHit() && farLimitTriggered) {
    Serial.println("far limit switch cleared");
    farLimitTriggered = false;
  }

  if (homeLimitHit() && !homeLimitTriggered) {
    Serial.println("home limit switch triggered");
    homeLimitTriggered = true;
  } else if (!homeLimitHit() && homeLimitTriggered) {
    Serial.println("home limit switch cleared");
    homeLimitTriggered = false;
  }
}
// ----------- HOMING -----------
void startHoming() {
  Serial.println("startHoming called");
  if (!canRun(START_HOMING)) return;
  digitalWrite(LEADSCREW_DIR_PIN, HOME_DIR);   // toward home switch — flip if it homes the wrong way
  setAxisRate(LEADSCREW_CH, HOMING_FREQ_HZ);
  machineState = HOMING;
  Serial.printf("homing: seeking home limit — %s at %.1f Hz\n", HOME_DIR == LEADSCREW_CCW ? "CCW" : "CW", HOMING_FREQ_HZ);
}

void updateHoming() {
  if (state(HOMING) && homeLimitTriggered) {
    digitalWrite(LEADSCREW_DIR_PIN, FAR_DIR);
    setAxisRate(LEADSCREW_CH, HOMING_FREQ_HZ / 2);
    machineState = BACKING_OFF;
    homeBackoffStart = millis();          // now a safety timeout, not the stop condition
  }

  if (state(BACKING_OFF)) {
    if (!homeLimitTriggered) {            // switch released — that's our zero
      setAxisRate(LEADSCREW_CH, 0);
      machineState = IDLE;
      Serial.println("homing: complete");
    } else if (millis() - homeBackoffStart > BACKOFF_TIMEOUT_MS) {
      setAxisRate(LEADSCREW_CH, 0);
      machineState = IDLE;
      Serial.println("homing: FAILED — switch never cleared");
    }
  }
}

// ---------- Websocket handler ------------
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  
  if (type == WS_EVT_CONNECT) {
      Serial.printf("Client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
  }
  if (type != WS_EVT_DATA) return;
  String msg = String((char*)data).substring(0, len);
  Serial.println("ws msg: " + msg);
  
  if (msg.startsWith("{")) {
    // JSON payload — parse as job upload
    DynamicJsonDocument doc(4096);
    if (deserializeJson(doc, msg)) {
      client->text("{\"type\":\"ack\",\"status\":\"error\",\"msg\":\"bad json\"}");
      return;
    }
    if (state(RUNNING) || state(PAUSED)) {
      client->text("{\"type\":\"ack\",\"status\":\"error\",\"msg\":\"job running\"}");
      return;
    }
    if (doc.containsKey("spindle_rpm")) spindleRPM = doc["spindle_rpm"];
    segmentCount = 0;
    for (JsonObject s : doc["segments"].as<JsonArray>()) {
      if (segmentCount >= MAX_SEGMENTS) break;
      segments[segmentCount].pitch_mm = s["pitch_mm"];
      segments[segmentCount].turns    = s["turns"];
      segmentCount++;
    }
    client->text("{\"type\":\"ack\",\"status\":\"ok\"}");
    return;
  }

  if (msg == "JOG_SPINDLE_FWD_START") jogSpindle(true);
  else if (msg == "JOG_SPINDLE_REV_START") jogSpindle(false);
  else if (msg == "JOG_CARRIAGE_LEFT_START") jogCarriage(false);
  else if (msg == "JOG_CARRIAGE_RIGHT_START") jogCarriage(true);
  else if (msg == "JOG_STOP") jogStop();
  else if (msg == "HOME") startHoming();
  else if (msg == "RUN") startJob();
  else if (msg == "PAUSE") pauseJob();
  else if (msg == "RESUME") resumeJob();
  else if (msg == "STOP") stopJob();
}

void broadcastStatus() {
  if (millis() - lastStatusMillis < 200) return;
  lastStatusMillis = millis();

  StaticJsonDocument<256> doc;
  doc["running"] = state(RUNNING);
  doc["paused"] = state(PAUSED);
  doc["segment"] = currentSegment;
  doc["segmentCount"] = segmentCount;
  if (state(RUNNING) && !state(PAUSED)) {
    float elapsedSec = (millis() - jobStartMillis) / 1000.0;
    doc["turnsElapsed"] = elapsedSec * (spindleRPM / 60.0);
  }
  String out;
  serializeJson(doc, out);
  ws.textAll(out);
}

// ---------------------- PIN SETUP ------------------------
void setupMotorPins() {
  pinMode(SPINDLE_STEP_PIN, OUTPUT);
  pinMode(SPINDLE_DIR_PIN, OUTPUT);
  pinMode(LEADSCREW_STEP_PIN, OUTPUT);
  pinMode(LEADSCREW_DIR_PIN, OUTPUT);
  pinMode(SPINDLE_EN_PIN, OUTPUT);
  pinMode(LEADSCREW_EN_PIN, OUTPUT);
}
void enableSteppers() {
  digitalWrite(SPINDLE_EN_PIN, LOW);
  digitalWrite(LEADSCREW_EN_PIN, LOW);
}

void setupJogPins() {
  pinMode(JOG_SPINDLE_FWD_PIN, INPUT_PULLUP);
  pinMode(JOG_SPINDLE_REV_PIN, INPUT_PULLUP);
  pinMode(JOG_CARRIAGE_LEFT_PIN, INPUT_PULLUP);
  pinMode(JOG_CARRIAGE_RIGHT_PIN, INPUT_PULLUP);
}
void setupLimitPins() {
  pinMode(HOME_LIMIT_PIN, INPUT_PULLUP);
  pinMode(FAR_LIMIT_PIN, INPUT_PULLUP);
}

void setupStepperTimers() {
  ledcSetup(SPINDLE_CH,    1, 10);
  ledcSetup(LEADSCREW_CH,  1, 10);
  ledcAttachPin(SPINDLE_STEP_PIN, SPINDLE_CH);
  ledcAttachPin(LEADSCREW_STEP_PIN, LEADSCREW_CH);
}

void startWifi() {

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED) delay(200);
  MDNS.begin(MDNS_NAME);
  Serial.println(WiFi.localIP());
}

void startServer() {
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
  server.begin();
}


void setup() {
  Serial.begin(115200);
  logPins();
  // stepper pins
  setupMotorPins();
  // jog pins
  setupJogPins();
  // limit switch pins
  setupLimitPins();
  // enable the steppers (active: LOW)
  enableSteppers();
  delay(100);  // let the drivers wake up

  setupStepperTimers();

  if (farLimitHit()) {
    Serial.println("far limit switch triggered at startup — please clear it before running a job");
    farLimitTriggered = true;
  }
  
  logPins();
  startWifi();

  startServer();
  logPins();
}

void loop() {
  handleLimitSwitches();
  handleJogSwitches();
  updateHoming();
  updateJob();
  broadcastStatus();
  
}
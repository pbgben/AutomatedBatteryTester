#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// ----------------- Servo channels -----------------
int moverServo     = 0;
int contactorServo = 1;

// ----------------- Servo PWM values -----------------
int contactOpen  = 160;
int contactClose = 300;

// Continuous rotation servo "speeds" (calibrate for your servo)
int moveFast = 450;
int moveSlow = 410;
int moveStop = 375;

// ----------------- Switch pins -----------------
int homeSwitch   = 30;  // index switch: OPENS at notch (alignment)
int detectSwitch = 31;  // cells present detect (active LOW assumed)

// ----------------- Relay output pins -----------------
int relay1Pin = 22;
int relay2Pin = 23;

// ----------------- Behavior tuning -----------------
const unsigned long debounceMs      = 30;
const unsigned long homeTimeoutMs   = 12000;
const unsigned long stepTimeoutMs   = 8000;
const unsigned long unloadTimeoutMs = 20000; // safety timeout for unload
const unsigned long settleMs        = 150;
const unsigned long reconnectGapMs  = 80;

// NEW: require BOTH switches open continuously for this long to stop during UNLOAD
const unsigned long unloadBothOpenHoldMs = 1000;

// ----------------- IO state helpers -----------------
// Assumes switches wired to GND when "closed/active" and using INPUT_PULLUP.
// CLOSED => LOW, OPEN => HIGH.
bool isHomeClosed()   { return digitalRead(homeSwitch) == LOW; }
bool isDetectClosed() { return digitalRead(detectSwitch) == LOW; }

// "Aligned" means home switch OPEN at the notch
bool isAligned() { return !isHomeClosed(); }

// ----------------- Actuator helpers -----------------
void setMover(int pwmVal) { pwm.setPWM(moverServo, 0, pwmVal); }
void setContactsOpen()    { pwm.setPWM(contactorServo, 0, contactOpen); }
void setContactsClosed()  { pwm.setPWM(contactorServo, 0, contactClose); }

// ----------------- Contact state helpers -----------------
enum ContactState { CONTACT_OPEN, CONTACT_CLOSED };
ContactState contactState = CONTACT_OPEN;

void doConnect()    { setContactsClosed(); contactState = CONTACT_CLOSED; }
void doDisconnect() { setContactsOpen();   contactState = CONTACT_OPEN;   }
void doReconnect()  { doDisconnect(); delay(reconnectGapMs); doConnect(); }

// ----------------- Relay helpers -----------------
enum RelayState { RELAY_OFF, RELAY_ON };
RelayState relay1State = RELAY_OFF;
RelayState relay2State = RELAY_OFF;

// If your relay board is active-LOW, swap these two constants.
const int RELAY_ON_LEVEL  = HIGH;
const int RELAY_OFF_LEVEL = LOW;

void relay1On()  { digitalWrite(relay1Pin, RELAY_ON_LEVEL);  relay1State = RELAY_ON; }
void relay1Off() { digitalWrite(relay1Pin, RELAY_OFF_LEVEL); relay1State = RELAY_OFF; }
void relay2On()  { digitalWrite(relay2Pin, RELAY_ON_LEVEL);  relay2State = RELAY_ON; }
void relay2Off() { digitalWrite(relay2Pin, RELAY_OFF_LEVEL); relay2State = RELAY_OFF; }

// ----------------- Debounce utilities -----------------
bool waitStableRead(bool (*fn)(), bool targetState, unsigned long stableForMs) {
  unsigned long start = millis();
  while (millis() - start < stableForMs) {
    if (fn() != targetState) start = millis();
    delay(1);
  }
  return true;
}

// Require BOTH switches open continuously for holdMs
bool waitBothOpenHold(unsigned long holdMs, unsigned long overallTimeoutMs) {
  unsigned long t0 = millis();
  unsigned long stableStart = 0;

  while (true) {
    bool bothOpen = (!isHomeClosed() && !isDetectClosed());

    if (bothOpen) {
      if (stableStart == 0) stableStart = millis();
      if (millis() - stableStart >= holdMs) return true;
    } else {
      stableStart = 0;
    }

    if (millis() - t0 > overallTimeoutMs) return false;
    delay(1);
  }
}

// ----------------- Motion primitives -----------------
bool homeRoutineForwardOnly() {
  unsigned long t0 = millis();
  setMover(moveSlow); // homing is slow only

  // If already aligned (OPEN), move forward until we leave notch (CLOSED)
  if (isAligned()) {
    while (isAligned()) {
      if (millis() - t0 > homeTimeoutMs) { setMover(moveStop); return false; }
      delay(1);
    }
    waitStableRead(isAligned, false, debounceMs);
  }

  // Move forward until next notch (OPEN) => aligned
  while (!isAligned()) {
    if (millis() - t0 > homeTimeoutMs) { setMover(moveStop); return false; }
    delay(1);
  }
  waitStableRead(isAligned, true, debounceMs);

  setMover(moveStop);
  delay(settleMs);
  return true;
}

bool stepToNextAlignedFast() {
  unsigned long t0 = millis();
  setMover(moveFast);

  // Leave current notch (OPEN->CLOSED)
  if (isAligned()) {
    while (isAligned()) {
      if (millis() - t0 > stepTimeoutMs) { setMover(moveStop); return false; }
      delay(1);
    }
    waitStableRead(isAligned, false, debounceMs);
  }

  // Enter next notch (CLOSED->OPEN)
  while (!isAligned()) {
    if (millis() - t0 > stepTimeoutMs) { setMover(moveStop); return false; }
    delay(1);
  }
  waitStableRead(isAligned, true, debounceMs);

  setMover(moveStop);
  delay(settleMs);
  return true;
}

// UNLOAD: ignore indexing; run forward fast until BOTH switches are OPEN
// and have remained OPEN continuously for 1 second (unloadBothOpenHoldMs).
bool unloadUntilBothOpenHeld() {
  doDisconnect();     // safety
  setMover(moveFast);

  bool ok = waitBothOpenHold(unloadBothOpenHoldMs, unloadTimeoutMs);

  setMover(moveStop);
  delay(settleMs);
  return ok;
}

// ----------------- Serial command handling -----------------
String cmdLine;

void printHelp() {
  Serial.println(F("Commands:"));
  Serial.println(F("  HELP              - show this help"));
  Serial.println(F("  STATUS            - print switch states + contact/relay states"));
  Serial.println(F("  HOME              - slow, forward-only home to aligned notch (contacts OPEN)"));
  Serial.println(F("  NEXT              - move to next aligned notch (fast) then RECONNECT contacts"));
  Serial.println(F("  CONNECT           - close contacts"));
  Serial.println(F("  DISCONNECT        - open contacts"));
  Serial.println(F("  RECONNECT         - open then close contacts"));
  Serial.println(F("  UNLOAD            - run forward fast until BOTH switches OPEN for 1s"));
  Serial.println(F("  R1 ON|OFF         - relay 1 control"));
  Serial.println(F("  R2 ON|OFF         - relay 2 control"));
  Serial.println(F("  STOP              - stop mover servo"));
}

void printStatus() {
  Serial.print(F("STATUS "));
  Serial.print(F("homeClosed="));    Serial.print(isHomeClosed() ? F("1") : F("0"));
  Serial.print(F(" aligned="));      Serial.print(isAligned() ? F("1") : F("0"));
  Serial.print(F(" detectClosed=")); Serial.print(isDetectClosed() ? F("1") : F("0"));
  Serial.print(F(" contacts="));     Serial.print(contactState == CONTACT_CLOSED ? F("CLOSED") : F("OPEN"));
  Serial.print(F(" r1="));           Serial.print(relay1State == RELAY_ON ? F("ON") : F("OFF"));
  Serial.print(F(" r2="));           Serial.print(relay2State == RELAY_ON ? F("ON") : F("OFF"));
  Serial.println();
}

bool startsWith(const String& s, const char* prefix) {
  return s.startsWith(prefix);
}

void handleRelayCommand(const String& s) {
  if (s == "R1 ON")  { relay1On();  Serial.println(F("OK R1 ON"));  return; }
  if (s == "R1 OFF") { relay1Off(); Serial.println(F("OK R1 OFF")); return; }
  if (s == "R2 ON")  { relay2On();  Serial.println(F("OK R2 ON"));  return; }
  if (s == "R2 OFF") { relay2Off(); Serial.println(F("OK R2 OFF")); return; }
  Serial.println(F("ERR RELAY_CMD (use: R1 ON|OFF, R2 ON|OFF)"));
}

void handleCommand(String s) {
  s.trim();
  s.toUpperCase();
  if (s.length() == 0) return;

  if (s == "HELP" || s == "?") { printHelp(); return; }
  if (s == "STATUS")           { printStatus(); return; }

  if (startsWith(s, "R1") || startsWith(s, "R2")) {
    handleRelayCommand(s);
    return;
  }

  if (s == "STOP") {
    setMover(moveStop);
    Serial.println(F("OK STOP"));
    return;
  }

  if (s == "CONNECT") {
    doConnect();
    Serial.println(F("OK CONNECT"));
    return;
  }

  if (s == "DISCONNECT") {
    doDisconnect();
    Serial.println(F("OK DISCONNECT"));
    return;
  }

  if (s == "RECONNECT") {
    doReconnect();
    Serial.println(F("OK RECONNECT"));
    return;
  }

  if (s == "HOME") {
    doDisconnect();
    bool ok = homeRoutineForwardOnly();
    Serial.println(ok ? F("OK HOME") : F("ERR HOME_TIMEOUT"));
    return;
  }

  if (s == "NEXT") {
    doDisconnect();
    bool ok = stepToNextAlignedFast();
    if (!ok) { Serial.println(F("ERR NEXT_TIMEOUT")); return; }
    doReconnect();
    Serial.println(F("OK NEXT"));
    return;
  }

  if (s == "UNLOAD") {
    bool ok = unloadUntilBothOpenHeld();
    Serial.println(ok ? F("OK UNLOAD") : F("ERR UNLOAD_TIMEOUT"));
    return;
  }

  Serial.println(F("ERR UNKNOWN_CMD (try HELP)"));
}

void serviceSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      handleCommand(cmdLine);
      cmdLine = "";
    } else {
      if (cmdLine.length() < 80) cmdLine += c;
    }
  }
}

// ----------------- Setup/Loop -----------------
void setup() {
  Serial.begin(9600);

  pinMode(homeSwitch, INPUT_PULLUP);
  pinMode(detectSwitch, INPUT_PULLUP);

  pinMode(relay1Pin, OUTPUT);
  pinMode(relay2Pin, OUTPUT);

  relay1Off();
  relay2Off();

  pwm.begin();
  pwm.setPWMFreq(60);

  delay(10);
  setMover(moveStop);
  doDisconnect();

  Serial.println(F("Ready. Type HELP for commands."));
}

void loop() {
  serviceSerial();
  // Fully serial-controlled.
}

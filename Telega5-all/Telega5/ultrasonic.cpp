// ultrasonic.cpp — перенесено из sensor_node v3 без изменения логики
#include "ultrasonic.h"
#include "config.h"

// ---------- настройки ----------
static long stopMM = 500;
static long hystMM = 200;
static long maxMM  = 1300;
static long lostMM = 1000;
static float airC  = 20.0;
static const unsigned long US_SLOT = 40;   // мс между пингами (датчики опрашиваются по очереди)

// ---------- эхо (прерывания) ----------
static volatile unsigned long echoStart[2] = {0, 0}, echoDur[2] = {0, 0};
static volatile bool echoDone[2] = {false, false};

static void IRAM_ATTR echoISR0() {
  if (digitalRead(PIN_ECHO[0])) echoStart[0] = micros();
  else { echoDur[0] = micros() - echoStart[0]; echoDone[0] = true; }
}
static void IRAM_ATTR echoISR1() {
  if (digitalRead(PIN_ECHO[1])) echoStart[1] = micros();
  else { echoDur[1] = micros() - echoStart[1]; echoDone[1] = true; }
}

// ---------- состояние ----------
static long usHist[2][3] = {{-1, -1, -1}, {-1, -1, -1}};
static uint8_t usIdx[2] = {0, 0};
static bool pending[2] = {false, false};
static bool stopState[2] = {false, false}, lostState[2] = {false, false};
static unsigned long clearSince[2] = {0, 0};
static long lastValid[2] = {0, 0};
static unsigned long lastValidT[2] = {0, 0};
static unsigned long lastPing = 0;
static uint8_t nextSide = 0;

static long median3(long a, long b, long c) {
  if (a > b) { long t = a; a = b; b = t; }
  if (b > c) { long t = b; b = c; c = t; }
  if (a > b) { long t = a; a = b; b = t; }
  return b;
}

bool usObstacle() { return stopState[0] || stopState[1] || lostState[0] || lostState[1]; }

static void updateStopPin() { digitalWrite(PIN_STOP, usObstacle() ? HIGH : LOW); }

static void event(unsigned long now, uint8_t s, const __FlashStringHelper *ev, long mm) {
  Serial.print(F("E,")); Serial.print(now); Serial.print(','); Serial.print(SIDE[s]); Serial.print(',');
  Serial.print(ev); Serial.print(','); Serial.println(mm);
}

static void usResult(unsigned long now, uint8_t s, long mm) {
  usHist[s][usIdx[s]] = mm; usIdx[s] = (usIdx[s] + 1) % 3;
  long med = median3(usHist[s][0], usHist[s][1], usHist[s][2]);
  Serial.print(F("U,")); Serial.print(now); Serial.print(','); Serial.print(SIDE[s]); Serial.print(',');
  Serial.print(mm); Serial.print(','); Serial.println(med);
  if (med > 0) { lastValid[s] = med; lastValidT[s] = now; }
  else if (lostMM > 0 && lastValid[s] > 0 && lastValid[s] < lostMM && now - lastValidT[s] < 500 && !lostState[s]) {
    lostState[s] = true; updateStopPin(); event(now, s, F("LOST"), lastValid[s]);
    lastValid[s] = 0;
  }
  bool near = (med > 0 && med < stopMM);
  if (near && !stopState[s]) { stopState[s] = true; updateStopPin(); event(now, s, F("STOP"), med); }
  if (stopState[s]) {
    bool far = (med < 0 || med > stopMM + hystMM);
    if (!far) clearSince[s] = 0;
    else if (clearSince[s] == 0) clearSince[s] = now;
    else if (now - clearSince[s] > 1000) {
      stopState[s] = false; clearSince[s] = 0; updateStopPin(); event(now, s, F("CLEAR"), med);
    }
  }
}

void usSetup() {
  for (uint8_t s = 0; s < 2; s++) {
    pinMode(PIN_TRIG[s], OUTPUT); digitalWrite(PIN_TRIG[s], LOW);
    pinMode(PIN_ECHO[s], INPUT);
  }
  pinMode(PIN_STOP, OUTPUT); digitalWrite(PIN_STOP, LOW);
  attachInterrupt(digitalPinToInterrupt(PIN_ECHO[0]), echoISR0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ECHO[1]), echoISR1, CHANGE);
}

void usUpdate(unsigned long now) {
  for (uint8_t s = 0; s < 2; s++) {
    if (echoDone[s]) {
      noInterrupts(); unsigned long d = echoDur[s]; echoDone[s] = false; interrupts();
      pending[s] = false;
      float c = 331.3 + 0.606 * airC;
      long mm = (long)(d * c / 2000.0);
      if (mm < 20 || mm > maxMM) mm = -1;
      usResult(now, s, mm);
    }
  }
  if (now - lastPing >= US_SLOT) {
    lastPing = now;
    uint8_t s = nextSide; nextSide ^= 1;
    if (pending[s]) usResult(now, s, -1);
    pending[s] = true;
    digitalWrite(PIN_TRIG[s], LOW);  delayMicroseconds(3);
    digitalWrite(PIN_TRIG[s], HIGH); delayMicroseconds(10);
    digitalWrite(PIN_TRIG[s], LOW);
  }
}

void usStatus() {
  for (uint8_t s = 0; s < 2; s++) {
    Serial.print(F("# HC-SR04 ")); Serial.print(SIDE[s]);
    Serial.print(F(": стоп ")); Serial.print(stopState[s] ? 1 : 0);
    Serial.print(F(", LOST ")); Serial.println(lostState[s] ? 1 : 0);
  }
  Serial.print(F("# стоп ")); Serial.print(stopMM); Serial.print(F(" мм, гист. ")); Serial.print(hystMM);
  Serial.print(F(", макс ")); Serial.print(maxMM); Serial.print(F(", LOST ")); Serial.print(lostMM);
  Serial.print(F(", T=")); Serial.print(airC); Serial.println(F(" C"));
}

bool usCommand(char c, long v, const char *arg) {
  switch (c) {
    case 'S': if (v >= 100 && v <= 3000) stopMM = v; usStatus(); return true;
    case 'H': if (v >= 50 && v <= 1000) hystMM = v; usStatus(); return true;
    case 'M': if (v >= 500 && v <= 4000) maxMM = v; usStatus(); return true;
    case 'K': if (v == 0 || (v >= 300 && v <= 4000)) lostMM = v; usStatus(); return true;
    case 'R':
      for (uint8_t i = 0; i < 2; i++) lostState[i] = false;
      updateStopPin(); Serial.println(F("# LOST снят")); return true;
    case 'T': airC = atof(arg); usStatus(); return true;
  }
  return false;
}

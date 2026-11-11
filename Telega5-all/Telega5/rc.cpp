// rc.cpp — перенесено из Telega4 без изменения логики
#include "rc.h"
#include "config.h"

static volatile uint16_t pStick = 1500, pTumb1 = 2000, pTumb2 = 2000;
static volatile uint32_t tStick = 0, tTumb1 = 0, tTumb2 = 0;

static void IRAM_ATTR isrStick() {
  uint32_t now = micros();
  if (digitalRead(RC_STICK_PIN)) tStick = now;
  else if (tStick) { uint32_t p = now - tStick; tStick = 0; if (p > 800 && p < 2200) pStick = p; }
}
static void IRAM_ATTR isrTumb1() {
  uint32_t now = micros();
  if (digitalRead(RC_TUMB1_PIN)) tTumb1 = now;
  else if (tTumb1) { uint32_t p = now - tTumb1; tTumb1 = 0; if (p > 800 && p < 2200) pTumb1 = p; }
}
static void IRAM_ATTR isrTumb2() {
  uint32_t now = micros();
  if (digitalRead(RC_TUMB2_PIN)) tTumb2 = now;
  else if (tTumb2) { uint32_t p = now - tTumb2; tTumb2 = 0; if (p > 800 && p < 2200) pTumb2 = p; }
}

void rcSetup() {
  pinMode(RC_STICK_PIN, INPUT); attachInterrupt(digitalPinToInterrupt(RC_STICK_PIN), isrStick, CHANGE);
  pinMode(RC_TUMB1_PIN, INPUT); attachInterrupt(digitalPinToInterrupt(RC_TUMB1_PIN), isrTumb1, CHANGE);
  pinMode(RC_TUMB2_PIN, INPUT); attachInterrupt(digitalPinToInterrupt(RC_TUMB2_PIN), isrTumb2, CHANGE);
}

uint16_t rcStick()  { return pStick; }
bool rcManual()     { return pTumb1 < 1400; }
bool rcAuto()       { return pTumb2 < 1400; }
bool rcTogglesOff() { return (pTumb1 >= 1400) && (pTumb2 >= 1400); }

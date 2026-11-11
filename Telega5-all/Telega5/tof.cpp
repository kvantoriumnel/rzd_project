// tof.cpp — перенесено из sensor_node v3 без изменения логики
#include "tof.h"
#include "config.h"
#include <Wire.h>
#include <VL53L1X.h>

static const uint8_t  TOF_ADDR[TOF_N]   = {0x30, 0x31, 0x32, 0x33};
static const char     TOF_KIND[TOF_N]   = {'H', 'H', 'G', 'G'};
static const uint8_t  TOF_SIDE[TOF_N]   = {0, 1, 0, 1};
static const uint16_t TOF_PERIOD[TOF_N] = {33, 33, 100, 100};          // H 30 Гц, G 10 Гц
static const uint32_t TOF_BUDGET[TOF_N] = {30000, 30000, 50000, 50000};

static VL53L1X tof[TOF_N];
static bool tofOk[TOF_N] = {false, false, false, false}, tofStream = true;
static unsigned long tofPoll[TOF_N] = {0, 0, 0, 0};

static void printName(uint8_t i) {
  Serial.print(F("# ToF ")); Serial.print(TOF_KIND[i]); Serial.print(' '); Serial.print(SIDE[TOF_SIDE[i]]);
}

static bool tofInitOne(uint8_t i) {
  pinMode(PIN_XSHUT[i], INPUT);      // отпустить XSHUT (подтяжка на модуле)
  delay(10);
  tof[i].setBus(&Wire);
  tof[i].setTimeout(200);
  printName(i);
  Serial.print(F(" (XSHUT GPIO")); Serial.print(PIN_XSHUT[i]); Serial.print(F(")"));
  tof[i].setAddress(0x29);
  if (!tof[i].init()) { Serial.println(F(": НЕТ ответа")); return false; }
  tof[i].setAddress(TOF_ADDR[i]);
  tof[i].setDistanceMode(VL53L1X::Short);
  tof[i].setMeasurementTimingBudget(TOF_BUDGET[i]);
  tof[i].setROISize(4, 4);
  tof[i].startContinuous(TOF_PERIOD[i]);
  tofPoll[i] = millis();
  Serial.print(F(": OK, 0x")); Serial.println(TOF_ADDR[i], HEX);
  return true;
}

static void tofInitAll() {
  for (uint8_t i = 0; i < TOF_N; i++) { pinMode(PIN_XSHUT[i], OUTPUT); digitalWrite(PIN_XSHUT[i], LOW); }
  delay(10);
  for (uint8_t i = 0; i < TOF_N; i++) tofOk[i] = tofInitOne(i);
}

void tofSetup() { tofInitAll(); }

void tofUpdate(unsigned long now) {
  for (uint8_t i = 0; i < TOF_N; i++) {
    if (!tofOk[i]) continue;
    if (now - tofPoll[i] < (unsigned long)TOF_PERIOD[i] - 3) continue;
    if (!tof[i].dataReady()) {
      if (now - tofPoll[i] > 4UL * TOF_PERIOD[i] + 200) {
        tofOk[i] = false;
        printName(i); Serial.println(F(" не отвечает — команда N X"));
      }
      continue;
    }
    uint16_t mm = tof[i].read(false);
    tofPoll[i] = now;
    if (!tofStream) continue;
    Serial.print(F("T,")); Serial.print(now); Serial.print(','); Serial.print(SIDE[TOF_SIDE[i]]); Serial.print(',');
    Serial.print(TOF_KIND[i]); Serial.print(','); Serial.print(mm); Serial.print(',');
    Serial.println((int)tof[i].ranging_data.range_status);
  }
}

void tofStatus() {
  for (uint8_t i = 0; i < TOF_N; i++) {
    printName(i); Serial.print(F(" 0x")); Serial.print(TOF_ADDR[i], HEX); Serial.print(' ');
    Serial.println(tofOk[i] ? F("OK") : F("НЕТ"));
  }
}

bool tofCommand(char c, long v) {
  switch (c) {
    case 'D': tofStream = (v == 1); return true;
    case 'X': tofInitAll(); tofStatus(); return true;   // перед вызовом моторы останавливаются (commands.cpp)
  }
  return false;
}

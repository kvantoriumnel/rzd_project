#include <hoverserial.h>
#include <Preferences.h>
#include <math.h>

// ================= ПИНЫ =================
#define HOVER_RX_PIN  16
#define HOVER_TX_PIN  17
#define RC_STICK_PIN  23
#define RC_TUMB1_PIN  15
#define RC_TUMB2_PIN  2

// ================= ПАРАМЕТРЫ =================
#define RC_MAX_SPEED    70
#define RC_MIN_SPEED    20
#define AUTO_SPEED_MIN  5
#define AUTO_SPEED_MAX  20
#define ODOM_STEP_MIN   100.0f
#define CAL_STOP_RPM    15.0f

HardwareSerial hoverSerial(2);
hoverserial hoverboard;
Preferences prefs;

// ================= КОНФИГ (NVS) =================
float wheelDiamMm = 230.0;
float calFactor   = 1.0;
float revFactor   = 1.0f;
int16_t autoSpeed = 7 ;
float odomStepMm  = 1000.0;

// ================= RC =================
volatile uint16_t pStick = 1500, pTumb1 = 2000, pTumb2 = 2000;
volatile uint32_t tStick = 0, tTumb1 = 0, tTumb2 = 0;

void IRAM_ATTR isrStick() {
  uint32_t now = micros();
  if (digitalRead(RC_STICK_PIN)) tStick = now;
  else if (tStick) { uint32_t p = now - tStick; tStick = 0; if (p > 800 && p < 2200) pStick = p; }
}
void IRAM_ATTR isrTumb1() {
  uint32_t now = micros();
  if (digitalRead(RC_TUMB1_PIN)) tTumb1 = now;
  else if (tTumb1) { uint32_t p = now - tTumb1; tTumb1 = 0; if (p > 800 && p < 2200) pTumb1 = p; }
}
void IRAM_ATTR isrTumb2() {
  uint32_t now = micros();
  if (digitalRead(RC_TUMB2_PIN)) tTumb2 = now;
  else if (tTumb2) { uint32_t p = now - tTumb2; tTumb2 = 0; if (p > 800 && p < 2200) pTumb2 = p; }
}

// ================= СОСТОЯНИЯ =================
enum Mode { MODE_STOP, MODE_COM, MODE_RC, MODE_AUTO, MODE_CALIB, MODE_CAL_DIST };
Mode mode = MODE_STOP;
Mode prevMode = MODE_STOP;
int16_t cmdSpeed = 0;
float currentRPM = 0;

float accumRevs = 0;
double rawRevsRun = 0;
double rawMm   = 0;
double accumMm = 0;
double nextMm  = 1000.0;
float targetCalibRevs = 0;
int   photoCount = 0;

unsigned long lastLoopMs = 0;
char lineBuf[64];
uint8_t lineLen = 0;

// ================= ВСПОМОГАТЕЛЬНЫЕ =================
void loadConfig() {
  prefs.begin("hb", true);
  wheelDiamMm = prefs.getFloat("diam", 230.0);
  calFactor   = prefs.getFloat("calf", 1.0);
  revFactor   = prefs.getFloat("revf", 1.0);
  autoSpeed   = prefs.getInt  ("aspd", 400);
  odomStepMm  = prefs.getFloat("ostp", 1000.0);
  prefs.end();
  if (autoSpeed < AUTO_SPEED_MIN) autoSpeed = AUTO_SPEED_MIN;
  if (autoSpeed > AUTO_SPEED_MAX) autoSpeed = AUTO_SPEED_MAX;
  if (odomStepMm < ODOM_STEP_MIN) odomStepMm = ODOM_STEP_MIN;
  if (calFactor < 0.001f) calFactor = 0.001f;
  if (calFactor > 100.0f) calFactor = 100.0f;
  if (revFactor < 0.01f)  revFactor = 0.01f;
  if (revFactor > 100.0f) revFactor = 100.0f;
  if (wheelDiamMm < 10.0f)   wheelDiamMm = 10.0f;
  if (wheelDiamMm > 2000.0f) wheelDiamMm = 2000.0f;
}
void saveCal()  { prefs.begin("hb", false); prefs.putFloat("calf", calFactor);   prefs.end(); }
void saveRev()  { prefs.begin("hb", false); prefs.putFloat("revf", revFactor);   prefs.end(); }
void saveDiam() { prefs.begin("hb", false); prefs.putFloat("diam", wheelDiamMm); prefs.end(); }
void saveAuto() { prefs.begin("hb", false); prefs.putInt  ("aspd", autoSpeed);   prefs.end(); }
void saveOdom() { prefs.begin("hb", false); prefs.putFloat("ostp", odomStepMm);  prefs.end(); }

void sendSpeed(int16_t spd) {
  hoverboard.setSpeed(spd);
  hoverboard.setSteer(0);
}
float getBattV() { return hoverboard.getBatteryVoltage() / 100.0f; }
float getTempC() { return hoverboard.getBoardTemperature() / 10.0f; }
bool togglesOff() { return (pTumb1 >= 1400) && (pTumb2 >= 1400); }

// ================= ПАРСЕР =================
void parseCommand(char* line) {
  while (*line == ' ') line++;
  if (*line == '\0') return;
  if (*line >= 'a' && *line <= 'z') *line -= 32;

  char* cmd  = strtok(line, " ");
  char* arg1 = strtok(NULL, " ");
  char* arg2 = strtok(NULL, " ");
  if (!cmd) return;

  if (strcmp(cmd, "STOP") == 0) {
    mode = MODE_STOP; cmdSpeed = 0; sendSpeed(0);
    Serial.println("STOPPED"); return;
  }
  if (strcmp(cmd, "S") == 0 && arg1) {
    mode = MODE_COM;
    cmdSpeed = constrain(atoi(arg1), -1000, 1000);
    Serial.print("COM SPEED="); Serial.println(cmdSpeed);
    return;
  }
  if (strcmp(cmd, "R") == 0 && arg1) {
    if (!togglesOff()) { Serial.println("R ignored: turn both toggles OFF"); return; }
    mode = MODE_CALIB;
    targetCalibRevs = fabs(atof(arg1));
    cmdSpeed = arg2 ? constrain(atoi(arg2), -1000, 1000) : 100;
    accumRevs = 0; rawRevsRun = 0;
    Serial.print("ROTATING "); Serial.print(targetCalibRevs);
    Serial.print(" units-revs at full speed "); Serial.print(cmdSpeed);
    Serial.println(" (informational only)");
    return;
  }
  if (strcmp(cmd, "K") == 0 && arg1) {
    float v = atof(arg1);
    if (v < 0.01f) v = 0.01f;
    if (v > 100.0f) v = 100.0f;
    revFactor = v; saveRev();
    Serial.print("OK REV_FACTOR="); Serial.println(revFactor, 5);
    return;
  }
  if (strcmp(cmd, "KR") == 0 && arg1) {
    float realRevs = fabs(atof(arg1));
    if (rawRevsRun < 0.01 || realRevs < 0.01) {
      Serial.println("KR FAIL (run R first)"); return;
    }
    float newFactor = realRevs / rawRevsRun;
    if (newFactor < 0.01f) newFactor = 0.01f;
    if (newFactor > 100.0f) newFactor = 100.0f;
    revFactor = newFactor; saveRev();
    Serial.print("KR OK. RF="); Serial.println(revFactor, 5);
    return;
  }
  if (strcmp(cmd, "D") == 0 && arg1) {
    float v = atof(arg1);
    if (v < 10.0f) v = 10.0f;
    if (v > 2000.0f) v = 2000.0f;
    wheelDiamMm = v; saveDiam();
    Serial.print("OK D="); Serial.println(wheelDiamMm);
    return;
  }
  if (strcmp(cmd, "A") == 0 && arg1) {
    autoSpeed = constrain(atoi(arg1), AUTO_SPEED_MIN, AUTO_SPEED_MAX); saveAuto();
    Serial.print("OK AUTO_SPEED="); Serial.println(autoSpeed);
    return;
  }
  if (strcmp(cmd, "M") == 0 && arg1) {
    float v = atof(arg1);
    if (v < ODOM_STEP_MIN) v = ODOM_STEP_MIN;
    odomStepMm = v; saveOdom();
    Serial.print("OK ODOM_STEP="); Serial.println(odomStepMm, 0);
    return;
  }
  if (strcmp(cmd, "RESET") == 0) {
    accumMm = 0; rawMm = 0; nextMm = odomStepMm; photoCount = 0;
    Serial.println("RESET OK");
    return;
  }

  if (strcmp(cmd, "C") == 0 && arg1) {
    if (strcmp(arg1, "ZERO") == 0) {
      accumMm = 0; nextMm = odomStepMm; photoCount = 0;
      Serial.println("ZERO OK (coord=0)");
      return;
    }
    if (strcmp(arg1, "START") == 0) {
      if (!togglesOff()) { Serial.println("C START ignored: turn both toggles OFF"); return; }
      mode = MODE_CAL_DIST;
      rawMm = 0;
      if (arg2) cmdSpeed = constrain(atoi(arg2), -1000, 1000);
      else      cmdSpeed = -autoSpeed;
      sendSpeed(cmdSpeed);   // сразу полная скорость, без ramp
      Serial.print("CAL_DIST START. speed="); Serial.print(cmdSpeed);
      Serial.println(" (same as AUTO). Drive known dist, then: C STOP <real_mm>");
      return;
    }
    if (strcmp(arg1, "STOP") == 0 && arg2) {
      if (mode != MODE_CAL_DIST) {
        Serial.println("C STOP ignored: not in CAL_DIST (run C START first)");
        return;
      }
      double realMm = fabs(atof(arg2));
      sendSpeed(0);
      if (rawMm < 1.0 || realMm < 1.0) {
        Serial.print("CAL_DIST FAIL. raw="); Serial.print(rawMm, 1);
        Serial.print(" real="); Serial.println(realMm, 1);
        mode = MODE_STOP; cmdSpeed = 0; return;
      }
      float newCal = (float)(realMm / rawMm);
      calFactor = calFactor * newCal;
      if (calFactor < 0.001f) calFactor = 0.001f;
      if (calFactor > 100.0f) calFactor = 100.0f;
      saveCal();
      Serial.print("CAL_DIST OK. raw="); Serial.print(rawMm, 1);
      Serial.print(" real="); Serial.print(realMm, 1);
      Serial.print(" newCal="); Serial.println(calFactor, 6);
      Serial.print("Ratio this run = "); Serial.println(newCal, 4);
      mode = MODE_STOP; cmdSpeed = 0;
      return;
    }
    if (strcmp(arg1, "RESET") == 0) {
      calFactor = 1.0f; saveCal();
      Serial.println("CAL RESET (calFactor=1.0)");
      return;
    }
    if (strcmp(arg1, "INFO") == 0) {
      Serial.print("CAL="); Serial.print(calFactor, 6);
      Serial.print(" RF="); Serial.print(revFactor, 5);
      Serial.print(" D="); Serial.print(wheelDiamMm, 1);
      Serial.print("mm SPEED="); Serial.print(autoSpeed);
      Serial.print(" STEP="); Serial.print(odomStepMm, 0);
      Serial.print("mm DIST="); Serial.print(accumMm, 0);
      Serial.print("mm RAW="); Serial.print(rawMm, 0);
      Serial.println("mm");
      return;
    }
    Serial.println("C: START [speed] | STOP <real_mm> | ZERO | RESET | INFO");
    return;
  }

  Serial.println("Cmd: S|R|K|KR|D|A|M|STOP|RESET|C");
}

// ================= SETUP =================
void setup() {
  Serial.begin(115200);
  delay(200);
  loadConfig();
  nextMm = odomStepMm;

  hoverSerial.begin(115200, SERIAL_8N1, HOVER_RX_PIN, HOVER_TX_PIN);
  hoverboard.begin(hoverSerial);

  pinMode(RC_STICK_PIN, INPUT); attachInterrupt(digitalPinToInterrupt(RC_STICK_PIN), isrStick, CHANGE);
  pinMode(RC_TUMB1_PIN, INPUT); attachInterrupt(digitalPinToInterrupt(RC_TUMB1_PIN), isrTumb1, CHANGE);
  pinMode(RC_TUMB2_PIN, INPUT); attachInterrupt(digitalPinToInterrupt(RC_TUMB2_PIN), isrTumb2, CHANGE);

  lastLoopMs = millis();
  Serial.println("\n=== Hoverboard Control v6 ===");
  Serial.print("CAL="); Serial.print(calFactor, 6);
  Serial.print(" D="); Serial.print(wheelDiamMm, 1);
  Serial.print("mm SPEED="); Serial.print(autoSpeed);
  Serial.print(" STEP="); Serial.print(odomStepMm, 0); Serial.println("mm");
  Serial.println("AUTO/C START go full speed immediately. No ramp.");
  Serial.println("Path calibration: C START -> drive known dist -> C STOP <real_mm>");
}

// ================= LOOP =================
void loop() {
  hoverboard.housekeeping();

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) { lineBuf[lineLen] = '\0'; parseCommand(lineBuf); lineLen = 0; }
    } else if (lineLen < 63) lineBuf[lineLen++] = c;
  }

  if (!hoverboard.isConnected()) return;

  unsigned long now = millis();
  float dt = (now - lastLoopMs) / 1000.0f;
  if (dt <= 0) dt = 0.001f;
  if (dt > 0.5f) dt = 0.5f;
  lastLoopMs = now;

  int16_t rL = hoverboard.getSpeedLeft();
  int16_t rR = hoverboard.getSpeedRight();
  float units = (fabsf(rL) + fabsf(rR)) / 2.0f;
  currentRPM = units;

  float dRevs_units = (units / 60.0f) * dt;
  float d_mm_raw = dRevs_units * PI * wheelDiamMm;   // без revFactor

  bool rc       = (pTumb1 < 1400);
  bool autoMode = (pTumb2 < 1400);

  if (rc) {
    mode = MODE_RC;
    int16_t diff = pStick - 1500;
    if (abs(diff) < RC_MIN_SPEED) cmdSpeed = 0;
    else {
      int16_t sp = (int32_t)(abs(diff) - RC_MIN_SPEED) * RC_MAX_SPEED / (500 - RC_MIN_SPEED);
      cmdSpeed = (diff > 0) ? -sp : sp;
    }
    sendSpeed(cmdSpeed);
  }
  else if (autoMode) {
    mode = MODE_AUTO;
    if (prevMode != MODE_AUTO) {
      accumMm = 0; nextMm = odomStepMm; photoCount = 0;
      Serial.print("AUTO START (dist=0, step=");
      Serial.print(odomStepMm, 0); Serial.println("mm)");
    }
    cmdSpeed = -autoSpeed;      // сразу полная, без ramp
    sendSpeed(cmdSpeed);
    accumMm += d_mm_raw * calFactor;
    while (accumMm >= nextMm) {
      photoCount++;
      Serial.print("Take photo #"); Serial.print(photoCount);
      Serial.print(" | Distance="); Serial.print(accumMm, 0);
      Serial.println("mm");
      nextMm += odomStepMm;
    }
  }
  else if (mode == MODE_CAL_DIST) {
    rawMm += d_mm_raw;
    sendSpeed(cmdSpeed);        // держим ту же скорость, что и на старте
  }
  else if (mode == MODE_CALIB) {
    // Информационный прогон, к пути отношения не имеет.
    // Крутим на полной скорости, резко стопим, ждём физической остановки для точного rawRevsRun.
    rawRevsRun += dRevs_units;
    accumRevs  += dRevs_units * revFactor;
    if (accumRevs < targetCalibRevs) {
      sendSpeed(cmdSpeed);
    } else {
      sendSpeed(0);
      if (currentRPM < CAL_STOP_RPM) {
        mode = MODE_STOP;
        Serial.print("ROTATION DONE. RealRevs="); Serial.print(accumRevs, 3);
        Serial.print(" RawRevs="); Serial.print(rawRevsRun, 3);
        Serial.print(" (target="); Serial.print(targetCalibRevs, 2); Serial.println(")");
        Serial.println("NOTE: informational. For path use C START/C STOP.");
      }
    }
  }
  else {
    if (mode == MODE_RC || mode == MODE_AUTO || mode == MODE_CAL_DIST) {
      mode = MODE_STOP; cmdSpeed = 0; sendSpeed(0);
    }
    if (mode == MODE_COM) sendSpeed(cmdSpeed);
    else if (mode == MODE_STOP) sendSpeed(0);
  }
  prevMode = mode;

  static unsigned long lastLog = 0;
  if (now - lastLog >= 500) {
    lastLog = now;
    Serial.print("[");
    switch (mode) {
      case MODE_RC: Serial.print("RC"); break;
      case MODE_AUTO: Serial.print("AUTO"); break;
      case MODE_COM: Serial.print("COM"); break;
      case MODE_CALIB: Serial.print("CALIB"); break;
      case MODE_CAL_DIST: Serial.print("CAL_DIST"); break;
      default: Serial.print("STOP");
    }
    Serial.print("] Cmd="); Serial.print(cmdSpeed);
    Serial.print(" Units="); Serial.print(units, 0);
    if (mode == MODE_CALIB) {
      Serial.print(" Revs="); Serial.print(accumRevs, 3);
      Serial.print("/"); Serial.print(targetCalibRevs, 2);
    } else if (mode == MODE_CAL_DIST) {
      Serial.print(" Raw="); Serial.print(rawMm, 0); Serial.print("mm");
    } else {
      Serial.print(" Dist="); Serial.print(accumMm, 0); Serial.print("mm");
    }
    Serial.print(" CAL="); Serial.print(calFactor, 5);
    Serial.print(" Batt="); Serial.print(getBattV(), 2);
    Serial.print("V T="); Serial.print(getTempC(), 1); Serial.println("C");
  }

  delay(10);
}

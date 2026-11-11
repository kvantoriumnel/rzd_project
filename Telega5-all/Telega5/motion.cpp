// motion.cpp — перенесено из Telega4 v4. Изменения при объединении:
//  1) delay(10) в loop() заменён тактом 10 мс без блокировки (датчики опрашиваются между тактами);
//  2) при потере связи с платой гироскутера останавливается только движение, датчики продолжают работать;
//  3) препятствие от HC-SR04 (бывший вывод «стоп» на плату управления) теперь останавливает тележку
//     программно в режимах COM / AUTO / CAL_DIST; в ручном режиме RC — не вмешивается. Команда HOLD 0 отключает;
//  4) новая строка одометрии O (10 Гц) для привязки показаний датчиков к пройденному пути. Команда O 0 отключает.
#include "motion.h"
#include "config.h"
#include "rc.h"
#include "ultrasonic.h"
#include <hoverserial.h>
#include <Preferences.h>
#include <math.h>

// ================= ПАРАМЕТРЫ =================
#define RC_MAX_SPEED    700
#define RC_MIN_SPEED    100
#define AUTO_SPEED_MIN  200
#define AUTO_SPEED_MAX  1000
#define ODOM_STEP_MIN   100.0f

// === Калибровка оборотов ===
#define CAL_DECEL_ZONE  3.0f    // за сколько оборотов до цели начинать тормозить
#define CAL_MIN_SPEED   60      // минимальная команда на замедлении
#define CAL_STOP_RPM    15.0f   // ниже этого колесо считается остановленным

static const unsigned long TICK_MS = 10;     // такт управления (как delay(10) в Telega4)
static const unsigned long LOG_MS  = 500;    // строка состояния [MODE] …
static const unsigned long ODOM_MS = 100;    // строка одометрии O

static HardwareSerial hoverSerial(2);
static hoverserial hoverboard;
static Preferences prefs;

// ================= КОНФИГ (NVS) =================
static float wheelDiamMm = 230.0;
static float calFactor   = 1.0;
static float revFactor   = 1.0f;
static int16_t autoSpeed = 400;
static float odomStepMm  = 1000.0;

// ================= СОСТОЯНИЯ =================
enum Mode { MODE_STOP, MODE_COM, MODE_RC, MODE_AUTO, MODE_CALIB, MODE_CAL_DIST };
static Mode mode = MODE_STOP;
static Mode prevMode = MODE_STOP;
static int16_t cmdSpeed = 0;
static float currentRPM = 0;

static float accumRevs = 0;
static double rawRevsRun = 0;
static double rawMm   = 0;
static double accumMm = 0;
static double nextMm  = 1000.0;
static float targetCalibRevs = 0;
static int   photoCount = 0;
static float rateMmS = 0;

static unsigned long lastLoopMs = 0, lastTick = 0, lastLog = 0, lastOdom = 0;
static bool holdEnabled = true, holding = false, odomStream = true;

// ================= ВСПОМОГАТЕЛЬНЫЕ =================
static void loadConfig() {
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
static void saveCal()  { prefs.begin("hb", false); prefs.putFloat("calf", calFactor);   prefs.end(); }
static void saveRev()  { prefs.begin("hb", false); prefs.putFloat("revf", revFactor);   prefs.end(); }
static void saveDiam() { prefs.begin("hb", false); prefs.putFloat("diam", wheelDiamMm); prefs.end(); }
static void saveAuto() { prefs.begin("hb", false); prefs.putInt  ("aspd", autoSpeed);   prefs.end(); }
static void saveOdom() { prefs.begin("hb", false); prefs.putFloat("ostp", odomStepMm);  prefs.end(); }

static const char *modeName(Mode m) {
  switch (m) {
    case MODE_RC: return "RC";
    case MODE_AUTO: return "AUTO";
    case MODE_COM: return "COM";
    case MODE_CALIB: return "CALIB";
    case MODE_CAL_DIST: return "CAL_DIST";
    default: return "STOP";
  }
}

// Удержание по препятствию: в автоматических режимах команда скорости заменяется нулём
static int16_t applyHold(int16_t spd) {
  bool block = holdEnabled && usObstacle() &&
               (mode == MODE_AUTO || mode == MODE_COM || mode == MODE_CAL_DIST);
  if (block != holding) {
    holding = block;
    Serial.print(F("E,")); Serial.print(millis()); Serial.print(F(",M,"));
    Serial.println(block ? F("HOLD,1") : F("GO,0"));
  }
  return block ? 0 : spd;
}

static void sendSpeed(int16_t spd) {
  hoverboard.setSpeed(applyHold(spd));
  hoverboard.setSteer(0);
}
static float getBattV() { return hoverboard.getBatteryVoltage() / 100.0f; }
static float getTempC() { return hoverboard.getBoardTemperature() / 10.0f; }

void motionHalt() {
  mode = MODE_STOP; cmdSpeed = 0; sendSpeed(0);
}

// ================= ПАРСЕР (команды Telega4) =================
bool motionCommand(char *line) {
  while (*line == ' ') line++;
  if (*line == '\0') return false;
  if (*line >= 'a' && *line <= 'z') *line -= 32;

  char *cmd  = strtok(line, " ");
  char *arg1 = strtok(NULL, " ");
  char *arg2 = strtok(NULL, " ");
  if (!cmd) return false;

  if (strcmp(cmd, "STOP") == 0) {
    motionHalt();
    Serial.println("STOPPED"); return true;
  }
  if (strcmp(cmd, "S") == 0 && arg1) {
    mode = MODE_COM;
    cmdSpeed = constrain(atoi(arg1), -1000, 1000);
    Serial.print("COM SPEED="); Serial.println(cmdSpeed);
    return true;
  }
  if (strcmp(cmd, "R") == 0 && arg1) {
    if (!rcTogglesOff()) { Serial.println("R ignored: turn both toggles OFF"); return true; }
    mode = MODE_CALIB;
    targetCalibRevs = fabs(atof(arg1));
    cmdSpeed = arg2 ? constrain(atoi(arg2), -1000, 1000) : 100;
    accumRevs = 0; rawRevsRun = 0;
    Serial.print("ROTATING "); Serial.print(targetCalibRevs);
    Serial.print(" revs at speed "); Serial.print(cmdSpeed);
    Serial.print(" (RF="); Serial.print(revFactor, 4);
    Serial.println(") with deceleration ramp");
    return true;
  }
  if (strcmp(cmd, "K") == 0 && arg1) {
    float v = atof(arg1);
    if (v < 0.01f) v = 0.01f;
    if (v > 100.0f) v = 100.0f;
    revFactor = v; saveRev();
    Serial.print("OK REV_FACTOR="); Serial.println(revFactor, 5);
    return true;
  }
  if (strcmp(cmd, "KR") == 0 && arg1) {
    float realRevs = fabs(atof(arg1));
    if (rawRevsRun < 0.01 || realRevs < 0.01) {
      Serial.print("KR FAIL. rawRevsRun="); Serial.print(rawRevsRun, 4);
      Serial.print(" real="); Serial.println(realRevs, 4);
      return true;
    }
    float newFactor = realRevs / rawRevsRun;
    if (newFactor < 0.01f) newFactor = 0.01f;
    if (newFactor > 100.0f) newFactor = 100.0f;
    revFactor = newFactor; saveRev();
    Serial.print("KR OK. real="); Serial.print(realRevs, 3);
    Serial.print(" raw="); Serial.print(rawRevsRun, 3);
    Serial.print(" RF="); Serial.println(revFactor, 5);
    return true;
  }
  if (strcmp(cmd, "D") == 0 && arg1) {
    float v = atof(arg1);
    if (v < 10.0f) v = 10.0f;
    if (v > 2000.0f) v = 2000.0f;
    wheelDiamMm = v; saveDiam();
    Serial.print("OK D="); Serial.println(wheelDiamMm);
    return true;
  }
  if (strcmp(cmd, "A") == 0 && arg1) {
    autoSpeed = constrain(atoi(arg1), AUTO_SPEED_MIN, AUTO_SPEED_MAX); saveAuto();
    Serial.print("OK AUTO_SPEED="); Serial.println(autoSpeed);
    return true;
  }
  if (strcmp(cmd, "M") == 0 && arg1) {
    float v = atof(arg1);
    if (v < ODOM_STEP_MIN) v = ODOM_STEP_MIN;
    odomStepMm = v; saveOdom();
    Serial.print("OK ODOM_STEP="); Serial.println(odomStepMm, 0);
    return true;
  }
  if (strcmp(cmd, "RESET") == 0) {
    accumMm = 0; rawMm = 0; nextMm = odomStepMm; photoCount = 0;
    Serial.println("RESET OK");
    return true;
  }
  if (strcmp(cmd, "HOLD") == 0 && arg1) {          // новое: остановка по препятствию вкл/выкл
    holdEnabled = (atoi(arg1) == 1);
    Serial.print("OK HOLD="); Serial.println(holdEnabled ? 1 : 0);
    return true;
  }
  if (strcmp(cmd, "O") == 0 && arg1) {             // новое: поток одометрии O вкл/выкл
    odomStream = (atoi(arg1) == 1);
    Serial.print("OK ODOM_STREAM="); Serial.println(odomStream ? 1 : 0);
    return true;
  }

  if (strcmp(cmd, "C") == 0 && arg1) {
    if (strcmp(arg1, "ZERO") == 0) {
      accumMm = 0; nextMm = odomStepMm; photoCount = 0;
      Serial.println("ZERO OK (coord=0)");
      return true;
    }
    if (strcmp(arg1, "START") == 0) {
      if (!rcTogglesOff()) { Serial.println("C START ignored: turn both toggles OFF"); return true; }
      mode = MODE_CAL_DIST;
      rawMm = 0;
      if (arg2) cmdSpeed = constrain(atoi(arg2), -1000, 1000);
      else      cmdSpeed = -autoSpeed;
      sendSpeed(cmdSpeed);
      Serial.print("CAL_DIST START. speed="); Serial.print(cmdSpeed);
      Serial.println(". At mark: C STOP <real_mm>");
      return true;
    }
    if (strcmp(arg1, "STOP") == 0 && arg2) {
      if (mode != MODE_CAL_DIST) {
        Serial.println("C STOP ignored: not in CAL_DIST (run C START first)");
        return true;
      }
      double realMm = fabs(atof(arg2));
      sendSpeed(0);
      if (rawMm < 1.0 || realMm < 1.0) {
        Serial.print("CAL_DIST FAIL. raw="); Serial.print(rawMm, 1);
        Serial.print(" real="); Serial.println(realMm, 1);
        mode = MODE_STOP; cmdSpeed = 0; return true;
      }
      float newCal = (float)(realMm / rawMm);
      calFactor = calFactor * newCal;
      if (calFactor < 0.001f) calFactor = 0.001f;
      if (calFactor > 100.0f) calFactor = 100.0f;
      saveCal();
      Serial.print("CAL_DIST OK. raw="); Serial.print(rawMm, 1);
      Serial.print(" real="); Serial.print(realMm, 1);
      Serial.print(" calFactor="); Serial.println(calFactor, 6);
      mode = MODE_STOP; cmdSpeed = 0;
      return true;
    }
    if (strcmp(arg1, "RESET") == 0) {
      calFactor = 1.0f; saveCal();
      Serial.println("CAL RESET (calFactor=1.0)");
      return true;
    }
    if (strcmp(arg1, "INFO") == 0) { motionStatus(); return true; }
    Serial.println("C: START [speed] | STOP <real_mm> | ZERO | RESET | INFO");
    return true;
  }
  return false;
}

void motionStatus() {
  Serial.print("CAL="); Serial.print(calFactor, 6);
  Serial.print(" RF="); Serial.print(revFactor, 5);
  Serial.print(" D="); Serial.print(wheelDiamMm, 1);
  Serial.print("mm SPEED="); Serial.print(autoSpeed);
  Serial.print(" STEP="); Serial.print(odomStepMm, 0);
  Serial.print("mm DIST="); Serial.print(accumMm, 0);
  Serial.print("mm RAW="); Serial.print(rawMm, 0);
  Serial.println("mm");
  Serial.print("# движение: "); Serial.print(modeName(mode));
  Serial.print(", плата "); Serial.print(hoverboard.isConnected() ? "на связи" : "НЕТ СВЯЗИ");
  Serial.print(", HOLD="); Serial.print(holdEnabled ? 1 : 0);
  Serial.print(holding ? " (стоит: препятствие)" : "");
  Serial.print(", O="); Serial.println(odomStream ? 1 : 0);
}

// ================= SETUP =================
void motionSetup() {
  loadConfig();
  nextMm = odomStepMm;
  hoverSerial.begin(115200, SERIAL_8N1, HOVER_RX_PIN, HOVER_TX_PIN);
  hoverboard.begin(hoverSerial);
  rcSetup();
  lastLoopMs = lastTick = millis();

  Serial.println("\n=== Hoverboard Control v5 (Telega5) ===");
  motionStatus();
  Serial.println("R-cmd uses deceleration ramp. Use low speed: R 10 100");
  Serial.println("Example: R 10 100 -> count real -> KR <count>");
}

// ================= ТАКТ УПРАВЛЕНИЯ (бывший loop() Telega4) =================
static void motionTick(unsigned long now) {
  float dt = (now - lastLoopMs) / 1000.0f;
  if (dt <= 0) dt = 0.001f;
  if (dt > 0.5f) dt = 0.5f;
  lastLoopMs = now;

  int16_t rL = hoverboard.getSpeedLeft();
  int16_t rR = hoverboard.getSpeedRight();
  float units = (fabsf(rL) + fabsf(rR)) / 2.0f;
  currentRPM = units;

  float dRevs_units = (units / 60.0f) * dt;
  float dRevs_real  = dRevs_units * revFactor;
  float d_mm_real   = dRevs_real * PI * wheelDiamMm;
  rateMmS           = d_mm_real / dt;

  if (rcManual()) {
    mode = MODE_RC;
    int16_t diff = rcStick() - 1500;
    if (abs(diff) < RC_MIN_SPEED) cmdSpeed = 0;
    else {
      int16_t sp = (int32_t)(abs(diff) - RC_MIN_SPEED) * RC_MAX_SPEED / (500 - RC_MIN_SPEED);
      cmdSpeed = (diff > 0) ? -sp : sp;
    }
    sendSpeed(cmdSpeed);
  }
  else if (rcAuto()) {
    mode = MODE_AUTO;
    if (prevMode != MODE_AUTO) {
      accumMm = 0; nextMm = odomStepMm; photoCount = 0;
      Serial.print("AUTO START (dist=0, step=");
      Serial.print(odomStepMm, 0); Serial.println("mm)");
    }
    cmdSpeed = -autoSpeed;
    sendSpeed(cmdSpeed);
    accumMm += d_mm_real * calFactor;
    while (accumMm >= nextMm) {
      photoCount++;
      Serial.print("Take photo #"); Serial.print(photoCount);
      Serial.print(" | Distance="); Serial.print(accumMm, 0);
      Serial.println("mm");
      nextMm += odomStepMm;
    }
  }
  else if (mode == MODE_CAL_DIST) {
    rawMm += d_mm_real;
    sendSpeed(cmdSpeed);
  }
  else if (mode == MODE_CALIB) {
    // замедление + ожидание остановки
    rawRevsRun += dRevs_units;
    accumRevs  += dRevs_real;

    float remaining = targetCalibRevs - accumRevs;
    int16_t dir = (cmdSpeed >= 0) ? 1 : -1;
    int16_t absCmd = abs(cmdSpeed);

    if (remaining > CAL_DECEL_ZONE) {
      sendSpeed(dir * absCmd);
    } else if (remaining > 0.3f) {
      int16_t slow = (int16_t)(remaining / CAL_DECEL_ZONE * absCmd);
      if (slow < CAL_MIN_SPEED) slow = CAL_MIN_SPEED;
      sendSpeed(dir * slow);
    } else {
      sendSpeed(0);
      if (currentRPM < CAL_STOP_RPM) {
        mode = MODE_STOP;
        Serial.print("ROTATION DONE! RealRevs="); Serial.print(accumRevs, 3);
        Serial.print(" RawRevs="); Serial.print(rawRevsRun, 3);
        Serial.print(" (target="); Serial.print(targetCalibRevs, 3); Serial.println(")");
        Serial.println("If physical count differs by >2%, run: KR <real_revs>");
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

  if (now - lastLog >= LOG_MS) {
    lastLog = now;
    Serial.print("["); Serial.print(modeName(mode));
    Serial.print("] Cmd="); Serial.print(cmdSpeed);
    Serial.print(" Units="); Serial.print(units, 0);
    Serial.print(" Rate="); Serial.print(rateMmS, 0); Serial.print("mm/s");
    if (mode == MODE_CALIB) {
      Serial.print(" Revs="); Serial.print(accumRevs, 3);
      Serial.print("/"); Serial.print(targetCalibRevs, 2);
      Serial.print(" RawRevs="); Serial.print(rawRevsRun, 3);
    } else if (mode == MODE_CAL_DIST) {
      Serial.print(" Raw="); Serial.print(rawMm, 0); Serial.print("mm");
    } else {
      Serial.print(" Dist="); Serial.print(accumMm, 0); Serial.print("mm");
    }
    Serial.print(" CAL="); Serial.print(calFactor, 4);
    Serial.print(" RF="); Serial.print(revFactor, 3);
    Serial.print(" Batt="); Serial.print(getBattV(), 2);
    Serial.print("V T="); Serial.print(getTempC(), 1); Serial.println("C");
  }

  if (odomStream && now - lastOdom >= ODOM_MS) {   // O,<мс>,<режим>,<путь мм>,<скорость мм/с>
    lastOdom = now;
    Serial.print("O,"); Serial.print(now); Serial.print(','); Serial.print(modeName(mode));
    Serial.print(','); Serial.print(accumMm, 0); Serial.print(','); Serial.println(rateMmS, 0);
  }
}

void motionUpdate() {
  hoverboard.housekeeping();
  unsigned long now = millis();
  if (now - lastTick < TICK_MS) return;
  lastTick = now;
  if (!hoverboard.isConnected()) { lastLoopMs = now; return; }   // нет связи: датчики работают дальше
  motionTick(now);
}

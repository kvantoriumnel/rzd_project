// imu.cpp — перенесено из sensor_node v3. Калибровка хранится в Preferences (NVS, раздел "imu")
// вместо EEPROM: на объединённой плате её нужно один раз выполнить заново (команда N C).
#include "imu.h"
#include "config.h"
#include <Wire.h>
#include <Preferences.h>

static const uint8_t MPU_ADDR[2] = {0x68, 0x69};
static const float ACC_LSB = 8192.0;   // ±4 g
static const float GYR_LSB = 65.5;     // ±500 °/с

struct Cal { uint16_t magic; int16_t ax, ay, az, gx, gy, gz; };
static Cal cal[2];
static const uint16_t CAL_MAGIC = 0xCA1B;
static const char *CAL_KEY[2] = {"calA", "calB"};
static Preferences imuPrefs;

static bool imuOk[2] = {false, false}, imuStream = true;
static unsigned long lastSample = 0;
static long sum[2][6]; static int nSum[2] = {0, 0}; static long pk[2] = {0, 0};
static uint8_t phase[2] = {0, 1};

// ================= I2C =================
static void mpuWrite(uint8_t a, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(a); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}

static uint8_t mpuRead8(uint8_t a, uint8_t reg) {
  Wire.beginTransmission(a); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  Wire.requestFrom((int)a, 1);
  return Wire.available() ? Wire.read() : 0xFF;
}

static bool mpuReadRaw(uint8_t a, int16_t v[7]) {
  Wire.beginTransmission(a); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)a, 14) != 14) return false;
  for (int i = 0; i < 7; i++) { int16_t h = Wire.read(); v[i] = (h << 8) | Wire.read(); }
  return true;
}

static bool mpuInit(uint8_t s) {
  uint8_t a = MPU_ADDR[s];
  uint8_t who = mpuRead8(a, 0x75);
  Serial.print(F("# MPU ")); Serial.print(SIDE[s]); Serial.print(F(" 0x")); Serial.print(a, HEX);
  Serial.print(F(" WHO_AM_I=0x")); Serial.println(who, HEX);
  if (who == 0xFF || who == 0x00) return false;
  if (who != 0x68) Serial.println(F("# внимание: WHO_AM_I не 0x68"));
  mpuWrite(a, 0x6B, 0x01); delay(50);   // тактирование от гироскопа X
  mpuWrite(a, 0x1A, 0x03);              // DLPF 44 Гц
  mpuWrite(a, 0x19, 0x09);              // 100 Гц
  mpuWrite(a, 0x1B, 0x08);              // ±500 °/с
  mpuWrite(a, 0x1C, 0x08);              // ±4 g
  return true;
}

// ================= калибровка =================
static void loadCal() {
  imuPrefs.begin("imu", true);
  for (uint8_t s = 0; s < 2; s++) {
    size_t n = imuPrefs.getBytes(CAL_KEY[s], &cal[s], sizeof(Cal));
    Serial.print(F("# калибровка IMU ")); Serial.print(SIDE[s]);
    if (n != sizeof(Cal) || cal[s].magic != CAL_MAGIC) { cal[s] = {0, 0, 0, 0, 0, 0, 0}; Serial.println(F(": нет — выполните N C")); }
    else Serial.println(F(": загружена"));
  }
  imuPrefs.end();
}

static void saveCal() {
  imuPrefs.begin("imu", false);
  for (uint8_t s = 0; s < 2; s++) imuPrefs.putBytes(CAL_KEY[s], &cal[s], sizeof(Cal));
  imuPrefs.end();
}

static void calibrateImu() {   // блокирует на 2 с; перед вызовом моторы останавливаются (commands.cpp)
  Serial.println(F("# калибровка IMU: не трогайте 2 с"));
  long sm[2][6]; int n[2] = {0, 0};
  for (uint8_t s = 0; s < 2; s++) for (uint8_t i = 0; i < 6; i++) sm[s][i] = 0;
  int16_t v[7];
  unsigned long t0 = millis();
  while (millis() - t0 < 2000) {
    for (uint8_t s = 0; s < 2; s++)
      if (imuOk[s] && mpuReadRaw(MPU_ADDR[s], v)) {
        sm[s][0] += v[0]; sm[s][1] += v[1]; sm[s][2] += v[2];
        sm[s][3] += v[4]; sm[s][4] += v[5]; sm[s][5] += v[6]; n[s]++;
      }
    delay(4);
  }
  for (uint8_t s = 0; s < 2; s++) {
    Serial.print(F("# IMU ")); Serial.print(SIDE[s]);
    if (n[s] < 100) { Serial.println(F(": ошибка, мало отсчётов")); continue; }
    Cal &c = cal[s];
    c.magic = CAL_MAGIC;
    c.ax = sm[s][0] / n[s]; c.ay = sm[s][1] / n[s]; c.az = sm[s][2] / n[s] - (int16_t)ACC_LSB;
    c.gx = sm[s][3] / n[s]; c.gy = sm[s][4] / n[s]; c.gz = sm[s][5] / n[s];
    Serial.print(F(": готово, N=")); Serial.print(n[s]);
    Serial.print(F("; a=")); Serial.print(c.ax); Serial.print(' '); Serial.print(c.ay); Serial.print(' '); Serial.print(c.az);
    Serial.print(F(" g=")); Serial.print(c.gx); Serial.print(' '); Serial.print(c.gy); Serial.print(' '); Serial.println(c.gz);
    if (labs(sm[s][2] / n[s] - (long)ACC_LSB) > 1200) {
      Serial.print(F("# внимание: ось Z IMU ")); Serial.print(SIDE[s]); Serial.println(F(" не вертикальна"));
    }
  }
  saveCal();
}

// ================= поток =================
static void outImu(uint8_t s) {
  if (imuStream && nSum[s] > 0) {
    Serial.print(F("I,")); Serial.print(millis()); Serial.print(','); Serial.print(SIDE[s]);
    for (int i = 0; i < 3; i++) { Serial.print(','); Serial.print((long)(sum[s][i] / nSum[s] * 1000.0 / ACC_LSB)); }
    for (int i = 3; i < 6; i++) { Serial.print(','); Serial.print((long)(sum[s][i] / nSum[s] * 100.0 / GYR_LSB)); }
    Serial.print(','); Serial.println((long)(pk[s] * 1000.0 / ACC_LSB));
  }
  for (int i = 0; i < 6; i++) sum[s][i] = 0;
  nSum[s] = 0; pk[s] = 0;
}

void imuUpdate(unsigned long nowUs) {
  if (nowUs - lastSample < 10000) return;   // 100 Гц
  lastSample = nowUs;
  for (uint8_t s = 0; s < 2; s++) {
    if (!imuOk[s]) continue;
    int16_t v[7];
    if (!mpuReadRaw(MPU_ADDR[s], v)) continue;
    const Cal &c = cal[s];
    long a[6] = {v[0] - c.ax, v[1] - c.ay, v[2] - c.az, v[4] - c.gx, v[5] - c.gy, v[6] - c.gz};
    for (int i = 0; i < 6; i++) sum[s][i] += a[i];
    nSum[s]++;
    long d = labs(a[2] - (long)ACC_LSB);
    if (d > pk[s]) pk[s] = d;
    if (++phase[s] >= 2) { phase[s] = 0; outImu(s); }
  }
}

void imuSetup() {
  for (uint8_t s = 0; s < 2; s++) {
    imuOk[s] = mpuInit(s);
    if (!imuOk[s]) {
      Serial.print(F("# ошибка: MPU-6050 ")); Serial.print(SIDE[s]);
      Serial.println(F(" не отвечает (SDA/SCL, питание 3V3, AD0)"));
    }
  }
  loadCal();
}

void imuStatus() {
  for (uint8_t s = 0; s < 2; s++) {
    Serial.print(F("# IMU ")); Serial.print(SIDE[s]); Serial.print(imuOk[s] ? F(": OK") : F(": НЕТ"));
    Serial.print(F(", калибровка ")); Serial.println(cal[s].magic == CAL_MAGIC ? F("есть") : F("нет"));
  }
}

bool imuCommand(char c, long v) {
  switch (c) {
    case 'C': calibrateImu(); return true;
    case 'Z':
      for (uint8_t i = 0; i < 2; i++) cal[i] = {0, 0, 0, 0, 0, 0, 0};
      saveCal();
      Serial.println(F("# калибровка IMU сброшена")); return true;
    case 'Q': imuStream = (v == 1); return true;
  }
  return false;
}

/*
  sensor_node.ino v4 (колея 750 мм) — узел датчиков двух тележек: 2 x HC-SR04 + 2 x MPU-6050 (GY-521) + 4 x CJMCU-531 (VL53L1X)
  Тележка A — левая (с направляющими роликами), тележка B — правая. На каждой тележке:
    HC-SR04 — препятствие на своём рельсе; GY-521 — удары/крен; дальномер H («высота») — вниз на головку рельса
    (ступенька в стыке, просадка); дальномер G («колея») — горизонтально на рабочую грань головки (ширина колеи).

  КОЛЕЯ. Настроено под конкурсный полигон: номинал 750 мм. Тележки те же, что под 1520, — короче только поперечины,
  поэтому расстояние каждого дальномера G до своей рабочей грани не меняется, меняется поправка:
      колея = поправка + kA * G_A + kB * G_B,   поправка 750 = 1398,2 (модель ред.7 при 1520) - 770 = 628,2 мм.
  Точную поправку задаёт калибровка по путевому шаблону: команда WG750 (раздел «Команды»).
  Узел сам считает колею и выдаёт строку W и событие при выходе из допуска. Допуски по умолчанию перенесены
  с колеи 1520 (№ 2288р: -4/+8 мм, закрытие движения -8/+28 мм от номинала) — СВЕРИТЬ с регламентом полигона
  и при необходимости поменять командами WL/WH/WX/WY. Для сети РЖД: WN1520, WO1398.2 (или WG1520 на шаблоне), WS.
  Плата: Arduino Nano / Uno (ATmega328P, 5 В). Связь с ноутбуком: USB, 115200 бод.
  Библиотека: «VL53L1X by Pololu» (Arduino IDE: Инструменты -> Управление библиотеками -> VL53L1X, Pololu), версия 1.3 и новее.

  Подключение (подробно — инструкция, раздел 3):
    HC-SR04 A  VCC->5V  GND->GND  TRIG->D9   ECHO->D2 (INT0)
    HC-SR04 B  VCC->5V  GND->GND  TRIG->D10  ECHO->D3 (INT1)
    GY-521  A  VCC->5V  GND->GND  SCL->A5  SDA->A4  AD0->GND  (адрес 0x68)
    GY-521  B  VCC->5V  GND->GND  SCL->A5  SDA->A4  AD0->3V3  (адрес 0x69; не на 5V — вход MPU-6050 рассчитан на 3,3 В)
    CJMCU-531 (все 4) VIN->5V*  GND->GND  SCL->A5  SDA->A4  (общая шина I2C с GY-521)
       XSHUT: H A -> D4, H B -> D5, G A -> D6, G B -> D7; адреса задаются при старте: 0x30, 0x31, 0x32, 0x33
       * если на плате НЕТ стабилизатора 3,3 В (см. инструкцию, шаг проверки) — VIN только к 3V3 Arduino!
       XSHUT узел никогда не подаёт HIGH: «держать в сбросе» = LOW, «отпустить» = вход (вывод поднимает подтяжка платы)
    СТОП       D8 -> вход «стоп» платы управления приводом (HIGH = препятствие на любом рельсе), общий GND

  Строки узла (поля через запятую; S — сторона A или B):
    I,t_ms,S,ax_mg,ay_mg,az_mg,gx_cdps,gy_cdps,gz_cdps,pk_mg — IMU, 50 Гц на каждую; pk — максимум |az-1g| за 20 мс (выборка 100 Гц)
    U,t_ms,S,raw_mm,med_mm        — ультразвук, ~12 Гц на каждый; -1 = нет эха в диапазоне
    E,t_ms,S,STOP|CLEAR|LOST,mm   — смена состояния; LOST — эхо близкого предмета пропало: стоп держится до команды R
    T,t_ms,S,K,mm,st              — дальномер; K = H (высота, ~30 Гц) или G (колея, 10 Гц); mm — СЫРОЕ расстояние
                                    (калибровка — в sensors.py); st — статус VL53L1X: 0 = норма, иначе отсчёт не использовать
    W,t_ms,мм,откл_мм,OK|TOL|CLOSE — колея (~10 Гц, только когда оба G дали годный отсчёт): ширина, отклонение от
                                    номинала, оценка: OK — в допуске, TOL — вне допуска, CLOSE — порог закрытия движения
    #...                          — текст: ответы на команды, ошибки; «# колея: ...» — смена оценки колеи
    Строки T, I, U, E — как в v3: sensors.py работает без изменений (строки W он пропускает).
  Команды (строка + Enter):
    ?            состояние и настройки
    C            калибровка обоих IMU в покое (2 с, тележки неподвижны и горизонтальны), запись в EEPROM
    Z            сброс калибровки IMU
    S<мм>        порог «стоп», S500          H<мм>  гистерезис отпускания, H200
    M<мм>        дальность игнорирования, M1300 (дальше — «нет препятствия»)
    K<мм>        порог LOST, K1000 (0 — выключить)       R  снять стоп LOST после осмотра
    T<°C>        температура воздуха, T5     Q0 / Q1 выключить / включить поток IMU
    D0 / D1      выключить / включить поток дальномеров    X  перезапуск дальномеров (после замены, обрыва провода)
    Колея (настройки хранятся в EEPROM после WS):
    W?           настройки колеи          W0 / W1  выключить / включить строки W
    WG<мм>       калибровка поправки на путевом шаблоне известной ширины, WG750 (2 с, тележки неподвижны) -> затем WS
    WN<мм>       номинал колеи, WN750      WO<мм>  поправка вручную, WO628.2
    WA<k> WB<k>  коэффициенты дальномеров G A / G B, WA1.000 (0,90..1,10)
    WL<мм> WH<мм> допуск сужения / уширения от номинала, WL4 WH8
    WX<мм> WY<мм> порог закрытия: сужение / уширение от номинала, WX8 WY28
    WS           сохранить в EEPROM        WR  вернуть значения по умолчанию (750 мм)
*/
#include <Wire.h>
#include <EEPROM.h>
#include <VL53L1X.h>

// ---------------- выводы
const uint8_t PIN_TRIG[2] = {9, 10};
const uint8_t PIN_ECHO[2] = {2, 3};     // INT0, INT1
const uint8_t PIN_STOP = 8;
const uint8_t MPU_ADDR[2] = {0x68, 0x69};
const char SIDE[2] = {'A', 'B'};

// ---------------- дальномеры CJMCU-531 (VL53L1X): 0 = H A, 1 = H B, 2 = G A, 3 = G B
const uint8_t TOF_N = 4;
const uint8_t PIN_XSHUT[TOF_N] = {4, 5, 6, 7};
const uint8_t TOF_ADDR[TOF_N] = {0x30, 0x31, 0x32, 0x33};
const char TOF_KIND[TOF_N] = {'H', 'H', 'G', 'G'};
const uint8_t TOF_SIDE[TOF_N] = {0, 1, 0, 1};
const uint16_t TOF_PERIOD[TOF_N] = {33, 33, 100, 100};          // мс между измерениями
const uint32_t TOF_BUDGET[TOF_N] = {30000, 30000, 50000, 50000}; // время накопления, мкс (Short: 20..1000 мс)
VL53L1X tof[TOF_N];
bool tofOk[TOF_N] = {false, false, false, false}, tofStream = true;
unsigned long tofPoll[TOF_N] = {0, 0, 0, 0};

// ---------------- настройки (меняются командами, после сброса — эти значения)
long stopMM = 500;       // ближе — стоп
long hystMM = 200;       // отпустить, когда дальше stopMM + hystMM не менее 1 с
long maxMM = 1300;       // дальше — не учитывать
long lostMM = 1000;      // эхо было ближе lostMM и пропало -> LOST
float airC = 20.0;       // температура воздуха, °C
const unsigned long US_SLOT = 40;      // мс между импульсами; датчики по очереди -> каждый раз в 80 мс

// ---------------- калибровка IMU (EEPROM: A по адресу 0, B — следом)
struct Cal { uint16_t magic; int16_t ax, ay, az, gx, gy, gz; };
Cal cal[2];
const uint16_t CAL_MAGIC = 0xCA1B;

// ---------------- колея (EEPROM: после двух Cal IMU, адрес 32)
struct GaugeCfg { uint16_t magic; float off, kA, kB; int16_t nom, tolLo, tolHi, clsLo, clsHi; };
const uint16_t GAUGE_MAGIC = 0x0750;
const int GAUGE_EE = 32;
const GaugeCfg GAUGE_DEF = {GAUGE_MAGIC, 628.2, 1.0, 1.0, 750, 4, 8, 8, 28};
GaugeCfg gc = GAUGE_DEF;
uint16_t gRaw[2] = {0, 0};
unsigned long gT[2] = {0, 0};
bool gGood[2] = {false, false}, gNew[2] = {false, false};
bool gaugeStream = true;
uint8_t gaugeState = 255;              // 0 OK, 1 TOL, 2 CLOSE, 255 — ещё не было отсчёта
long gCalTarget = 0;                   // калибровка по шаблону: ширина шаблона, мм
unsigned long gCalUntil = 0;
float gCalSum = 0; int gCalN = 0;
const unsigned long G_PAIR_MS = 150;   // отсчёты A и B не дальше друг от друга во времени

// ---------------- ультразвук: измерение по прерыванию
volatile unsigned long echoStart[2] = {0, 0}, echoDur[2] = {0, 0};
volatile bool echoDone[2] = {false, false};
void echoISR0() {
  if (digitalRead(PIN_ECHO[0])) echoStart[0] = micros();
  else { echoDur[0] = micros() - echoStart[0]; echoDone[0] = true; }
}
void echoISR1() {
  if (digitalRead(PIN_ECHO[1])) echoStart[1] = micros();
  else { echoDur[1] = micros() - echoStart[1]; echoDone[1] = true; }
}
long usHist[2][3] = {{-1, -1, -1}, {-1, -1, -1}};
uint8_t usIdx[2] = {0, 0};
bool pending[2] = {false, false};
bool stopState[2] = {false, false}, lostState[2] = {false, false};
unsigned long clearSince[2] = {0, 0};
long lastValid[2] = {0, 0};
unsigned long lastValidT[2] = {0, 0};
unsigned long lastPing = 0;
uint8_t nextSide = 0;

// ---------------- IMU
bool imuOk[2] = {false, false}, imuStream = true;
unsigned long lastSample = 0;
long sum[2][6]; int nSum[2] = {0, 0}; long pk[2] = {0, 0};
uint8_t phase[2] = {0, 1};      // B сдвинута на 1 отсчёт: строки A и B не идут подряд
const float ACC_LSB = 8192.0;   // ±4 g
const float GYR_LSB = 65.5;     // ±500 °/с

String line;

void mpuWrite(uint8_t a, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(a); Wire.write(reg); Wire.write(val); Wire.endTransmission();
}

uint8_t mpuRead8(uint8_t a, uint8_t reg) {
  Wire.beginTransmission(a); Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFF;
  Wire.requestFrom((int)a, 1);
  return Wire.available() ? Wire.read() : 0xFF;
}

bool mpuReadRaw(uint8_t a, int16_t v[7]) {   // ax ay az temp gx gy gz
  Wire.beginTransmission(a); Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)a, 14) != 14) return false;
  for (int i = 0; i < 7; i++) { int16_t h = Wire.read(); v[i] = (h << 8) | Wire.read(); }
  return true;
}

bool mpuInit(uint8_t s) {
  uint8_t a = MPU_ADDR[s];
  uint8_t who = mpuRead8(a, 0x75);
  Serial.print(F("# MPU ")); Serial.print(SIDE[s]); Serial.print(F(" адрес 0x")); Serial.print(a, HEX);
  Serial.print(F(" WHO_AM_I=0x")); Serial.println(who, HEX);
  if (who == 0xFF || who == 0x00) return false;     // нет ответа по I2C
  if (who != 0x68) Serial.println(F("# внимание: не оригинал MPU-6050 (WHO_AM_I не 0x68), проверьте регистры по паспорту"));
  mpuWrite(a, 0x6B, 0x01);   // проснуться, тактирование от гироскопа X
  delay(50);
  mpuWrite(a, 0x1A, 0x03);   // цифровой ФНЧ 44 Гц
  mpuWrite(a, 0x19, 0x09);   // 1 кГц / (1+9) = 100 Гц (шина I2C общая с 4 дальномерами)
  mpuWrite(a, 0x1B, 0x08);   // гироскоп ±500 °/с
  mpuWrite(a, 0x1C, 0x08);   // акселерометр ±4 g
  return true;
}

void loadCal() {
  for (uint8_t s = 0; s < 2; s++) {
    EEPROM.get(s * sizeof(Cal), cal[s]);
    Serial.print(F("# калибровка IMU ")); Serial.print(SIDE[s]);
    if (cal[s].magic != CAL_MAGIC) { cal[s] = {0, 0, 0, 0, 0, 0, 0}; Serial.println(F(": нет — выполните C")); }
    else Serial.println(F(": загружена"));
  }
}

void calibrateImu() {
  Serial.println(F("# калибровка IMU: не трогайте тележки 2 с"));
  long sm[2][6]; int n[2] = {0, 0};
  for (uint8_t s = 0; s < 2; s++) for (uint8_t i = 0; i < 6; i++) sm[s][i] = 0;
  int16_t v[7];
  unsigned long t0 = millis();
  while (millis() - t0 < 2000) {
    for (uint8_t s = 0; s < 2; s++)
      if (imuOk[s] && mpuReadRaw(MPU_ADDR[s], v)) {
        sm[s][0] += v[0]; sm[s][1] += v[1]; sm[s][2] += v[2]; sm[s][3] += v[4]; sm[s][4] += v[5]; sm[s][5] += v[6]; n[s]++;
      }
    delay(4);
  }
  for (uint8_t s = 0; s < 2; s++) {
    Serial.print(F("# IMU ")); Serial.print(SIDE[s]);
    if (n[s] < 100) { Serial.println(F(": ошибка, мало отсчётов — проверьте провода I2C и AD0")); continue; }
    Cal &c = cal[s];
    c.magic = CAL_MAGIC;
    c.ax = sm[s][0] / n[s]; c.ay = sm[s][1] / n[s]; c.az = sm[s][2] / n[s] - (int16_t)ACC_LSB;   // ось Z вверх: 1 g
    c.gx = sm[s][3] / n[s]; c.gy = sm[s][4] / n[s]; c.gz = sm[s][5] / n[s];
    EEPROM.put(s * sizeof(Cal), c);
    Serial.print(F(": готово, отсчётов ")); Serial.print(n[s]);
    Serial.print(F("; смещения a=")); Serial.print(c.ax); Serial.print(' '); Serial.print(c.ay); Serial.print(' '); Serial.print(c.az);
    Serial.print(F(" g=")); Serial.print(c.gx); Serial.print(' '); Serial.print(c.gy); Serial.print(' '); Serial.println(c.gz);
    if (abs(sm[s][2] / n[s] - (long)ACC_LSB) > 1200) {
      Serial.print(F("# внимание: ось Z IMU ")); Serial.print(SIDE[s]); Serial.println(F(" не вертикальна (>8°) — проверьте установку"));
    }
  }
}

bool tofInitOne(uint8_t i) {
  pinMode(PIN_XSHUT[i], INPUT);                 // отпустить XSHUT: поднимет подтяжка платы
  delay(10);                                    // загрузка датчика ~1,2 мс
  tof[i].setBus(&Wire);
  tof[i].setTimeout(200);
  Serial.print(F("# ToF ")); Serial.print(TOF_KIND[i]); Serial.print(' '); Serial.print(SIDE[TOF_SIDE[i]]);
  Serial.print(F(" (XSHUT D")); Serial.print(PIN_XSHUT[i]); Serial.print(F(")"));
  tof[i].setAddress(0x29);                      // после сброса у всех 0x29
  if (!tof[i].init()) { Serial.println(F(": НЕТ ответа — провода, питание, XSHUT")); return false; }
  tof[i].setAddress(TOF_ADDR[i]);
  tof[i].setDistanceMode(VL53L1X::Short);       // до 1,3 м, лучше всего при солнце
  tof[i].setMeasurementTimingBudget(TOF_BUDGET[i]);
  tof[i].setROISize(4, 4);                      // узкий луч ~15° вместо 27°: пятно ~12 мм на 45 мм
  tof[i].startContinuous(TOF_PERIOD[i]);
  tofPoll[i] = millis();
  Serial.print(F(": OK, адрес 0x")); Serial.println(TOF_ADDR[i], HEX);
  return true;
}

void tofInitAll() {
  for (uint8_t i = 0; i < TOF_N; i++) { pinMode(PIN_XSHUT[i], OUTPUT); digitalWrite(PIN_XSHUT[i], LOW); }  // все в сброс
  delay(10);
  for (uint8_t i = 0; i < TOF_N; i++) tofOk[i] = tofInitOne(i);   // по одному: включили -> сменили адрес
}

// ---------------- колея
void loadGauge() {
  EEPROM.get(GAUGE_EE, gc);
  bool ok = gc.magic == GAUGE_MAGIC && gc.nom >= 500 && gc.nom <= 1700 && gc.kA > 0.5 && gc.kA < 1.5 && gc.kB > 0.5 && gc.kB < 1.5;
  if (!ok) gc = GAUGE_DEF;
  Serial.print(F("# колея: ")); Serial.println(ok ? F("настройки загружены") : F("настройки по умолчанию (750 мм)"));
}

void gaugeStatus() {
  Serial.print(F("# колея: номинал ")); Serial.print(gc.nom); Serial.print(F(" мм, поправка ")); Serial.print(gc.off, 1);
  Serial.print(F(" мм, kA ")); Serial.print(gc.kA, 3); Serial.print(F(", kB ")); Serial.print(gc.kB, 3);
  Serial.print(F("; допуск -")); Serial.print(gc.tolLo); Serial.print(F("/+")); Serial.print(gc.tolHi);
  Serial.print(F(" мм, закрытие <")); Serial.print(gc.nom - gc.clsLo); Serial.print(F(" / >")); Serial.print(gc.nom + gc.clsHi);
  Serial.print(F(" мм; поток W ")); Serial.println(gaugeStream ? 1 : 0);
  if (gc.clsLo < gc.tolLo || gc.clsHi < gc.tolHi) Serial.println(F("# внимание: порог закрытия уже допуска — проверьте WX/WY"));
}

void gaugeUpdate(unsigned long now) {
  if (!gNew[0] || !gNew[1]) return;                       // нужна свежая пара A + B
  gNew[0] = gNew[1] = false;
  unsigned long dt = gT[0] > gT[1] ? gT[0] - gT[1] : gT[1] - gT[0];
  if (dt > G_PAIR_MS || !gGood[0] || !gGood[1]) return;   // разнесены во времени или статус не 0
  float sumG = gc.kA * gRaw[0] + gc.kB * gRaw[1];
  if (gCalUntil) { gCalSum += sumG; gCalN++; return; }    // идёт калибровка по шаблону
  float g = gc.off + sumG, d = g - gc.nom;
  uint8_t st = (d < -gc.clsLo || d > gc.clsHi) ? 2 : ((d < -gc.tolLo || d > gc.tolHi) ? 1 : 0);
  const __FlashStringHelper *name = st == 2 ? F("CLOSE") : (st == 1 ? F("TOL") : F("OK"));
  if (gaugeStream) {
    Serial.print(F("W,")); Serial.print(now); Serial.print(','); Serial.print(g, 1); Serial.print(',');
    Serial.print(d, 1); Serial.print(','); Serial.println(name);
  }
  if (st != gaugeState) {
    gaugeState = st;
    Serial.print(F("# колея: ")); Serial.print(name); Serial.print(' '); Serial.print(g, 1); Serial.println(F(" мм"));
  }
}

void gaugeCalStart(long target) {
  if (target < 500 || target > 1700) { Serial.println(F("# WG: ширина шаблона 500..1700 мм, например WG750")); return; }
  if (!tofOk[2] || !tofOk[3]) { Serial.println(F("# WG: дальномеры G A и G B должны отвечать (команда ?)")); return; }
  gCalTarget = target; gCalSum = 0; gCalN = 0; gCalUntil = millis() + 2000;
  Serial.print(F("# калибровка колеи по шаблону ")); Serial.print(target); Serial.println(F(" мм: не трогайте тележки 2 с"));
}

void gaugeCalCheck(unsigned long now) {
  if (!gCalUntil || (long)(now - gCalUntil) < 0) return;
  gCalUntil = 0;
  if (gCalN < 10) { Serial.print(F("# WG: ошибка, годных пар ")); Serial.print(gCalN); Serial.println(F(" (нужно >= 10) — проверьте G A, G B")); return; }
  float off = gCalTarget - gCalSum / gCalN;
  float expd = GAUGE_DEF.off + (gCalTarget - GAUGE_DEF.nom);   // ожидаемая поправка по модели
  gc.off = off;
  if (labs(gCalTarget - gc.nom) > 100) {
    Serial.print(F("# внимание: шаблон ")); Serial.print(gCalTarget); Serial.print(F(" мм, а номинал ")); Serial.print(gc.nom);
    Serial.println(F(" мм — смените номинал командой WN"));
  }
  Serial.print(F("# колея: пар ")); Serial.print(gCalN); Serial.print(F(", поправка ")); Serial.print(off, 1);
  Serial.print(F(" мм (по модели около ")); Serial.print(expd, 1); Serial.println(F("). Сохранить: WS"));
  if (fabs(off - expd) > 25) Serial.println(F("# внимание: поправка отличается от модели > 25 мм — проверьте шаблон и крепление G"));
}

void gaugeCommand(const String &s) {   // s начинается с 'W'
  char c = s.length() > 1 ? s.charAt(1) : '?';
  float v = s.substring(2).toFloat();
  switch (c) {
    case '?': break;
    case '0': gaugeStream = false; break;
    case '1': gaugeStream = true; break;
    case 'G': gaugeCalStart((long)v); return;
    case 'N': if (v >= 500 && v <= 1700) gc.nom = (int16_t)v; break;
    case 'O': if (v >= 0 && v <= 1700) gc.off = v; break;
    case 'A': if (v >= 0.9 && v <= 1.1) gc.kA = v; break;
    case 'B': if (v >= 0.9 && v <= 1.1) gc.kB = v; break;
    case 'L': if (v >= 0 && v <= 50) gc.tolLo = (int16_t)v; break;
    case 'H': if (v >= 0 && v <= 50) gc.tolHi = (int16_t)v; break;
    case 'X': if (v >= 0 && v <= 80) gc.clsLo = (int16_t)v; break;
    case 'Y': if (v >= 0 && v <= 80) gc.clsHi = (int16_t)v; break;
    case 'S': gc.magic = GAUGE_MAGIC; EEPROM.put(GAUGE_EE, gc); Serial.println(F("# колея: сохранено в EEPROM")); break;
    case 'R': gc = GAUGE_DEF; Serial.println(F("# колея: значения по умолчанию (750 мм); сохранить: WS")); break;
    default: Serial.print(F("# неизвестная команда: ")); Serial.println(s); return;
  }
  gaugeState = 255;
  gaugeStatus();
}

void handleTof(unsigned long now) {
  for (uint8_t i = 0; i < TOF_N; i++) {
    if (!tofOk[i]) continue;
    // не опрашивать раньше времени: шина общая
    if (now - tofPoll[i] < (unsigned long)TOF_PERIOD[i] - 3) continue;
    if (!tof[i].dataReady()) {
      if (now - tofPoll[i] > 4UL * TOF_PERIOD[i] + 200) {
        tofOk[i] = false;
        Serial.print(F("# ошибка: ToF ")); Serial.print(TOF_KIND[i]); Serial.print(' '); Serial.print(SIDE[TOF_SIDE[i]]);
        Serial.println(F(" перестал отвечать — команда X после проверки провода"));
      }
      continue;
    }
    uint16_t mm = tof[i].read(false);
    tofPoll[i] = now;
    if (TOF_KIND[i] == 'G') {
      uint8_t s = TOF_SIDE[i];
      gRaw[s] = mm; gT[s] = now; gNew[s] = true;
      gGood[s] = (tof[i].ranging_data.range_status == VL53L1X::RangeValid);
      gaugeUpdate(now);
    }
    if (!tofStream) continue;
    Serial.print(F("T,")); Serial.print(now); Serial.print(','); Serial.print(SIDE[TOF_SIDE[i]]); Serial.print(',');
    Serial.print(TOF_KIND[i]); Serial.print(','); Serial.print(mm); Serial.print(',');
    Serial.println((int)tof[i].ranging_data.range_status);
  }
}

long median3(long a, long b, long c) {
  if (a > b) { long t = a; a = b; b = t; }
  if (b > c) { long t = b; b = c; c = t; }
  if (a > b) { long t = a; a = b; b = t; }
  return b;
}

void updateStopPin() {
  bool any = stopState[0] || stopState[1] || lostState[0] || lostState[1];
  digitalWrite(PIN_STOP, any ? HIGH : LOW);
}

void event(unsigned long now, uint8_t s, const __FlashStringHelper *ev, long mm) {
  Serial.print(F("E,")); Serial.print(now); Serial.print(','); Serial.print(SIDE[s]); Serial.print(',');
  Serial.print(ev); Serial.print(','); Serial.println(mm);
}

void usResult(unsigned long now, uint8_t s, long mm) {
  usHist[s][usIdx[s]] = mm; usIdx[s] = (usIdx[s] + 1) % 3;
  long med = median3(usHist[s][0], usHist[s][1], usHist[s][2]);
  Serial.print(F("U,")); Serial.print(now); Serial.print(','); Serial.print(SIDE[s]); Serial.print(',');
  Serial.print(mm); Serial.print(','); Serial.println(med);
  // LOST: близкий предмет был и пропал — низкий предмет ушёл под нижний край конуса
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

void handleUltrasound(unsigned long now) {
  for (uint8_t s = 0; s < 2; s++) {
    if (echoDone[s]) {
      noInterrupts(); unsigned long d = echoDur[s]; echoDone[s] = false; interrupts();
      pending[s] = false;
      float c = 331.3 + 0.606 * airC;                  // м/с
      long mm = (long)(d * c / 2000.0);                // мкс * м/с / 2 / 1000 = мм
      if (mm < 20 || mm > maxMM) mm = -1;
      usResult(now, s, mm);
    }
  }
  if (now - lastPing >= US_SLOT) {
    lastPing = now;
    uint8_t s = nextSide; nextSide ^= 1;
    if (pending[s]) usResult(now, s, -1);              // датчик не ответил на прошлый импульс
    pending[s] = true;
    digitalWrite(PIN_TRIG[s], LOW); delayMicroseconds(3);
    digitalWrite(PIN_TRIG[s], HIGH); delayMicroseconds(10);
    digitalWrite(PIN_TRIG[s], LOW);
  }
}

void outImu(uint8_t s) {
  if (imuStream && nSum[s] > 0) {
    Serial.print(F("I,")); Serial.print(millis()); Serial.print(','); Serial.print(SIDE[s]);
    for (int i = 0; i < 3; i++) { Serial.print(','); Serial.print((long)(sum[s][i] / nSum[s] * 1000.0 / ACC_LSB)); }
    for (int i = 3; i < 6; i++) { Serial.print(','); Serial.print((long)(sum[s][i] / nSum[s] * 100.0 / GYR_LSB)); }
    Serial.print(','); Serial.println((long)(pk[s] * 1000.0 / ACC_LSB));
  }
  for (int i = 0; i < 6; i++) sum[s][i] = 0;
  nSum[s] = 0; pk[s] = 0;
}

void handleImu(unsigned long nowUs) {
  if (nowUs - lastSample < 10000) return;             // 100 Гц
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
    if (++phase[s] >= 2) { phase[s] = 0; outImu(s); }   // каждые 2 отсчёта = 20 мс = 50 Гц
  }
}

void status() {
  for (uint8_t s = 0; s < 2; s++) {
    Serial.print(F("# ")); Serial.print(SIDE[s]); Serial.print(F(": IMU ")); Serial.print(imuOk[s] ? F("OK") : F("НЕТ"));
    Serial.print(F(", калибровка ")); Serial.print(cal[s].magic == CAL_MAGIC ? F("есть") : F("нет"));
    Serial.print(F("; стоп ")); Serial.print(stopState[s] ? 1 : 0); Serial.print(F(", LOST ")); Serial.println(lostState[s] ? 1 : 0);
  }
  Serial.print(F("# стоп ")); Serial.print(stopMM); Serial.print(F(" мм, гистерезис ")); Serial.print(hystMM);
  Serial.print(F(", макс ")); Serial.print(maxMM); Serial.print(F(", LOST ")); Serial.print(lostMM);
  Serial.print(F(", T=")); Serial.print(airC); Serial.println(F(" C"));
  for (uint8_t i = 0; i < TOF_N; i++) {
    Serial.print(F("# ToF ")); Serial.print(TOF_KIND[i]); Serial.print(' '); Serial.print(SIDE[TOF_SIDE[i]]);
    Serial.print(F(" 0x")); Serial.print(TOF_ADDR[i], HEX); Serial.print(' '); Serial.println(tofOk[i] ? F("OK") : F("НЕТ"));
  }
  gaugeStatus();
}

void command(String s) {
  s.trim();
  if (s.length() == 0) return;
  char c = s.charAt(0);
  long v = s.substring(1).toInt();
  if (c == 'W') { gaugeCommand(s); return; }
  switch (c) {
    case '?': status(); break;
    case 'C': calibrateImu(); break;
    case 'Z':
      for (uint8_t i = 0; i < 2; i++) { cal[i] = {0, 0, 0, 0, 0, 0, 0}; EEPROM.put(i * sizeof(Cal), cal[i]); }
      Serial.println(F("# калибровка IMU сброшена")); break;
    case 'S': if (v >= 100 && v <= 3000) stopMM = v; status(); break;
    case 'H': if (v >= 50 && v <= 1000) hystMM = v; status(); break;
    case 'M': if (v >= 500 && v <= 4000) maxMM = v; status(); break;
    case 'K': if (v == 0 || (v >= 300 && v <= 4000)) lostMM = v; status(); break;
    case 'R':
      for (uint8_t i = 0; i < 2; i++) lostState[i] = false;
      updateStopPin(); Serial.println(F("# LOST снят")); break;
    case 'T': airC = s.substring(1).toFloat(); status(); break;
    case 'Q': imuStream = (v == 1); break;
    case 'D': tofStream = (v == 1); break;
    case 'X': tofInitAll(); status(); break;
    default: Serial.print(F("# неизвестная команда: ")); Serial.println(s);
  }
}

void setup() {
  Serial.begin(115200);
  for (uint8_t s = 0; s < 2; s++) {
    pinMode(PIN_TRIG[s], OUTPUT); digitalWrite(PIN_TRIG[s], LOW);
    pinMode(PIN_ECHO[s], INPUT);
  }
  pinMode(PIN_STOP, OUTPUT); digitalWrite(PIN_STOP, LOW);
  attachInterrupt(digitalPinToInterrupt(PIN_ECHO[0]), echoISR0, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_ECHO[1]), echoISR1, CHANGE);
  Wire.begin();
  Wire.setClock(100000);        // 100 кГц — надёжнее на проводах до 1,5 м к каждой тележке
  digitalWrite(SDA, LOW); digitalWrite(SCL, LOW);   // выключить внутренние подтяжки к 5 В: шину держат подтяжки 3,3 В модулей
  Serial.println(F("# sensor_node v4 (две тележки, дальномеры VL53L1X, колея 750 мм)"));
  tofInitAll();                 // до MPU: пока дальномеры в сбросе, на шине только GY-521
  for (uint8_t s = 0; s < 2; s++) {
    imuOk[s] = mpuInit(s);
    if (!imuOk[s]) {
      Serial.print(F("# ошибка: MPU-6050 ")); Serial.print(SIDE[s]);
      Serial.println(F(" не отвечает (SDA/SCL, питание, AD0: A — на GND, B — на 3V3)"));
    }
  }
  loadCal();
  loadGauge();
  status();
}

void loop() {
  unsigned long nowUs = micros();
  unsigned long now = millis();
  handleImu(nowUs);
  handleUltrasound(now);
  handleTof(now);
  gaugeCalCheck(now);
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') { command(line); line = ""; }
    else if (line.length() < 32) line += ch;
  }
}

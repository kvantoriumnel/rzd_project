/*
  Telega5 — общая прошивка ESP32 для «Аргус-01»: управление тележками (бывший Telega4 v4)
  + узел датчиков (бывший sensor_node_esp32 v3) на одной плате.

  Модули (вкладки скетча):
    config.h            — выводы и общие константы
    motion.h/.cpp       — плата гироскутера, режимы, одометрия, калибровки, настройки (NVS "hb")
    rc.h/.cpp           — приёмник RC: стик и два тумблера
    ultrasonic.h/.cpp   — 2 x HC-SR04: препятствие, события STOP / CLEAR / LOST
    imu.h/.cpp          — 2 x MPU-6050: поток I, калибровка (NVS "imu")
    tof.h/.cpp          — 4 x VL53L1X: поток T
    commands.h/.cpp     — команды из USB Serial (115200)

  Библиотеки: hoverserial (как в Telega4), VL53L1X (Pololu), Wire и Preferences (ядро ESP32).
  Плата: ESP32 Dev Module.
*/
#include <Wire.h>
#include "config.h"
#include "motion.h"
#include "ultrasonic.h"
#include "imu.h"
#include "tof.h"
#include "commands.h"

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("# Telega5: движение + датчики, ESP32"));

  usSetup();                       // HC-SR04 и индикатор «стоп»

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(I2C_HZ);
  Serial.print(F("# I2C SDA=")); Serial.print(PIN_SDA); Serial.print(F(" SCL=")); Serial.print(PIN_SCL);
  Serial.print(F(" @ ")); Serial.print(I2C_HZ / 1000); Serial.println(F(" кГц"));
  tofSetup();                      // VL53L1X: адреса 0x30…0x33 по очереди через XSHUT
  imuSetup();                      // MPU-6050 A 0x68, B 0x69

  motionSetup();                   // плата гироскутера (UART2), RC, настройки
  printStatus();
}

void loop() {
  unsigned long nowUs = micros();
  unsigned long now   = millis();
  motionUpdate();                  // такт 10 мс, без delay()
  imuUpdate(nowUs);
  usUpdate(now);
  tofUpdate(now);
  commandsUpdate();
}

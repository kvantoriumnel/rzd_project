// config.h — выводы ESP32 и общие константы (Telega5 = Telega4 + sensor_node v3 на одной плате)
#pragma once
#include <Arduino.h>

// ---------- Плата управления гироскутером (UART2) ----------
constexpr int HOVER_RX_PIN = 16;
constexpr int HOVER_TX_PIN = 17;

// ---------- Приёмник RC (ШИМ 1000…2000 мкс) ----------
constexpr int RC_STICK_PIN = 23;
constexpr int RC_TUMB1_PIN = 15;   // тумблер 1: ручное управление
constexpr int RC_TUMB2_PIN = 2;    // тумблер 2: автоматический режим

// ---------- I2C: 2 x MPU-6050 + 4 x VL53L1X (как в коде sensor_node v3) ----------
constexpr int PIN_SDA = 22;
constexpr int PIN_SCL = 21;
constexpr uint32_t I2C_HZ = 100000;

// ---------- HC-SR04 (ECHO выдаёт 5 В — только через делитель 1к/2к!) ----------
// TRIG A перенесён с GPIO 2 на GPIO 18: GPIO 2 занят тумблером 2 приёмника RC.
constexpr uint8_t PIN_TRIG[2] = {18, 25};
constexpr uint8_t PIN_ECHO[2] = {26, 34};

// Индикатор «препятствие» (HIGH = препятствие или потеря цели). Моторы останавливаются программно,
// вывод оставлен для светодиода / внешнего устройства.
constexpr uint8_t PIN_STOP = 27;

// ---------- VL53L1X: XSHUT ----------
constexpr uint8_t TOF_N = 4;
constexpr uint8_t PIN_XSHUT[TOF_N] = {33, 4, 13, 32};

// ---------- Общие ----------
constexpr char SIDE[2] = {'A', 'B'};   // тележка A / тележка B

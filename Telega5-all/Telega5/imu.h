// imu.h — 2 x MPU-6050 (GY-521): поток I, калибровка с сохранением в NVS
#pragma once
#include <Arduino.h>

void imuSetup();                       // после Wire.begin()
void imuUpdate(unsigned long nowUs);   // 100 Гц опрос, вывод 50 Гц
bool imuCommand(char c, long v);       // команды N C/Z/Q; true — обработана
void imuStatus();

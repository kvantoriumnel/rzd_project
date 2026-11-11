// ultrasonic.h — 2 x HC-SR04: дальность до препятствия, события STOP / CLEAR / LOST
#pragma once
#include <Arduino.h>

void usSetup();
void usUpdate(unsigned long nowMs);
bool usObstacle();                     // true — препятствие ближе порога или цель пропала (LOST)
bool usCommand(char c, long v, const char *arg);   // команды N S/H/M/K/R/T; true — обработана
void usStatus();

// motion.h — движение: плата гироскутера, режимы STOP/COM/RC/AUTO/CALIB/CAL_DIST, одометрия, настройки в NVS
#pragma once
#include <Arduino.h>

void motionSetup();
void motionUpdate();             // вызывать каждый проход loop(); такт управления 10 мс внутри
bool motionCommand(char *line);  // команды Telega4 + HOLD, O; true — обработана
void motionHalt();               // немедленная остановка (режим STOP), для блокирующих команд датчиков
void motionStatus();

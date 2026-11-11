// commands.h — приём строк из USB Serial и разбор команд
//   команды движения (Telega4):  STOP, S, R, K, KR, D, A, M, RESET, C …, HOLD, O
//   команды датчиков (sensor_node): с префиксом N — N ?, N C, N S500, N X …
//   ?  — общий статус
#pragma once
#include <Arduino.h>

void commandsUpdate();   // вызывать в loop()
void printStatus();

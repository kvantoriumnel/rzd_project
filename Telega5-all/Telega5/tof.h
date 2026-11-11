// tof.h — 4 x CJMCU-531 (VL53L1X): высотные H и колейные G, поток T
#pragma once
#include <Arduino.h>

void tofSetup();                       // после Wire.begin(): XSHUT и адреса 0x30…0x33
void tofUpdate(unsigned long nowMs);
bool tofCommand(char c, long v);       // команды N D/X; true — обработана
void tofStatus();

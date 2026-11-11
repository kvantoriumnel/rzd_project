// rc.h — приёмник RC: стик скорости и два тумблера (ШИМ, измерение по прерываниям)
#pragma once
#include <Arduino.h>

void rcSetup();
uint16_t rcStick();      // мкс, 1500 — нейтраль
bool rcManual();         // тумблер 1 включён — ручное управление
bool rcAuto();           // тумблер 2 включён — автоматический режим
bool rcTogglesOff();     // оба тумблера выключены

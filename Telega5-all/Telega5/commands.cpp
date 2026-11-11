// commands.cpp — общий разбор команд. В двух исходных скетчах были одинаковые буквы (C, S, M, K, R, D)
// с разным смыслом, поэтому команды датчиков получили префикс N.
#include "commands.h"
#include "motion.h"
#include "ultrasonic.h"
#include "imu.h"
#include "tof.h"

static char lineBuf[64];
static uint8_t lineLen = 0;

void printStatus() {
  motionStatus();
  imuStatus();
  usStatus();
  tofStatus();
}

static void printHelp() {
  Serial.println(F("# движение: STOP | S <скор.> | A <скор.> | M <шаг мм> | D <диам. мм> | R <об.> [скор.] | KR <об.> | K <к>"));
  Serial.println(F("#           RESET | C START [скор.] | C STOP <мм> | C ZERO | C RESET | C INFO | HOLD 0/1 | O 0/1"));
  Serial.println(F("# датчики:  N ? | N C | N Z | N Q0/1 | N D0/1 | N X | N S<мм> | N H<мм> | N M<мм> | N K<мм> | N R | N T<°C>"));
  Serial.println(F("# общий статус: ?"));
}

// Команды датчиков (бывший command() из sensor_node): буква + число
static void sensorCommand(char *s) {
  while (*s == ' ') s++;
  if (*s == '\0') { printHelp(); return; }
  char c = *s;
  if (c >= 'a' && c <= 'z') c -= 32;
  const char *arg = s + 1;
  while (*arg == ' ') arg++;
  long v = atol(arg);

  if (c == '?') { imuStatus(); usStatus(); tofStatus(); return; }
  // блокирующие операции (калибровка IMU 2 с, перезапуск дальномеров): сначала остановить тележку
  if (c == 'C' || c == 'X') { motionHalt(); Serial.println(F("# движение остановлено на время операции")); }
  if (imuCommand(c, v)) return;
  if (tofCommand(c, v)) return;
  if (usCommand(c, v, arg)) return;
  Serial.print(F("# неизвестная команда датчиков: N ")); Serial.println(s);
}

static void dispatch(char *line) {
  while (*line == ' ') line++;
  if (*line == '\0') return;
  if (line[0] == '?' && line[1] == '\0') { printStatus(); return; }
  if (line[0] == 'N' || line[0] == 'n') {           // ни одна команда движения не начинается с N
    sensorCommand(line + 1);
    return;
  }
  char copy[64];
  strncpy(copy, line, sizeof(copy)); copy[sizeof(copy) - 1] = '\0';
  if (motionCommand(line)) return;
  Serial.print(F("# неизвестная команда: ")); Serial.println(copy);
  printHelp();
}

void commandsUpdate() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (lineLen > 0) { lineBuf[lineLen] = '\0'; dispatch(lineBuf); lineLen = 0; }
    } else if (lineLen < sizeof(lineBuf) - 1) lineBuf[lineLen++] = c;
  }
}

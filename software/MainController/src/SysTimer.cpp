#include <Arduino.h>

#include "SysTimer.h"

uint32_t SysGetTimeMs(void) {

   return millis();
}

uint32_t SysGetTimeUs(void) {

   return micros();
}


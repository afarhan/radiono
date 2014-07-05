
#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>
#include "debug.h"

void debug(char const *fmt, ... ) {
  char tmp[64]; // resulting string limited to 64 chars
  va_list args;
  va_start (args, fmt );
  vsnprintf(tmp, sizeof(tmp), fmt, args);
  va_end (args);
  Serial.println(tmp);
}



void debug(const __FlashStringHelper *c) {
    Serial.println(c);
}



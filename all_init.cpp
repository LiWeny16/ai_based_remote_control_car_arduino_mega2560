#include "Arduino.h"
#include "head.h"

void all_init() {
  init_serial(115200);
  init_ultrasonic();
  init_motor();
  init_encoder();
  init_test();
  init_en();
}

#line 1 "C:\\Learning\\sample\\Arduino\\FYP\\Mega 2560\\main\\all_init.cpp"
// @开源协议 GPL3.0
// @作者 Bigonion
// @NameSpace bigonion.cn
// @github https://github.com/LiWeny16/ai_based_remote_control_car_arduino_mega2560

#include "Arduino.h"
#include "head.h"

void all_init() {
  init_serial(115200);
  init_ultrasonic();
  init_motor();
  init_encoder();
  init_test();
  init_en();
  init_pid();
}

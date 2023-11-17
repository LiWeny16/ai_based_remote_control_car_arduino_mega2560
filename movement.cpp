// @开源协议 GPL3.0
// @作者 Bigonion
// @NameSpace bigonion.cn
// @github https://github.com/LiWeny16/ai_based_remote_control_car_arduino_mega2560

#include "Arduino.h"
#include "head.h"

All_Direction_Movement all_direction_movement;
void All_Direction_Movement::stop(){
  en_all_arg.en_motor.en_motor_all = false;
}
void All_Direction_Movement::straight(Speed_Set *speed_set, int speed_rate) {
  (*speed_set).speed_set_1 = speed_rate;
  (*speed_set).speed_set_2 = speed_rate;
  (*speed_set).speed_set_3 = speed_rate;
  (*speed_set).speed_set_4 = speed_rate;
}
void All_Direction_Movement::back(Speed_Set *speed_set, int speed_rate) {
  (*speed_set).speed_set_1 = -speed_rate;
  (*speed_set).speed_set_2 = -speed_rate;
  (*speed_set).speed_set_3 = -speed_rate;
  (*speed_set).speed_set_4 = -speed_rate;
}

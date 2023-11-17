// @开源协议 GPL3.0
// @作者 Bigonion
// @NameSpace bigonion.cn
// @github https://github.com/LiWeny16/ai_based_remote_control_car_arduino_mega2560

#include "Arduino.h"
#include "head.h"

All_Direction_Movement all_direction_movement;
void All_Direction_Movement::stop() {
  en_all_arg.en_motor.en_motor_all = false;
}
void All_Direction_Movement::stop_use_pid(Speed_Set* speed_set) {
  (*speed_set).speed_set_1 = 0;
  (*speed_set).speed_set_2 = 0;
  (*speed_set).speed_set_3 = 0;
  (*speed_set).speed_set_4 = 0;
}

Movement_Conbine_Arg* All_Direction_Movement::straight(Speed_Set* speed_set, int speed_rate, Movement_Conbine_Arg* m_c_a) {
  // (*speed_set).speed_set_1 = speed_rate;
  // (*speed_set).speed_set_2 = speed_rate;
  // (*speed_set).speed_set_3 = speed_rate;
  // (*speed_set).speed_set_4 = speed_rate;
  (*m_c_a).speed_1 = speed_rate;
  (*m_c_a).speed_2 = speed_rate;
  (*m_c_a).speed_3 = speed_rate;
  (*m_c_a).speed_4 = speed_rate;
  return m_c_a;
  // int* temp = new int[4]{ speed_rate, speed_rate, speed_rate, speed_rate };
  // return conbine ? temp : nullptr;
}
void All_Direction_Movement::straight(Speed_Set* speed_set, int speed_rate) {
  (*speed_set).speed_set_1 = speed_rate;
  (*speed_set).speed_set_2 = speed_rate;
  (*speed_set).speed_set_3 = speed_rate;
  (*speed_set).speed_set_4 = speed_rate;
}

Movement_Conbine_Arg* All_Direction_Movement::back(Speed_Set* speed_set, int speed_rate, Movement_Conbine_Arg* m_c_a) {
  (*m_c_a).speed_1 = -speed_rate;
  (*m_c_a).speed_2 = -speed_rate;
  (*m_c_a).speed_3 = -speed_rate;
  (*m_c_a).speed_4 = -speed_rate;
  return m_c_a;
}
void All_Direction_Movement::back(Speed_Set* speed_set, int speed_rate) {
  (*speed_set).speed_set_1 = -speed_rate;
  (*speed_set).speed_set_2 = -speed_rate;
  (*speed_set).speed_set_3 = -speed_rate;
  (*speed_set).speed_set_4 = -speed_rate;
}

Movement_Conbine_Arg* All_Direction_Movement::left(Speed_Set* speed_set, int speed_rate, Movement_Conbine_Arg* m_c_a) {
  (*m_c_a).speed_1 = speed_rate;
  (*m_c_a).speed_2 = -speed_rate;
  (*m_c_a).speed_3 = -speed_rate;
  (*m_c_a).speed_4 = speed_rate;
  return m_c_a;
}
void All_Direction_Movement::left(Speed_Set* speed_set, int speed_rate) {
  (*speed_set).speed_set_1 = speed_rate;
  (*speed_set).speed_set_2 = -speed_rate;
  (*speed_set).speed_set_3 = -speed_rate;
  (*speed_set).speed_set_4 = speed_rate;
}
Movement_Conbine_Arg* All_Direction_Movement::right(Speed_Set* speed_set, int speed_rate, Movement_Conbine_Arg* m_c_a) {
  (*m_c_a).speed_1 = speed_rate;
  (*m_c_a).speed_2 = speed_rate;
  (*m_c_a).speed_3 = speed_rate;
  (*m_c_a).speed_4 = speed_rate;
  return m_c_a;
}
void All_Direction_Movement::right(Speed_Set* speed_set, int speed_rate) {
  (*speed_set).speed_set_1 = speed_rate;
  (*speed_set).speed_set_2 = speed_rate;
  (*speed_set).speed_set_3 = speed_rate;
  (*speed_set).speed_set_4 = speed_rate;
}

void All_Direction_Movement::any(Speed_Set* speed_set, float degree, int speed_rate) {
  // 0-90°
  if (degree > 0 && degree < 90) {
    Movement_Conbine_Arg* m_c_a_1;
    Movement_Conbine_Arg* m_c_a_2;
    float rad = radians(degree);
    m_c_a_1 = all_direction_movement.straight(speed_set, speed_rate * cos(rad), m_c_a_1);
    m_c_a_2 = all_direction_movement.right(speed_set, speed_rate * sin(rad), m_c_a_2);
    // (*m_c_a_1).final_speed->speed_1=
  } else if (degree > 90) {
  }
}

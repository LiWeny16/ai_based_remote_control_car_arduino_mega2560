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

Movement_Combine_Arg* All_Direction_Movement::straight(Speed_Set* speed_set, int speed_rate, Movement_Combine_Arg* m_c_a) {
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
  // return Combine ? temp : nullptr;
}
void All_Direction_Movement::straight(Speed_Set* speed_set, int speed_rate) {
  (*speed_set).speed_set_1 = speed_rate;
  (*speed_set).speed_set_2 = speed_rate;
  (*speed_set).speed_set_3 = speed_rate;
  (*speed_set).speed_set_4 = speed_rate;
}

Movement_Combine_Arg* All_Direction_Movement::back(Speed_Set* speed_set, int speed_rate, Movement_Combine_Arg* m_c_a) {
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

Movement_Combine_Arg* All_Direction_Movement::left(Speed_Set* speed_set, int speed_rate, Movement_Combine_Arg* m_c_a) {
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
Movement_Combine_Arg* All_Direction_Movement::right(Speed_Set* speed_set, int speed_rate, Movement_Combine_Arg* m_c_a) {
  (*m_c_a).speed_1 = -speed_rate;
  (*m_c_a).speed_2 = speed_rate;
  (*m_c_a).speed_3 = speed_rate;
  (*m_c_a).speed_4 = -speed_rate;
  return m_c_a;
}
void All_Direction_Movement::right(Speed_Set* speed_set, int speed_rate) {
  (*speed_set).speed_set_1 = -speed_rate;
  (*speed_set).speed_set_2 = speed_rate;
  (*speed_set).speed_set_3 = speed_rate;
  (*speed_set).speed_set_4 = -speed_rate;
}
void All_Direction_Movement::use_movement_arg(Speed_Set* speed_set, Movement_Combine_Arg* m_c_a_final) {
  (*speed_set).speed_set_1 = (*m_c_a_final).speed_1;
  (*speed_set).speed_set_2 = (*m_c_a_final).speed_2;
  (*speed_set).speed_set_3 = (*m_c_a_final).speed_3;
  (*speed_set).speed_set_4 = (*m_c_a_final).speed_4;
}
Movement_Combine_Arg* All_Direction_Movement::combine_movement_arg(Movement_Combine_Arg* m_c_a_final, Movement_Combine_Arg* m_c_a_x, Movement_Combine_Arg* m_c_a_y) {
  // _print(m_c_a_y);
  // printBreak();
  (*m_c_a_final).speed_1 = (*m_c_a_x).speed_1 + (*m_c_a_y).speed_1;
  (*m_c_a_final).speed_2 = (*m_c_a_x).speed_2 + (*m_c_a_y).speed_2;
  (*m_c_a_final).speed_3 = (*m_c_a_x).speed_3 + (*m_c_a_y).speed_3;
  (*m_c_a_final).speed_4 = (*m_c_a_x).speed_4 + (*m_c_a_y).speed_4;
  return m_c_a_final;
}

void All_Direction_Movement::any(Speed_Set* speed_set, float degree, int speed_rate) {
  
  // 0-360°
  if (degree >= 0 && degree <= 360) {
    /**
    *@notes:如果不用new 的话,执行combine的时候m_c_a_xxx会被销毁，所以必须用new来保证不被销毁, 但是注意要在结束释放,不然会内存泄漏！
    */
    Movement_Combine_Arg* m_c_a_y = new Movement_Combine_Arg;
    Movement_Combine_Arg* m_c_a_x = new Movement_Combine_Arg;
    Movement_Combine_Arg* m_c_a_final = new Movement_Combine_Arg;
    float rad = radians(degree);
    m_c_a_x = all_direction_movement.right(speed_set, (int)(speed_rate * cos(rad)), m_c_a_x);
    m_c_a_y = all_direction_movement.straight(speed_set, (int)(speed_rate * sin(rad)), m_c_a_y);
    // _print();
    // _print(&((*m_c_a_x).speed_2));
    // _print((*m_c_a_y).speed_2);
    // printBreak();
    // _print((*m_c_a_x).speed_2);
    // printBreak();

    m_c_a_final = this->combine_movement_arg(m_c_a_final, m_c_a_x, m_c_a_y);
    this->use_movement_arg(speed_set, m_c_a_final);
    /**
    *@notes:销毁防止内存泄漏
    */
    delete m_c_a_x;
    delete m_c_a_y;
    delete m_c_a_final;
  }
}

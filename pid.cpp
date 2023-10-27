// @开源协议 GPL3.0
// @作者 Bigonion
// @NameSpace bigonion.cn
// @github https://github.com/LiWeny16/ai_based_remote_control_car_arduino_mega2560

#include "Arduino.h"
#include "head.h"

PID_Motor pid_motor_1;
void init_pid() {
  pid_motor_1.pid_init(0, 0, 0, 0, 0, 0);
}

int PID_Motor::constrain_motor_out(int motor_out) {
  return motor_out > 254 ? 254 : (motor_out < -254 ? -254 : motor_out);
}
// @很奇怪，不能用inline这里
void PID_Motor::calculate_pid_motor(Err_Speed* err_speed) {
  this->motor_out_pid = (int)(this->P * ((*err_speed).err_speed_differential_1) + this->I * ((*err_speed).err_speed_now) + this->D * ((*err_speed).err_speed_differential_2));
}

void PID_Motor::pid_control_motor() {

  this->motor_out_now = this->motor_out_last + this->motor_out_pid;
  // Serial.print(this->constrain_motor_out((int)this->motor_out_now));
  Motor_Arg my_motor_arg_1(motor_port_zq, this->constrain_motor_out((int)this->motor_out_now));
  // Motor_Arg my_motor_arg_2(motor_port_yq, 50);
  motor_control(my_motor_arg_1);
  // update
  this->motor_out_last = this->motor_out_now;
  // motor_control(my_motor_arg_2);
}
void Err_Speed::calculate_err_motor(int speed_now, int speed_set) {
  this->err_speed_now = speed_set - speed_now;
  this->err_speed_differential_1 = this->err_speed_now - this->err_speed_last;
  this->err_speed_differential_2 = this->err_speed_now - 2 * this->err_speed_last + this->err_speed_past;
  // @注意：这里顺序非常重要，乱序则为空
  this->err_speed_past = this->err_speed_last;
  this->err_speed_last = this->err_speed_now;
}
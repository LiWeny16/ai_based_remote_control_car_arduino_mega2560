#include "Arduino.h"
#include "head.h"

  
void Err_Speed::calculate_err_motor(int speed_now, int speed_set) {
  this->err_speed_now = speed_set - speed_now;
  this->err_speed_differential_1 = this->err_speed_now - this->err_speed_last;
  this->err_speed_differential_2 = this->err_speed_last - this->err_speed_past;
  this->err_speed_last = this->err_speed_now;
  this->err_speed_past = this->err_speed_last;
}
// @开源协议 GPL3.0
// @作者 Bigonion
// @NameSpace bigonion.cn
// @github https://github.com/LiWeny16/ai_based_remote_control_car_arduino_mega2560

#include "Arduino.h"
#include "head.h"

class Base_Movement {
public:
  virtual void straight(Speed_Set* speed_set) {}
  virtual void back(Speed_Set* speed_set) {}
  virtual void left(Speed_Set* speed_set) {}
  virtual void right(Speed_Set* speed_set) {}
};

class All_Direction_Movement : public Base_Movement {
public:
  void straight(Speed_Set* speed_set, int speed_rate) {
    (*speed_set).speed_set_1 = speed_rate;
    (*speed_set).speed_set_2 = speed_rate;
    (*speed_set).speed_set_3 = speed_rate;
    (*speed_set).speed_set_4 = speed_rate;
  }
  void back(Speed_Set* speed_set, int speed_rate) {
    (*speed_set).speed_set_1 = -speed_rate;
    (*speed_set).speed_set_2 = -speed_rate;
    (*speed_set).speed_set_3 = -speed_rate;
    (*speed_set).speed_set_4 = -speed_rate;
  }
};
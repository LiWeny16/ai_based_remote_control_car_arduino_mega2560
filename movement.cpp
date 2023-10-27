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
  void straight(Speed_Set* speed_set) override {
    (*speed_set).speed_set_1 = 100;
    (*speed_set).speed_set_2 = 100;
    (*speed_set).speed_set_3 = 100;
    (*speed_set).speed_set_4 = 100;
  }
};
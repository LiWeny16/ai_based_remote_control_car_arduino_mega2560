#ifndef HEAD_H
#define HEAD_H
#include "Arduino.h"
#include "port.h"

// ******************************//Motor//****************
enum DIR {
  FOR,  // Forward;
  BACK  // Backward;
};

class Motor_Port {
public:
  int p1;
  int p2;
  void motor_port_init(int p1, int p2) {
    this->p1 = p1;
    this->p2 = p2;
  }
};
extern Motor_Port motor_port_zq;
extern Motor_Port motor_port_yq;
extern Motor_Port motor_port_zh;
extern Motor_Port motor_port_yq;
class Motor_Arg {
public:
  Motor_Port port;
  // DIR dir;
  int pwm;
  Motor_Arg(Motor_Port port, int pwm) {
    this->port = port;
    // this->dir = dir;
    this->pwm = pwm;
  }
};

void init_motor();
void motor_control(Motor_Arg motor_arg);

// ******************************//Encoder//****************
// extern int count;
extern int encoder_time;  // 时间标记
extern int encoder_time_old;

class Speed {
public:
  int speed_1;
  int speed_2;
  int speed_3;
  int speed_4;
  float speed_transform_to_real(int speed_1) {
    return (float)speed_1 / 16;
  }
  void speed_init(int speed_1, int speed_2, int speed_3, int speed_4) {
    this->speed_1 = speed_1;
    this->speed_2 = speed_2;
    this->speed_3 = speed_3;
    this->speed_4 = speed_4;
  }
};
extern Speed speed;
// extern int* encoder_count;
void interrupt_sum_encoder_1();
void interrupt_sum_encoder_2();
void speed_calculate(int* count_1);
void init_encoder();
// void sum_encoder(int* count);
void sum_encoder(int* count, int port_interrupt, int port_data);
// void sum_encoder(int count);
// ******************************//Ultrasonic//****************

extern float ultra_distance;
void init_ultrasonic();
float use_ultrasonic();

// ******************************//Enable flag//****************

class En_Motor {
public:
  bool en_motor_1;
  bool en_motor_2;
  bool en_motor_3;
  bool en_motor_4;
};

class En_Encoder{
  bool en_speed_1;
};

class En_All {
public:
  En_Motor en_motor;

};


// ******************************//Others//****************

// @ test variable

// extern volatile int global_temp;

class Test {
public:
  int test = 0;
  float test_f = 0.0;
  void init_test(int test, float test_f) {
    this->test = test;
    this->test_f = test_f;
  }
};
extern Test my_test;



void init_test();
void all_init();
void printBreak();
void init_serial(unsigned long baud);
#endif

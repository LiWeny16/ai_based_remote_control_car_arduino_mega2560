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
void speed_calculate(int* count_1, int* count_2);
void speed_calculate(int* count_1, int* count_2, int* count_3, int* count_4);
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
  void init_en_motor(bool en_motor_1,
                     bool en_motor_2,
                     bool en_motor_3,
                     bool en_motor_4) {
    this->en_motor_1 = en_motor_1;
    this->en_motor_2 = en_motor_2;
    this->en_motor_3 = en_motor_3;
    this->en_motor_4 = en_motor_4;
  }
};

class En_Encoder {
public:
  bool en_speed_1;
  bool en_speed_2;
  bool en_speed_3;
  bool en_speed_4;
  void init_en_encoder(
    bool en_speed_1,
    bool en_speed_2,
    bool en_speed_3,
    bool en_speed_4) {
    this->en_speed_1 = en_speed_1;
    this->en_speed_2 = en_speed_2;
    this->en_speed_3 = en_speed_3;
    this->en_speed_4 = en_speed_4;
  }
};

class En_All {
public:
  bool en_all;
  En_Motor en_motor;
  En_Encoder en_encoder;
  void init_en_all(bool en_all, En_Motor en_motor, En_Encoder en_encoder) {
    this->en_all = en_all;
    this->en_motor = en_motor;
    this->en_encoder = en_encoder;
  }
};
extern En_All en_all_arg;
void init_en();

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

// @开源协议 GPL3.0
// @作者 Bigonion
// @NameSpace bigonion.cn
// @github https://github.com/LiWeny16/ai_based_remote_control_car_arduino_mega2560

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

class Speed_Set {
public:
  int speed_set_1;
  int speed_set_2;
  int speed_set_3;
  int speed_set_4;
  void speed_set_init(int speed_1, int speed_2, int speed_3, int speed_4) {
    this->speed_set_1 = speed_1;
    this->speed_set_2 = speed_2;
    this->speed_set_3 = speed_3;
    this->speed_set_4 = speed_4;
  }
};

extern Speed speed_now;
extern Speed_Set speed_set;
// extern int* encoder_count;
void interrupt_sum_encoder_1();
void interrupt_sum_encoder_2();
void speed_calculate(int* count_1, int* count_2, Speed* speed_now);
// void speed_calculate(int* count_1, int* count_2, int* count_3, int* count_4);
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

// ******************************//PID//****************


class Err_Speed {
public:
  int err_speed_now;
  int err_speed_last;
  int err_speed_past;
  int err_speed_differential_1;
  int err_speed_differential_2;
  void init_err_speed(int err,
                      int err_last,
                      int err_past,
                      int err_speed_differential_1,
                      int err_speed_differential_2) {
    this->err_speed_now = err;
    this->err_speed_last = err_last;
    this->err_speed_past = err_past;
    this->err_speed_differential_1 = err_speed_differential_1;
    this->err_speed_differential_2 = err_speed_differential_2;
  }
  /**  
 * @brief 计算PID所需要的误差,可重复调用来精简代码量
 * @param speed_now 现在的速度,从speed_now 类里调用
 * @param speed_set 设置的速度
 */
  void calculate_err_motor(int speed, int speed_set);
};
extern Err_Speed err_speed_1;
class PID_Motor {
public:
  float P;
  float I;
  float D;
  int motor_out_pid;
  int motor_out_now;
  int motor_out_last;
  /**  
 * @brief 计算motor_motor
 * @param err_speed Err_Speed类型
 */
  void calculate_pid_motor(Err_Speed* err_speed);
  int constrain_motor_out(int motor_out);
  void pid_init(float P,
                float I,
                float D,
                int motor_out_pid,
                int motor_out_now,
                int motor_out_last) {
    this->P = P;
    this->I = I;
    this->D = D;
    this->motor_out_pid = motor_out_pid;
    this->motor_out_now = motor_out_now;
    this->motor_out_last = motor_out_last;
  }
  void pid_control_motor();
};
void init_pid();
extern PID_Motor pid_motor_1;
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

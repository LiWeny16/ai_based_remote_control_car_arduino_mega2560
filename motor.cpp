// @开源协议 GPL3.0
// @作者 Bigonion
// @NameSpace bigonion.cn
// @github https://github.com/LiWeny16/ai_based_remote_control_car_arduino_mega2560

#include "Arduino.h"
#include "head.h"

// 全局变量初始化申明
Motor_Port motor_port_zq;
Motor_Port motor_port_yq;
Motor_Port motor_port_zh;
Motor_Port motor_port_yh;
int encoder_time;
int encoder_time_old;
Speed speed_now;
Speed_Set speed_set;
Err_Speed err_speed_1;


void init_motor() {
  motor_port_zq.motor_port_init(Motor_PWM_1_1, Motor_PWM_1_2);
  // motor_port_yq.motor_port_init(Motor_PWM_2_1, Motor_PWM_2_2);
  pinMode(Motor_PWM_1_1, OUTPUT);
  pinMode(Motor_PWM_1_2, OUTPUT);
}
void init_encoder() {
  int encoder_time = 0;
  int encoder_time_old = 0;
  speed_now.speed_init(0, 0, 0, 0);
  speed_set.speed_set_init(0, 0, 0, 0);
  err_speed_1.init_err_speed(0, 0, 0, 0, 0);
  pinMode(Encoder_1_1, INPUT);
  pinMode(Encoder_1_2, INPUT);
}



// Motor_Arg Motor_Arg_1;

void motor_control(Motor_Arg motor_arg) {
  // 防止烧毁电机
  motor_arg.pwm = (motor_arg.pwm > 255 || motor_arg.pwm < -255) ? 0 : motor_arg.pwm;
  if (motor_arg.pwm > 0 || motor_arg.pwm == 0) {
    analogWrite(motor_arg.port.p1, motor_arg.pwm);
    analogWrite(motor_arg.port.p2, 0);
  } else {
    analogWrite(motor_arg.port.p1, 255 + motor_arg.pwm);
    analogWrite(motor_arg.port.p2, 255);
  }
}
void speed_calculate(int* count_1, int* count_2, Speed* speed_now) {
  encoder_time = millis();                         //以毫秒为单位，计算当前时间
  if (abs(encoder_time - encoder_time_old) >= 20)  // 如果计时时间已达20ms秒
  {
    detachInterrupt(digitalPinToInterrupt(Encoder_1_1));  // 关闭外部中断
    detachInterrupt(digitalPinToInterrupt(Encoder_2_1));
    (*speed_now).speed_1 = *count_1;
    (*speed_now).speed_2 = *count_2;
    *count_1 = 0;
    *count_2 = 0;
    encoder_time_old = millis();  // 记录每秒测速时的时间节点

    //当电平发生改变时触发中断函数
    en_all_arg.en_encoder.en_speed_1 ? (attachInterrupt(digitalPinToInterrupt(Encoder_1_1), interrupt_sum_encoder_1, CHANGE)) : (void)1;
    en_all_arg.en_encoder.en_speed_2 ? (attachInterrupt(digitalPinToInterrupt(Encoder_2_1), interrupt_sum_encoder_2, CHANGE)) : (void)1;
  }
}
// 重载，方便调试
// void speed_calculate(int* count_1, int* count_2, int* count_3, int* count_4) {
//   encoder_time = millis();                         //以毫秒为单位，计算当前时间
//   if (abs(encoder_time - encoder_time_old) >= 20)  // 如果计时时间已达20ms秒
//   {
//     detachInterrupt(digitalPinToInterrupt(Encoder_1_1));  // 关闭外部中断0
//     detachInterrupt(digitalPinToInterrupt(Encoder_1_1));  // 关闭外部中断0
//     speed_now.speed_1 = *count_1;
//     *count_1 = 0;
//     encoder_time_old = millis();                                                           // 记录每秒测速时的时间节点
//     attachInterrupt(digitalPinToInterrupt(Encoder_1_1), interrupt_sum_encoder_1, CHANGE);  // 重新开放外部中断0
//   }
// }
void sum_encoder(int* count, int port_interrupt, int port_data) {
  if (digitalRead(port_data) == HIGH) {
    if (digitalRead(port_interrupt) == LOW) {  //B 高 A 低
      (*count)++;
    } else {  //B 高 A 高
      (*count)--;
    }
  } else {
    if (digitalRead(port_interrupt) == HIGH) {  //B 低 A 高
      (*count)++;
    } else {  //B 低 A 低
      (*count)--;
    }
  }
}
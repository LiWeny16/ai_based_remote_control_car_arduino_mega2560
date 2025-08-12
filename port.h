// @开源协议 GPL3.0
// @作者 Bigonion
// @NameSpace bigonion.cn
// @github https://github.com/LiWeny16/ai_based_remote_control_car_arduino_mega2560

#ifndef PORT_H
#define PORT_H

#include "Arduino.h"

// @ echo def
#define Ultra_Trig 22  //引脚Tring 连接 IO D2
#define Ultra_Echo 23  //引脚Ultra_Echo 连接 IO D3

// @ PWM Port def
#define Motor_PWM_1_1 4
#define Motor_PWM_1_2 5 

#define Motor_PWM_2_1 6
#define Motor_PWM_2_2 7

#define Motor_PWM_3_1 8
#define Motor_PWM_3_2 9

#define Motor_PWM_4_1 10
#define Motor_PWM_4_2 11

#define Servo_PWM 12
// @ Encoder
#define Encoder_1_1 2  //负责中断
#define Encoder_1_2 24 // 判断正反

#define Encoder_2_1 3
#define Encoder_2_2 25

#define Encoder_3_1 18
#define Encoder_3_2 26

#define Encoder_4_1 19
#define Encoder_4_2 27

// @ Soft serial
// 8266
#define Soft_Serial_1_RX 52
#define Soft_Serial_1_TX 53
 
#endif
#ifndef PORT_H
#define PORT_H

#include "Arduino.h"

// @ echo def
#define Ultra_Trig 22  //引脚Tring 连接 IO D2
#define Ultra_Echo 23  //引脚Ultra_Echo 连接 IO D3

// @ PWM Port def
#define Motor_PWM_1_1 4
#define Motor_PWM_1_2 5

#define Encoder_1_1 2 //负责中断
#define Encoder_1_2 24 // 判断正反
#define Encoder_2_1 3
#define Encoder_2_2 25
// #define Encoder_3_1 20
// #define Encoder_3_2 21
// #define Encoder_4_1 20
// #define Encoder_4_2 21

#endif
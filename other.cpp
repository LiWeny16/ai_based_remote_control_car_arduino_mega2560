// @开源协议 GPL3.0
// @作者 Bigonion
// @NameSpace bigonion.cn
// @github https://github.com/LiWeny16/ai_based_remote_control_car_arduino_mega2560
#include "Arduino.h"
#include "head.h"

// 全局变量的初始化申明
Test my_test;
En_Motor en_motor;
En_Encoder en_encoder;
En_All en_all_arg;

void init_en() {
  // 电机使能
  en_motor.init_en_motor(true, true, true, true, true);
  // 编码器使能
  en_encoder.init_en_encoder(true, true, true, true, true);
  // 全局使能
  en_all_arg.init_en_all(true, en_motor, en_encoder);
}
// 调试函数
void printBreak() {
  Serial.println("\n");
}
void _print(int a){
  Serial.print(a);
}
void _print(float a){
  Serial.print(a);
}
void _println(int a){
  Serial.println(a);
}


void init_test() {
  my_test.init_test(1, 1.123);
}


// void myPrint(const String &s){
//    Serial.println(s);
// }
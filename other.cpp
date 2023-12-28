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
void _print(int a) {
  Serial.print(a);
}
void _print(float a) {
  Serial.print(a);
}
void _print(char a) {
  Serial.print(a);
}
void _println(int a) {
  Serial.println(a);
}


void init_test() {
  my_test.init_test(1, 1.123);
}

int sb(){
  return 0;
}

// void split_string() {
//   p = strtok(input, ",");  // 第一次调用 strtok
//   while (p != NULL) {       // 当 p 不为空时，继续拆分
//     Serial.println(p);      // 打印每个子字符串
//     p = strtok(NULL, ",");  // 再次调用 strtok
//   }
// }


// void myPrint(const String &s){
//    Serial.println(s);
// }
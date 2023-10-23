#include "Arduino.h"
#include "head.h"

// 全局变量的初始化申明
Test my_test;

void init_serial(unsigned long baud) {
  Serial.begin(baud);
  Serial.println("Mega 2560 is ready!");
}

// 调试函数
void printBreak() {
  Serial.println("\n");
}

void init_test() {
  my_test.init_test(1, 1.123);
}
// void myPrint(const String &s){
//    Serial.println(s);
// }
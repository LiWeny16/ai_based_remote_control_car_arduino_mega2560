#include "head.h"
float ultra_distance;  //距离变量

volatile int count_1 = 0;
volatile int count_2 = 0;
volatile int count_3 = 0;
volatile int count_4 = 0;
// @WHY：为什么写在这里呢，被逼无奈，extern 和volatile 在arduino不能同时使用，无法跨文件传全局变量，
// 又因为中断无法实现传参，只能写在主函数了
// Arduino 的跨文件数据结构做的很不好，非常不适合比较复杂的项目
// 下次改用STM32算了
void interrupt_sum_encoder_1() {
  sum_encoder(&count_1, Encoder_1_1, Encoder_1_2);
}
void interrupt_sum_encoder_2() {
  sum_encoder(&count_2, Encoder_2_1, Encoder_2_2);
}



void setup() {
  all_init();
  EIFR = _BV(INTF4);  //清除MEGA 2560中断0请求标志位//引脚2
  EIFR = _BV(INTF5);  //清除MEGA 2560中断1请求标志位//引脚3
  en_all_arg.en_encoder.en_speed_1 ? (attachInterrupt(digitalPinToInterrupt(Encoder_1_1), interrupt_sum_encoder_1, CHANGE)) : (void)1;
  en_all_arg.en_encoder.en_speed_2 ? attachInterrupt(digitalPinToInterrupt(Encoder_2_1), interrupt_sum_encoder_2, CHANGE) : (void)1;  //当电平发生改变时触发中断函数
  delay(3000);
}

void loop() {
  // ultra_distance = use_ultrasonic();
  Motor_Arg my_motor_arg_1(motor_port_zq, 150);
  Motor_Arg my_motor_arg_2(motor_port_yq, 50);
  motor_control(my_motor_arg_1);
  motor_control(my_motor_arg_2);
  speed_calculate(&count_1, &count_2);
  // printBreak();
  Serial.print(en_all_arg.en_encoder.en_speed_1);
  printBreak();
  // Serial.println("V1=");
  Serial.print(speed.speed_1);
  // Serial.print(speed.speed_1);
  // Serial.print(my_test.test);
  // Serial.println("m/s");
  // printBreak();
  // Serial.println("V2=");
  // Serial.print(speed.speed_transform_to_real(speed.speed_2));
  // Serial.println("m/s");
  // Serial.println("my_motor_arg.dir");
  delay(100);
}
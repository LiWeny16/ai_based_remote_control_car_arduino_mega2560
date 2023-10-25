// @开源协议 GPL3.0
// @作者 Bigonion
// @NameSpace bigonion.cn
// @github https://github.com/LiWeny16/ai_based_remote_control_car_arduino_mega2560

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
  if (en_all_arg.en_all) {
    EIFR = _BV(INTF4);  //清除MEGA 2560中断0请求标志位//引脚2
    EIFR = _BV(INTF5);  //清除MEGA 2560中断1请求标志位//引脚3
    (attachInterrupt(digitalPinToInterrupt(Encoder_1_1), interrupt_sum_encoder_1, CHANGE));
    attachInterrupt(digitalPinToInterrupt(Encoder_2_1), interrupt_sum_encoder_2, CHANGE);  //当电平发生改变时触发中断函数
    delay(3000);
  }
}

void loop() {
  if (en_all_arg.en_all) {
    // ultra_distance = use_ultrasonic();
    Motor_Arg my_motor_arg_1(motor_port_zq, 120);
    Motor_Arg my_motor_arg_2(motor_port_yq, 50);
    motor_control(my_motor_arg_1);
    motor_control(my_motor_arg_2);

    // 计算速度,写入speed_now
    speed_calculate(&count_1, &count_2, &speed_now);

    // 计算误差,写入err_speed
    err_speed.calculate_err_motor(speed_now.speed_1, speed_set.speed_set_1);

    Serial.println("speed_now: ");
    Serial.print(speed_now.speed_1);
    printBreak();
    Serial.println("speed_set.speed_set_1: ");
    Serial.print(speed_set.speed_set_1);
    printBreak();
    Serial.println("err_speed.err_speed_now: ");
    Serial.print(err_speed.err_speed_now);
    // Serial.print(count_1);
    // printBreak();
    // Serial.println("speed_set: ");
    // Serial.print(speed_set.speed_set_1);


    printBreak();
    // Serial.println("V1=");

    // Serial.print(speed_now.speed_1);
    // Serial.print(my_test.test);
    // Serial.println("m/s");
    // printBreak();
    // Serial.println("V2=");
    // Serial.print(speed_now.speed_transform_to_real(speed_now.speed_2));
    // Serial.println("m/s");
    // Serial.println("my_motor_arg.dir");
  }

  delay(100);
}
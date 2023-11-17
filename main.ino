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
String char_sum;

unsigned long speed_previous_time = 0;
unsigned long pid_previous_time = 0;
const unsigned long speed_sample_interval = 20;
const unsigned long pid_control_interval = 50;

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
void interrupt_sum_encoder_3() {
  sum_encoder(&count_3, Encoder_3_1, Encoder_3_2);
}
void interrupt_sum_encoder_4() {
  sum_encoder(&count_4, Encoder_4_1, Encoder_4_2);
}



void setup() {
  all_init();
  if (en_all_arg.en_all) {
    EIFR = _BV(INTF4);  //清除MEGA 2560中断0请求标志位//引脚2
    EIFR = _BV(INTF5);  //清除MEGA 2560中断1请求标志位//引脚3
    attachInterrupt(digitalPinToInterrupt(Encoder_1_1), interrupt_sum_encoder_1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Encoder_2_1), interrupt_sum_encoder_2, CHANGE);  //当电平发生改变时触发中断函数
    attachInterrupt(digitalPinToInterrupt(Encoder_3_1), interrupt_sum_encoder_3, CHANGE);  //当电平发生改变时触发中断函数
    attachInterrupt(digitalPinToInterrupt(Encoder_4_1), interrupt_sum_encoder_4, CHANGE);  //当电平发生改变时触发中断函数
    delay(3000);
  }
}

void loop() {
  // 使能
  if (en_all_arg.en_all) {
    // 处理串口,无延时
    handle_serial_from_8266(&Serial_8266, &char_sum);

    unsigned long current_time = millis();  // 获取当前时间
    // 判断是否达到延迟时间间隔
    if (current_time - speed_previous_time >= speed_sample_interval) {

      // ultra_distance = use_ultrasonic();

      // 计算速度,写入speed_now
      speed_calculate(&count_1, &count_2, &count_3, &count_4, &speed_now);

      // 调整速度
      // speed_set.speed_set_1 = 0;
      // speed_set.speed_set_2 = 0;
      // speed_set.speed_set_3 = 0;
      // speed_set.speed_set_4 = 0;

      // Motor_Arg my_motor_arg_2(motor_port_yq, 50);
      // motor_control(my_motor_arg_2);

      // 计算误差,更新写入err_speed
      err_speed_1.calculate_err_motor(speed_now.speed_1, speed_set.speed_set_1);
      err_speed_2.calculate_err_motor(speed_now.speed_2, speed_set.speed_set_2);
      err_speed_3.calculate_err_motor(speed_now.speed_3, speed_set.speed_set_3);
      err_speed_4.calculate_err_motor(speed_now.speed_4, speed_set.speed_set_4);

      pid_motor_1.P = 0.25;
      pid_motor_1.I = 0.10;
      pid_motor_1.D = 0;


      pid_motor_2.P = 0;
      pid_motor_2.I = 0.3;
      pid_motor_2.D = 0;

      pid_motor_3.P = 0;
      pid_motor_3.I = 0.3;
      pid_motor_3.D = 0;

      pid_motor_4.P = 0;
      pid_motor_4.I = 0.3;
      pid_motor_4.D = 0;

      // 计算PID修正量
      pid_motor_1.calculate_pid_motor(&err_speed_1);
      pid_motor_2.calculate_pid_motor(&err_speed_2);
      pid_motor_3.calculate_pid_motor(&err_speed_3);
      pid_motor_4.calculate_pid_motor(&err_speed_4);

      // 执行PID输出
      if (en_all_arg.en_motor.en_motor_all) {
        en_all_arg.en_motor.en_motor_1 ? pid_motor_1.pid_control_motor(motor_port_zq) : (void)1;
        en_all_arg.en_motor.en_motor_2 ? pid_motor_2.pid_control_motor(motor_port_yq) : (void)1;
        en_all_arg.en_motor.en_motor_3 ? pid_motor_3.pid_control_motor(motor_port_zh) : (void)1;
        en_all_arg.en_motor.en_motor_4 ? pid_motor_4.pid_control_motor(motor_port_yh) : (void)1;
      }

      // Serial.println("speed_now: ");
      // Serial.print(speed_now.speed_1);
      // printBreak();
      // Serial.println("speed_set.speed_set_1: ");
      // Serial.print(speed_set.speed_set_1);
      // printBreak();

      // Serial.println("err_speed.now ");
      // Serial.print(err_speed_1.err_speed_now);
      // printBreak();
      // Serial.println("err_speed.d1 ");
      // Serial.print(err_speed_1.err_speed_differential_1);
      // printBreak();
      // Serial.println("err_speed.d2: ");
      // Serial.print(err_speed_1.err_speed_differential_2);

      // Serial.print(count_1);
      // printBreak();
      // Serial.println("speed_set: ");
      // Serial.print(speed_set.speed_set_1);


      // Serial.println("V1=");
      // Serial.print(speed_now.speed_1);
      // printBreak();
      // Serial.println("V2=");
      // Serial.print(speed_now.speed_2);
      // printBreak();
      // Serial.println("V3=");
      // Serial.print(speed_now.speed_3);
      // printBreak();
      // Serial.println("V4=");
      // Serial.print(speed_now.speed_4);
      // printBreak();


      // Serial.print(my_test.test);
      // Serial.println("m/s");
      // printBreak();
      // Serial.println("V2=");
      // Serial.print(speed_now.speed_transform_to_real(speed_now.speed_2));
      // Serial.println("m/s");
      // Serial.println("my_motor_arg.dir");

      speed_previous_time = current_time;  // 更新上一次执行函数的时间
    }
    if (current_time - pid_previous_time >= pid_control_interval) {
      pid_previous_time = current_time;
    }
  }

  // delay(200);
}
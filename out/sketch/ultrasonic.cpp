#line 1 "C:\\Learning\\sample\\Arduino\\FYP\\Mega 2560\\main\\ultrasonic.cpp"
// @开源协议 GPL3.0
// @作者 Bigonion
// @NameSpace bigonion.cn
// @github https://github.com/LiWeny16/ai_based_remote_control_car_arduino_mega2560

#include "Arduino.h"
#include "head.h"

void init_ultrasonic() {
  pinMode(Ultra_Trig, OUTPUT);
  pinMode(Ultra_Echo, INPUT);
}

// @描述：超声波功能函数
// @return 转换距离
float use_ultrasonic() {
  float cm;    //距离变量
  float temp;  //
  //给Ultra_Trig发送一个低高低的短时间脉冲,触发测距
  digitalWrite(Ultra_Trig, LOW);   //给Ultra_Trig发送一个低电平
  delayMicroseconds(2);            //等待 2微妙
  digitalWrite(Ultra_Trig, HIGH);  //给Ultra_Trig发送一个高电平
  delayMicroseconds(10);           //等待 10微妙
  digitalWrite(Ultra_Trig, LOW);   //给Ultra_Trig发送一个低电平

  temp = float(pulseIn(Ultra_Echo, HIGH));  //存储回波等待时间,
  //pulseIn函数会等待引脚变为HIGH,开始计算时间,再等待变为LOW并停止计时
  //返回脉冲的长度

  //声速是:340m/1s 换算成 34000cm / 1000000μs => 34 / 1000
  //因为发送到接收,实际是相同距离走了2回,所以要除以2
  //距离(厘米)  =  (回波时间 * (34 / 1000)) / 2
  //简化后的计算公式为 (回波时间 * 17)/ 1000

  cm = (temp * 17) / 1000;  //把回波时间换算成cm
  return cm;
}
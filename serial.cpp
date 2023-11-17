// @开源协议 GPL3.0
// @作者 Bigonion
// @NameSpace bigonion.cn
// @github https://github.com/LiWeny16/ai_based_remote_control_car_arduino_mega2560
#include "Arduino.h"
#include "head.h"

SoftwareSerial Serial_8266(Soft_Serial_1_RX, Soft_Serial_1_TX);


void init_serial(unsigned long baud) {
  Serial.begin(baud);
  Serial.println("Mega 2560 is ready!");
  Serial_8266.begin(9600);
}

void handle_serial_from_8266(SoftwareSerial* Serial_8266, String* char_sum) {
  if ((*Serial_8266).available() > 0)  //串口接收到数据
  {
    char b = (*Serial_8266).read();
    *char_sum += b;
    if (b == '?') {
      // string1_past = string1_last;
      // string1_last = string1;
      // string1 = char_sum;
      // Serial.print(*char_sum);
      String splitResult = (*char_sum).substring(0, (*char_sum).length() - 1);
      Serial.print(splitResult);
      // ip:port = 192.168.173.202/switch?a=1&b=0&c=20
      // speed_set.speed_set_1 = 0;
      // speed_set.speed_set_2 = splitResult.toInt();
      // speed_set.speed_set_3 = splitResult.toInt();
      // speed_set.speed_set_4 = splitResult.toInt();

      if (splitResult.toInt() == 25) {
        all_direction_movement.stop();
      } else {
        all_direction_movement.back(&speed_set, splitResult.toInt());
      }
      *char_sum = "";
    }
  }
}
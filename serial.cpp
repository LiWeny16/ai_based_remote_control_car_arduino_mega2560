#include "WString.h"
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
    Receive_Arg_Movement* receive_arg_movement = new Receive_Arg_Movement;
    (*receive_arg_movement).arg_1 = "";
    (*receive_arg_movement).arg_2 = "";
    char b = (*Serial_8266).read();
    
    Serial.println((int)'0');
    Serial.println((int)b);
    if (b>='0')
      *char_sum += b;
    if (b == '?') {
      // string1_past = string1_last;
      // string1_last = string1;
      // string1 = char_sum;
      // Serial.print(*char_sum);
      /**
      * @note 把最后一个"?"去掉
      */
      String* split_result = new String;
      Serial.println(*char_sum);
      (*split_result) = (*char_sum).substring(0, (*char_sum).length() - 1);
      String tmp = *split_result;
      // String control_type = *(&split_result);
      // char* choice = new char;
      // *choice = (*split_result)[0];
      Serial.println(tmp[3]);
      Serial.println(*split_result);
      // if ((*split_result)[2] == '2') {
      //   Serial.println("aaa");
      // }
      // if ((*split_result)[2] == '7') {
      //   Serial.println("bbb");
      // }
      // Serial.println(&((*split_result)[2]))

      // switch (choice) {
      //   case 1:
      //     break;
      //   case 2:
      //     break;
      //   default:
      // }

      // const char* myString = "Hello, World!";
      // const char* secondCharPtr = myString + 1;
      // Serial.print(*secondCharPtr);

      // Serial.print(split_result);
      // @format: ip:port = 192.168.173.202/switch?a=1&b=0&c=20
      // speed_set.speed_set_4 = split_result.toInt();

      // 获取字符串的长度
      // int _length = (*split_result).length();

      // // 创建动态大小的字符数组
      // char* myCharArray = new char[_length + 1];  // 需要额外的空间来存储结尾的空字符 '\0'

      // // 将字符串复制到字符数组中
      // (*split_result).toCharArray(myCharArray, _length + 1);

      // // 输出字符数组
      // // Serial.println(myCharArray);

      // char* p = strtok(myCharArray, ",");
      // for (int i = 0; p != NULL; i++) {
      //   p = strtok(NULL, ",");
      // }

      // 释放动态分配的内存

      // if ((*split_result).toInt() == 25) {
      //   //  all_direction_movement.straight(&speed_set, 10, m_c_a_1);
      //   all_direction_movement.any(&speed_set, 50, 10);
      // } else {
      //   all_direction_movement.straight(&speed_set, (*split_result).toInt());
      // }
      *char_sum = "";
      delete split_result;
      // delete choice;
      delete receive_arg_movement;
      // delete[] myCharArray;
    }
  }
}
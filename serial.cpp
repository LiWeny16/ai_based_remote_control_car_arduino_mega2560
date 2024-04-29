#include "WString.h"
// @开源协议 GPL3.0
// @作者 Bigonion
// @NameSpace bigonion.cn
// @github https://github.com/LiWeny16/ai_based_remote_control_car_arduino_mega2560
#include "Arduino.h"
#include "head.h"

SoftwareSerial Serial_8266(Soft_Serial_1_RX, Soft_Serial_1_TX);
bool SerialLock;

void init_serial(unsigned long baud) {
  Serial.begin(baud);
  Serial.println("Mega 2560 is ready!");
  Serial_8266.begin(9600);
  SerialLock = true;
}
void handle_serial_to_8266(SoftwareSerial *Serial_8266, Speed *speed_now) {
  if (!SerialLock) {
    (*Serial_8266).print((*speed_now).speed_1);
    (*Serial_8266).print("|");
    (*Serial_8266).print((*speed_now).speed_2);
    (*Serial_8266).print("|");
    (*Serial_8266).print((*speed_now).speed_3);
    (*Serial_8266).print("|");
    (*Serial_8266).print((*speed_now).speed_4);
    (*Serial_8266).print("?");
  }
}
void handle_serial_from_8266(SoftwareSerial *Serial_8266, String *char_sum, Speed *speed_now) {

  if ((*Serial_8266).available() > 0)  // 串口接收到数据
  {
    // 锁住
    SerialLock = true;
    Receive_Arg_Movement *receive_arg_movement = new Receive_Arg_Movement;
    Receive_Arg_Movement *receive_arg_movement2 = new Receive_Arg_Movement;
    char b = (*Serial_8266).read();
    // Serial.println((int)'0');
    // Serial.print(b);
    // if (b >= 32)
    (*char_sum) += b;
    // else
    // 13,0
    // Serial.println((int)b);
    if (b == '?') {
      SerialLock = false;
      // (*Serial_8266).print((*speed_now).speed_1);
      // (*Serial_8266).print("|");
      // (*Serial_8266).print((*speed_now).speed_2);
      // (*Serial_8266).print("|");
      // (*Serial_8266).print((*speed_now).speed_3);
      // (*Serial_8266).print("|");
      // (*Serial_8266).print((*speed_now).speed_4);
      // (*Serial_8266).print("?");
      /**
       * @note 把最后一个"?"去掉
       */
      String *split_result = new String;
      // Serial.println(*char_sum);
      (*split_result) = (*char_sum).substring(0, (*char_sum).length() - 1);
      // Serial.println(*split_result);
      // String control_type = *(&split_result);
      // char* choice = new char;
      const char choice = (*split_result)[0];
      // String choice = ((*split_result));
      // Serial.println(choice);
      switch (choice) {
          // 全向移动
        case '1':
          // 获取字符串的长度
          const short _length = (*split_result).length();
          int *degree = new int;
          int *speed_rate = new int;

          // // 创建动态大小的字符数组
          char *myCharArray = new char[_length + 1];  // 需要额外的空间来存储结尾的空字符 '\0'
          // 将字符串复制到字符数组中
          (*split_result).toCharArray(myCharArray, _length + 1);
          char *p = strtok(myCharArray, ",");
          for (int i = 0; p != NULL; i++) {
            if (i > 0) {
              /**
             * @note 角度
             * */
              i == 1 ? ((receive_arg_movement)->arg_1 = p) : (char *)'1';
              /**
             * @note 速度
             */
              i == 2 ? ((receive_arg_movement)->arg_2 = p) : (char *)'1';
            }
            p = strtok(NULL, ",");
          }
          (*degree) = (int)atoi((*receive_arg_movement).arg_1);
          if ((*degree) > 360 || (*degree) < 0) {
            (*degree) = 0;
          }
          (*speed_rate) = (int)atoi((*receive_arg_movement).arg_2);
          if ((*speed_rate) > 30 || (*speed_rate) < 0) {
            (*speed_rate) = 0;
          }
          // Serial.print("degree:");
          // Serial.println((int)*degree);
          // Serial.print("rate:");
          // Serial.println((int)*speed_rate);
          all_direction_movement.any(&speed_set, (*degree), (*speed_rate));
          // 释放内存
          delete[] myCharArray;
          delete degree;
          delete speed_rate;
          break;
        case '2':
          // 即2,1 顺时针 2,2逆时针
          (receive_arg_movement2)->arg_1 = (*split_result)[2];
          if ((receive_arg_movement2)->arg_1 == "1") {
            all_direction_movement.rotate_clockwise(&speed_set, 10);
          } else if ((receive_arg_movement2)->arg_1 == "2") {
            all_direction_movement.rotate_clockwise(&speed_set, 10);
          }
          break;
        default:
          break;
      }

      // if ((*split_result)[2] == '2') {
      //   Serial.println("aaa");
      // }
      // if ((*split_result)[2] == '7') {
      //   Serial.println("bbb");
      // }
      // Serial.println(&((*split_result)[2]))

      // const char* myString = "Hello, World!";
      // const char* secondCharPtr = myString + 1;
      // Serial.print(*secondCharPtr);

      // Serial.print(split_result);
      // @format: ip:port = 192.168.173.202/switch?a=1&b=0&c=20
      // speed_set.speed_set_4 = split_result.toInt();

      // 释放动态分配的内存

      // if ((*split_result).toInt() == 25) {
      //   //  all_direction_movement.straight(&speed_set, 10, m_c_a_1);
      //   all_direction_movement.any(&speed_set, 50, 10);
      // } else {
      //   all_direction_movement.straight(&speed_set, (*split_result).toInt());
      // }
      *char_sum = "";
      delete split_result;
      delete receive_arg_movement;
      delete receive_arg_movement2;
    }
  }
}
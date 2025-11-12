#include "WString.h"
// @开源协议 GPL3.0
// @作者 Bigonion
// @NameSpace bigonion.cn
// @github https://github.com/LiWeny16/ai_based_remote_control_car_arduino_mega2560
#include "Arduino.h"
#include "head.h"
#include <Arduino_FreeRTOS.h>
#include <queue.h>

SoftwareSerial Serial_8266(Soft_Serial_1_RX, Soft_Serial_1_TX);
bool SerialLock;

// FreeRTOS 队列句柄
QueueHandle_t serialCommandQueue;
#define QUEUE_LENGTH 10
#define COMMAND_MAX_LENGTH 64

// 命令结构体
struct SerialCommand {
  char data[COMMAND_MAX_LENGTH];
  uint8_t length;
};

void init_serial(unsigned long baud) {
  Serial.begin(baud);
  Serial.println("Mega 2560 is ready!");
  Serial_8266.begin(9600);
  SerialLock = false;
  
  // 创建队列用于存储接收到的命令
  serialCommandQueue = xQueueCreate(QUEUE_LENGTH, sizeof(SerialCommand));
  if (serialCommandQueue == NULL) {
    Serial.println("Failed to create serial command queue!");
  }
}

void handle_serial_to_8266(SoftwareSerial *Serial_8266, Speed *speed_now) {
  (*Serial_8266).print((*speed_now).speed_1);
  (*Serial_8266).print("|");
  (*Serial_8266).print((*speed_now).speed_2);
  (*Serial_8266).print("|");
  (*Serial_8266).print((*speed_now).speed_3);
  (*Serial_8266).print("|");
  (*Serial_8266).print((*speed_now).speed_4);
  (*Serial_8266).print("|");
  (*Serial_8266).print(ultra_distance);
  (*Serial_8266).print("?");
}

/**
 * @brief 串口接收任务 - 只负责接收数据并放入队列
 * @note 该任务运行在高优先级，快速响应串口数据
 */
void Task_Serial_Receive(void *pvParameters) {
  static String char_sum = "";
  SerialCommand cmd;
  
  for (;;) {
    if (Serial_8266.available() > 0) {
      char b = Serial_8266.read();
      char_sum += b;
      
      // 检测到命令结束符
      if (b == '?') {
        // 准备命令数据
        cmd.length = min(char_sum.length(), (unsigned int)COMMAND_MAX_LENGTH - 1);
        char_sum.toCharArray(cmd.data, cmd.length + 1);
        
        // 发送到队列（非阻塞）
        if (xQueueSend(serialCommandQueue, &cmd, 0) != pdPASS) {
          Serial.println("Queue full! Command dropped.");
        }
        
        // 清空缓冲区
        char_sum = "";
      }
      
      // 防止缓冲区溢出
      if (char_sum.length() > COMMAND_MAX_LENGTH) {
        Serial.println("Command too long! Resetting buffer.");
        char_sum = "";
      }
    }
    
    // 短暂延时，让出CPU给其他任务
    vTaskDelay(pdMS_TO_TICKS(5)); // 5ms检查一次
  }
}

/**
 * @brief 命令解析任务 - 从队列中取出命令并解析执行
 * @note 该任务运行在较低优先级，不会阻塞串口接收
 */
void Task_Command_Parser(void *pvParameters) {
  SerialCommand cmd;
  
  for (;;) {
    // 等待队列中的命令（阻塞等待）
    if (xQueueReceive(serialCommandQueue, &cmd, portMAX_DELAY) == pdPASS) {
      
      // 解析命令
      String command_str = String(cmd.data);
      
      // 去掉结束符'?'
      command_str = command_str.substring(0, command_str.length() - 1);
      
      if (command_str.length() == 0) {
        continue;
      }
      
      const char choice = command_str[0];
      
      switch (choice) {
        case '1': {
          // 全向移动命令: 1,<angle>,<speed>?
          int firstComma = command_str.indexOf(',');
          int secondComma = command_str.indexOf(',', firstComma + 1);
          
          if (firstComma > 0 && secondComma > firstComma) {
            String angleStr = command_str.substring(firstComma + 1, secondComma);
            String speedStr = command_str.substring(secondComma + 1);
            
            int degree = angleStr.toInt();
            int speed_rate = speedStr.toInt();
            
            // 参数验证
            if (degree < 0 || degree > 360) {
              degree = 0;
              Serial.println("Invalid angle, set to 0");
            }
            if (speed_rate < 0 || speed_rate > 30) {
              speed_rate = 0;
              Serial.println("Invalid speed, set to 0");
            }
            
            // 执行运动命令
            all_direction_movement.any(&speed_set, degree, speed_rate);
            
            Serial.print("Move: angle=");
            Serial.print(degree);
            Serial.print(", speed=");
            Serial.println(speed_rate);
          }
          break;
        }
        
        case '2': {
          // 旋转命令: 2,<direction>?
          // 1=顺时针, 2=逆时针
          if (command_str.length() >= 3) {
            char direction = command_str[2];
            
            if (direction == '1') {
              all_direction_movement.rotate_clockwise(&speed_set, 20);
              Serial.println("Rotate clockwise");
            } else if (direction == '2') {
              all_direction_movement.rotate_anti_clockwise(&speed_set, 20);
              Serial.println("Rotate counter-clockwise");
            }
          }
          break;
        }
        
        case '3': {
          // 停止命令: 3?
          all_direction_movement.stop_use_pid(&speed_set);
          Serial.println("Stop");
          break;
        }
        
        default:
          Serial.print("Unknown command: ");
          Serial.println(choice);
          break;
      }
    }
  }
}

/**
 * @brief 初始化串口相关的FreeRTOS任务
 * @note 在setup()中调用此函数来创建任务
 */
void init_serial_tasks() {
  // 创建串口接收任务（高优先级）
  xTaskCreate(
    Task_Serial_Receive,
    "SerialRx",
    256,  // 栈大小
    NULL,
    3,    // 优先级：高
    NULL
  );
  
  // 创建命令解析任务（中等优先级）
  xTaskCreate(
    Task_Command_Parser,
    "CmdParser",
    512,  // 栈大小（解析需要更多栈空间）
    NULL,
    2,    // 优先级：中
    NULL
  );
  
  Serial.println("Serial tasks created successfully!");
}

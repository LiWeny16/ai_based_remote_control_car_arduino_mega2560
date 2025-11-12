// @开源协议 GPL3.0
// @作者 Bigonion
// @NameSpace bigonion.cn
// @github https://github.com/LiWeny16/ai_based_remote_control_car_arduino_mega2560

/**
 * @file main_freertos_serial.ino
 * @brief 使用FreeRTOS实现非阻塞串口命令解析的示例
 * 
 * 主要改进：
 * 1. 串口接收和命令解析分离，使用队列通信
 * 2. 接收任务高优先级快速响应，不会丢失数据
 * 3. 解析任务独立运行，不会阻塞其他功能
 * 4. 支持命令缓冲，即使处理较慢也不会丢失命令
 */

#include "head.h"
#include <Arduino_FreeRTOS.h>
#include "Arduino.h"

float ultra_distance;
volatile int count_1 = 0;
volatile int count_2 = 0;
volatile int count_3 = 0;
volatile int count_4 = 0;
int i = 40;
bool increasing = true;

// 中断函数保持不变
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

// 速度计算任务
void Task_Speed_Calculation(void *pvParameters) {
  for (;;) {
    speed_calculate(&count_1, &count_2, &count_3, &count_4, &speed_now);
    vTaskDelay(pdMS_TO_TICKS(20));  // 20ms 运行一次
  }
}

// PID控制任务
void Task_PID_Control(void *pvParameters) {
  for (;;) {
    // 计算误差
    err_speed_1.calculate_err_motor(speed_now.speed_1, speed_set.speed_set_1);
    err_speed_2.calculate_err_motor(speed_now.speed_2, speed_set.speed_set_2);
    err_speed_3.calculate_err_motor(speed_now.speed_3, speed_set.speed_set_3);
    err_speed_4.calculate_err_motor(speed_now.speed_4, speed_set.speed_set_4);

    // 设置PID参数
    pid_motor_1.P = 0.37;
    pid_motor_1.I = 0.44;
    pid_motor_1.D = 0.15;

    pid_motor_2.P = 0.37;
    pid_motor_2.I = 0.44;
    pid_motor_2.D = 0.15;

    pid_motor_3.P = 0.37;
    pid_motor_3.I = 0.44;
    pid_motor_3.D = 0.15;

    pid_motor_4.P = 0.42;
    pid_motor_4.I = 0.44;
    pid_motor_4.D = 0.15;

    // 计算 PID 修正
    pid_motor_1.calculate_pid_motor(&err_speed_1);
    pid_motor_2.calculate_pid_motor(&err_speed_2);
    pid_motor_3.calculate_pid_motor(&err_speed_3);
    pid_motor_4.calculate_pid_motor(&err_speed_4);

    // 执行 PID 输出
    if (en_all_arg.en_motor.en_motor_all) {
      if (en_all_arg.en_motor.en_motor_1) pid_motor_1.pid_control_motor(motor_port_zq);
      if (en_all_arg.en_motor.en_motor_2) pid_motor_2.pid_control_motor(motor_port_yq);
      if (en_all_arg.en_motor.en_motor_3) pid_motor_3.pid_control_motor(motor_port_zh);
      if (en_all_arg.en_motor.en_motor_4) pid_motor_4.pid_control_motor(motor_port_yh);
    }

    vTaskDelay(pdMS_TO_TICKS(50));  // 50ms 运行一次
  }
}

// 串口通信任务（发送数据到8266）
void Task_Serial_Transmit(void *pvParameters) {
  for (;;) {
    handle_serial_to_8266(&Serial_8266, &speed_now);
    vTaskDelay(pdMS_TO_TICKS(100));  // 100ms 运行一次
  }
}

// 超声波扫描任务
void Task_Ultrasonic(void *pvParameters) {
  for (;;) {
    ultra_distance = use_ultrasonic();

    // 舵机扫描
    if (increasing) {
      if (i < 140) {
        i++;
      } else {
        increasing = false;
      }
    } else {
      if (i > 40) {
        i--;
      } else {
        increasing = true;
      }
    }
    Myservo.write(i);

    vTaskDelay(pdMS_TO_TICKS(20));  // 20ms 运行一次
  }
}

void setup() {
  // 初始化所有模块
  all_init();

  if (en_all_arg.en_all) {
    // 清除中断标志位
    EIFR = _BV(INTF4);
    EIFR = _BV(INTF5);
    
    // 配置编码器中断
    attachInterrupt(digitalPinToInterrupt(Encoder_1_1), interrupt_sum_encoder_1, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Encoder_2_1), interrupt_sum_encoder_2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Encoder_3_1), interrupt_sum_encoder_3, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Encoder_4_1), interrupt_sum_encoder_4, CHANGE);
    
    delay(1000);

    Serial.println("=== Creating FreeRTOS Tasks ===");

    // 创建速度计算任务
    xTaskCreate(Task_Speed_Calculation, "SpeedCalc", 256, NULL, 2, NULL);
    
    // 创建PID控制任务
    xTaskCreate(Task_PID_Control, "PIDCtrl", 256, NULL, 3, NULL);
    
    // 创建串口发送任务
    xTaskCreate(Task_Serial_Transmit, "SerialTx", 256, NULL, 1, NULL);
    
    // 创建超声波任务
    xTaskCreate(Task_Ultrasonic, "Ultrasonic", 128, NULL, 1, NULL);
    
    // 初始化串口任务（接收和解析）
    init_serial_tasks();

    Serial.println("=== All Tasks Created ===");
    Serial.println("=== Starting FreeRTOS Scheduler ===");

    // 启动 FreeRTOS 调度器
    vTaskStartScheduler();
    
    // 如果调度器启动失败，会执行到这里
    Serial.println("ERROR: Scheduler failed to start!");
  }
}

void loop() {
  // FreeRTOS 启动后，loop() 不再执行
  // 如果执行到这里，说明调度器没有正常启动
}

#include <Arduino_FreeRTOS.h>
void Task_Speed_Calculation(void *pvParameters) {
    for (;;) {
        speed_calculate(&count_1, &count_2, &count_3, &count_4, &speed_now);
        vTaskDelay(pdMS_TO_TICKS(20));  // 20ms 运行一次
    }
}
void Task_PID_Control(void *pvParameters) {
    for (;;) {
        // 计算误差
        err_speed_1.calculate_err_motor(speed_now.speed_1, speed_set.speed_set_1);
        err_speed_2.calculate_err_motor(speed_now.speed_2, speed_set.speed_set_2);
        err_speed_3.calculate_err_motor(speed_now.speed_3, speed_set.speed_set_3);
        err_speed_4.calculate_err_motor(speed_now.speed_4, speed_set.speed_set_4);

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
void Task_Serial_Comm(void *pvParameters) {
    for (;;) {
        handle_serial_to_8266(&Serial_8266, &speed_now);
        vTaskDelay(pdMS_TO_TICKS(100));  // 100ms 运行一次
    }
}
void Task_Ultrasonic(void *pvParameters) {
    for (;;) {
        ultra_distance = use_ultrasonic();

        if (increasing) {
            if (i < 140) {
                i++;  // 增加角度
            } else {
                increasing = false;
            }
        } else {
            if (i > 40) {
                i--;  // 减少角度
            } else {
                increasing = true;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));  // 20ms 运行一次
    }
}

void setup() {
    Serial.begin(115200);
    all_init();

    // 创建 FreeRTOS 任务
    xTaskCreate(Task_Speed_Calculation, "SpeedCalc", 256, NULL, 2, NULL);
    xTaskCreate(Task_PID_Control, "PIDCtrl", 256, NULL, 3, NULL);
    xTaskCreate(Task_Serial_Comm, "SerialComm", 256, NULL, 1, NULL);
    xTaskCreate(Task_Ultrasonic, "Ultrasonic", 128, NULL, 1, NULL);

    // 启动 FreeRTOS 调度器
    vTaskStartScheduler();
}

void loop() {
    // FreeRTOS 启动后，loop() 不再执行
}

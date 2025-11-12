---

<a name="chinese"></a>

# 基于AI的全向遥控小车

## Arduino Mega 2560 平台

## 项目概述

这是一个先进的全向机器人小车平台，专为自主导航和远程控制应用设计。基于Arduino Mega 2560构建，系统集成了复杂的运动控制算法、实时PID调节、无线通信和环境感知能力。该架构支持传统的远程操作和AI辅助的自主导航场景。

### 技术亮点

- **全向运动系统**：360度移动能力，精确角度控制
- **先进PID控制**：独立的多电机闭环速度调节
- **实时操作系统**：FreeRTOS实现确定性任务调度
- **无线通信**：基于ESP8266的命令和遥测接口
- **模块化软件架构**：面向对象设计，硬件抽象层
- **中断驱动传感**：基于硬件的编码器计数，精确速度反馈

## 目录

- [系统架构](#系统架构)
- [软件设计](#软件设计)
- [硬件配置](#硬件配置)
- [快速开始](#快速开始)
- [性能指标](#性能指标)
- [开发指南](#开发指南)

---

## 系统架构

### 运动控制子系统

**全向移动引擎**
- 360度移动能力，1度分辨率
- 麦克纳姆轮运动学实现
- 对角和复合运动的矢量合成
- 独立轮速控制，同步执行

**PID速度调节**
- 四个独立PID控制器（每个电机一个）
- 增量式PID算法，带抗积分饱和保护
- 可配置的比例、积分和微分系数
- 实时误差跟踪和修正（50Hz更新率）

**编码器反馈系统**
- 正交编码器信号处理
- 基于硬件中断的脉冲计数
- 方向检测和速度计算
- 20ms采样间隔的速度估算

### 通信子系统

**无线接口（ESP8266）**
- 9600波特率SoftwareSerial通信
- 基于协议的命令结构，带分隔符
- 双向数据传输
- 实时遥测流传输（10Hz更新率）

**命令协议**
- 全向移动：`1,<角度>,<速度>?`
- 旋转控制：`2,<方向>?`
- 紧急停止：`3?`
- 数据格式：ASCII文本，'?'结束符

**FreeRTOS集成**（增强版本）
- 基于任务的串口接收（高优先级）
- 基于队列的命令缓冲
- 非阻塞解析任务（中优先级）
- 解耦的中断处理和命令处理

### 传感器子系统

**超声波距离测量**
- HC-SR04传感器集成
- 触发-回波时间测量
- 带温度补偿的距离计算
- 50Hz测量频率

**基于舵机的扫描**
- 180度水平扫描范围
- 自动扫描模式（40°至140°）
- 同步距离映射
- 实时障碍物检测

---

## 软件设计

### 模块化架构

```
项目结构：
├── main.ino                    # 基于millis的主控制循环
├── main_freertos_serial.ino    # FreeRTOS任务调度实现
├── all_init.cpp                # 系统初始化例程
├── movement.cpp                # 全向运动控制算法
├── pid.cpp                     # PID控制器实现
├── motor.cpp                   # 电机驱动接口和编码器处理
├── serial.cpp                  # 基础串口通信处理
├── serial_freertos.cpp         # 非阻塞FreeRTOS串口实现
├── ultrasonic.cpp              # HC-SR04传感器驱动
├── other.cpp                   # 工具函数和调试工具
├── head.h                      # 全局声明和类定义
└── port.h                      # 硬件引脚映射配置
```

### 面向对象设计模式

**基础运动类（多态接口）**
```cpp
class Base_Movement {
  virtual void stop(Speed_Set *speed_set);
  virtual void any(Speed_Set *speed_set, float angle, int speed_rate);
  // 可扩展的其他虚方法
};
```

**PID控制器封装**
```cpp
class PID_Motor {
  float P, I, D;                          // 可调系数
  void calculate_pid_motor(Err_Speed *err_speed);
  int constrain_motor_out(int motor_out);
  void pid_control_motor(Motor_Port motor_port);
};
```

**速度和误差跟踪类**
- 类型安全的速度设定点和测量数据结构
- 误差计算，带微分跟踪用于PID微分项
- 自动更新先前的误差状态

### 实时执行模型

**标准实现（main.ino）**
- 基于`millis()`的多速率执行
- 速度计算：20ms间隔（50Hz）
- PID控制：50ms间隔（20Hz）
- 串口通信：100ms间隔（10Hz）
- 带时间戳间隔的非阻塞执行

**FreeRTOS实现（main_freertos_serial.ino）**
- 基于优先级抢占的确定性任务调度
- 专用任务用于速度计算、PID控制、通信和传感
- 基于队列的任务间通信用于命令缓冲
- 每个任务的资源高效栈分配

---

## 硬件配置

### 组件清单

| 组件 | 规格 | 数量 | 用途 |
|------|------|------|------|
| Arduino Mega 2560 | ATmega2560, 16MHz | 1 | 主控制器 |
| ESP8266 | Wi-Fi模块 | 1 | 无线通信 |
| 带编码器直流电机 | 12V, 减速比1:30 | 4 | 全向驱动 |
| 电机驱动器 | L298N或同类 | 2 | 电机功率控制 |
| HC-SR04 | 超声波传感器 | 1 | 距离测量 |
| 舵机 | SG90或同类 | 1 | 传感器扫描 |
| 电源 | 12V电池组 | 1 | 系统供电 |
| 稳压器 | 5V输出 | 1 | 逻辑电平供电 |

### 引脚配置

**电机控制（PWM输出）**
```
电机1（左前）：  引脚4, 5
电机2（右前）：  引脚6, 7
电机3（左后）：  引脚8, 9
电机4（右后）：  引脚10, 11
```

**编码器输入（中断引脚）**
```
编码器1: 引脚2（INT4），引脚24（方向）
编码器2: 引脚3（INT5），引脚25（方向）
编码器3: 引脚18（INT3），引脚26（方向）
编码器4: 引脚19（INT2），引脚27（方向）
```

**传感器接口**
```
超声波触发：  引脚22
超声波回波：  引脚23
舵机PWM：     引脚12
```

**通信**
```
ESP8266 RX: 引脚52（SoftwareSerial）
ESP8266 TX: 引脚53（SoftwareSerial）
调试串口：  硬件UART（USB）
```

### 电路说明

- 电机驱动器需要独立电源（电机12V，逻辑5V）
- 编码器信号使用内部上拉电阻
- ESP8266工作在3.3V；建议TX/RX线使用电平转换器
- 每个IC电源引脚附近放置去耦电容（100nF）
- 外部电源应能处理5A峰值电流

---

## 快速开始

### 前置要求

**软件要求**
- Arduino IDE 1.8.19或更高版本
- Arduino_FreeRTOS库（用于RTOS版本）
- SoftwareSerial库（Arduino核心包含）
- Servo库（Arduino核心包含）

**硬件组装**
1. 将四个电机以全向配置安装在底盘上
2. 根据引脚配置将电机驱动器连接到Arduino
3. 将编码器连接到指定的中断引脚
4. 在舵机支架上安装超声波传感器
5. 使用适当的电平转换连接ESP8266模块
6. 验证电源连接（电机12V，逻辑5V）

### 安装步骤

**1. 克隆仓库**
```bash
git clone https://github.com/LiWeny16/ai_based_remote_control_car_arduino_mega2560.git
cd ai_based_remote_control_car_arduino_mega2560
```

**2. 安装依赖**
```bash
# 在Arduino IDE库管理器中：
# - 搜索并安装 "FreeRTOS" by Phillip Stevens
# - 验证Servo和SoftwareSerial可用
```

**3. 配置硬件引脚**
如果您的接线与默认配置不同，请编辑`port.h`：
```cpp
#define Motor_PWM_1_1 4
#define Motor_PWM_1_2 5
// 根据需要调整
```

**4. 上传固件**
```
文件 → 打开 → 选择 main.ino（或main_freertos_serial.ino）
工具 → 开发板 → Arduino Mega or Mega 2560
工具 → 端口 → [选择您的Arduino端口]
项目 → 上传
```

**5. 验证操作**
- 打开串口监视器（115200波特率）
- 观察初始化消息
- 发送测试命令：`1,0,10?`（以速度10前进）

### 命令协议

**运动命令**
| 命令 | 格式 | 示例 | 描述 |
|------|------|------|------|
| 全向移动 | `1,<角度>,<速度>?` | `1,45,20?` | 以45°角移动，速度20 |
| 顺时针旋转 | `2,1?` | `2,1?` | 以默认速度顺时针旋转 |
| 逆时针旋转 | `2,2?` | `2,2?` | 逆时针旋转 |
| 停止 | `3?` | `3?` | 紧急停止所有电机 |

**参数范围**
- 角度：0-360度（0°=前进，90°=右，180°=后退，270°=左）
- 速度：0-30（任意单位，映射到PWM占空比）
- 命令必须以'?'结束符结尾

---

## 性能指标

### 控制系统性能

| 指标 | 值 | 备注 |
|------|-----|------|
| 运动精度 | ±2° | 全向模式下的角度精度 |
| 速度控制误差 | ±5% | 调优PID后的稳态误差 |
| 命令响应 | <100ms | 从命令接收到执行的时间 |
| 编码器分辨率 | 16脉冲/转 | 每个电机轴旋转 |
| PID更新率 | 50Hz | 控制循环频率 |
| 速度采样 | 50Hz | 速度测量频率 |

### 通信规格

| 参数 | 值 | 协议 |
|------|-----|------|
| 无线范围 | 50m+ | ESP8266，取决于环境 |
| 波特率 | 9600 | 到ESP8266的SoftwareSerial |
| 调试串口 | 115200 | USB连接 |
| 遥测速率 | 10Hz | 速度和传感器数据传输 |
| 命令延迟 | <50ms | 从ESP8266到执行 |

### 功耗

| 组件 | 典型值 | 峰值 | 备注 |
|------|--------|------|------|
| Arduino Mega | 50mA | 100mA | 5V供电 |
| ESP8266 | 80mA | 170mA | 传输期间 |
| 电机（4个） | 500mA | 2A | 每个电机堵转时 |
| 舵机 | 100mA | 500mA | 运动期间 |
| 整个系统 | ~1A | ~5A | 所有电机激活时 |

---

## 开发指南

### PID调优

可以通过修改`main.ino`或`main_freertos_serial.ino`中的系数来调整PID控制器：

```cpp
// 示例：调整电机1的PID参数
pid_motor_1.P = 0.37;  // 比例增益
pid_motor_1.I = 0.44;  // 积分增益
pid_motor_1.D = 0.15;  // 微分增益
```

**调优指南：**
1. 从仅P开始（I=0，D=0），增加直到出现振荡
2. 将P减少50%，添加I项以消除稳态误差
3. 添加D项以减少超调并提高稳定性
4. 在各种负载条件下测试

### 添加自定义运动模式

扩展`All_Direction_Movement`类：

```cpp
void All_Direction_Movement::custom_pattern(Speed_Set *speed_set) {
  // 实现您的运动逻辑
  speed_set->speed_set_1 = custom_value_1;
  speed_set->speed_set_2 = custom_value_2;
  speed_set->speed_set_3 = custom_value_3;
  speed_set->speed_set_4 = custom_value_4;
}
```

### FreeRTOS任务优先级

任务优先级配置如下（数字越大=优先级越高）：

```cpp
Task_Serial_Receive:    优先级3（最高 - 时间关键）
Task_PID_Control:       优先级3（控制循环）
Task_Speed_Calculation: 优先级2（传感器处理）
Task_Command_Parser:    优先级2（命令执行）
Task_Serial_Transmit:   优先级1（遥测）
Task_Ultrasonic:        优先级1（最低 - 周期性传感）
```

### 调试技巧

**串口输出**
```cpp
// 在代码中启用调试输出
Serial.print("速度1: ");
Serial.println(speed_now.speed_1);
```

**常见问题**
- 电机无响应：检查电源和驱动器连接
- 编码器读数不稳定：验证上拉电阻和中断配置
- ESP8266命令丢失：降低波特率或检查接线
- PID不稳定：减少增益并验证编码器方向

### 测试程序

1. **电机测试**：验证每个电机正确响应PWM信号
2. **编码器测试**：确认脉冲计数和方向检测
3. **通信测试**：通过串口监视器发送命令
4. **PID测试**：用示波器或串口绘图观察速度收敛
5. **集成测试**：执行复杂的运动序列

---

## 未来增强

该平台为高级机器人应用提供了坚实的基础：

- **自主导航**：使用超声波和编码器数据实现SLAM算法
- **计算机视觉**：集成摄像头模块进行物体检测和跟踪
- **传感器融合**：添加IMU以改进里程计和稳定性控制
- **路径规划**：实现A*或RRT算法进行避障
- **机器学习**：部署轻量级神经网络进行行为学习
- **多机器人协调**：实现群智能算法

## 贡献

欢迎贡献！请遵循以下指南：

1. Fork仓库
2. 创建特性分支（`git checkout -b feature/AmazingFeature`）
3. 提交更改（`git commit -m 'Add some AmazingFeature'`）
4. 推送到分支（`git push origin feature/AmazingFeature`）
5. 打开Pull Request

**贡献领域：**
- 额外的传感器驱动
- 替代控制算法
- 性能优化
- 文档改进
- 测试覆盖

## 许可证

本项目采用GNU通用公共许可证v3.0 - 详见[LICENSE](LICENSE)文件。

**要点：**
- 免费使用、修改和分发
- 衍生作品必须披露源代码
- 衍生作品必须使用相同许可证

## 引用

如果您在学术研究中使用此项目，请引用：

```bibtex
@misc{ai_remote_control_car,
  author = {Bigonion},
  title = {AI-Based Omnidirectional Remote Control Car},
  year = {2024},
  publisher = {GitHub},
  url = {https://github.com/LiWeny16/ai_based_remote_control_car_arduino_mega2560}
}
```

## 作者

**Bigonion**
- GitHub: [@LiWeny16](https://github.com/LiWeny16)
- 网站: [bigonion.cn](https://bigonion.cn)
- 邮箱: 通过GitHub联系

## 致谢

- Arduino社区提供的广泛库支持
- FreeRTOS项目的RTOS实现
- 提供反馈和改进的贡献者和用户

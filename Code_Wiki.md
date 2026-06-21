# CYT4BB 开源库 — 完整 Code Wiki（飞控示例）

> 本文档基于对 `Seekfree_CYT4BB_Opensource_Library` 仓库的静态代码分析生成，
> 重点介绍当前工程中 **四旋翼飞行器飞控示例** 的软件架构、模块职责、
> 关键数据结构/函数、依赖关系、构建方式以及代码改进建议。
>
> **目标硬件**：Infineon / Cypress Traveo-II CYT4BB（CM0+ + CM7_0 + CM7_1，主频 250 MHz，带 FPU）
> **开发环境**：IAR Embedded Workbench for ARM 9.40.1
> **许可证**：GPLv3（参见 `libraries/doc/GPL3_permission_statement.txt`）
> **官方渠道**：https://seekfree.taobao.com/
> **仓库版本**：`libraries/doc/version.txt`（V3.9.3 起）

---

## 1. 项目整体架构

### 1.1 软件分层结构（自下而上）

```
┌────────────────────────────────────────────────────────────────────┐
│ 应用层 (Application)                                                │
│   project/user/main_cm7_0.c   → CM7_0 主函数（系统入口、主循环）   │
│   project/user/main_cm7_1.c   → CM7_1 主函数（当前空循环）         │
│   project/user/cm7_0_isr.c    → CM7_0 中断服务程序（PIT/UART/EXTI）│
│   project/.../Exe/control.c   → 飞控核心控制循环（双环 PID + 混控）│
│   project/.../Exe/pid.c       → PID 控制器（位置式 + 增量式 + LPF）│
│   project/.../Exe/motor.c     → 四电机 PWM 初始化与 X 型混控       │
│   project/.../Exe/zita.c      → IMU 读取 + 低通滤波 + Mahony 互补滤波 │
│   project/.../Exe/init.c      → 应用层总初始化（当前未启用）       │
│   project/.../Exe/pin.h       → 引脚宏定义（当前空壳）             │
│   project/.../Exe/image.c/.h  → 图像处理 / 连通域 / Beacon 识别（未启用）│
│   project/.../Exe/Beacon.c/.h → 信标定位 / 光点识别（未启用）       │
├────────────────────────────────────────────────────────────────────┤
│ zf_components（组件层）                                             │
│   seekfree_assistant.c/.h   → 上位机助手协议                       │
├────────────────────────────────────────────────────────────────────┤
│ zf_device（外部模块驱动层，30+ 个）                                  │
│   IMU：imu660ra / imu660rb / imu660rc / imu963ra / icm20602         │
│   测距：dl1a / dl1b（激光 TOF）                                      │
│   显示：ips114 / ips200 / ips200pro / oled / tft180                 │
│   视觉：mt9v03x（灰度全局曝光摄像头）                                 │
│   线性 CCD：tsl1401                                                   │
│   编码器：menc15a                                                      │
│   光流：pmw3901                                                       │
│   GNSS：gnss                                                            │
│   无线：ble6a20 / wifi_uart / wifi_spi / wireless_uart               │
│   串口接收框架：uart_receiver                                         │
├────────────────────────────────────────────────────────────────────┤
│ zf_driver（片上外设驱动层，16 个）                                    │
│   gpio / uart / spi / soft_spi / soft_i2c                           │
│   pwm / pit / adc / dma / timer / encoder / exti / flash / ipc      │
├────────────────────────────────────────────────────────────────────┤
│ zf_common（通用工具层，7 个）                                         │
│   zf_common_typedef（通用类型定义）                                  │
│   zf_common_clock（系统时钟配置）                                    │
│   zf_common_debug（调试信息输出 / printf 重定向）                    │
│   zf_common_fifo（环形缓冲区）                                       │
│   zf_common_font（ASCII / 中文字库）                                │
│   zf_common_function（通用工具函数）                                 │
│   zf_common_interrupt（全局中断管理）                                │
├────────────────────────────────────────────────────────────────────┤
│ Cypress 官方 SDK（底层）                                              │
│   CMSIS 头文件 / arm_math.h（DSP）                                    │
│   cy_gpio / cy_uart / cy_tcpwm / cy_ipc / cy_sysclk ...             │
│   startup_cm0plus.s / startup_cm7.s（启动文件）                      │
└────────────────────────────────────────────────────────────────────┘
```

### 1.2 项目目录结构

```
Seekfree_CYT4BB_Opensource_Library/
├─ project/
│   ├─ user/                         ← 用户主程序与中断（内核入口）
│   │   ├─ main_cm7_0.c              ← CM7_0 主入口（飞控主循环）
│   │   ├─ main_cm7_1.c              ← CM7_1 主入口（当前空循环）
│   │   ├─ cm7_0_isr.c               ← CM7_0 中断服务程序（PIT/UART/EXTI）
│   │   └─ cm7_1_isr.c               ← CM7_1 中断服务程序
│   ├─ iar/
│   │   ├─ cyt4bb7.eww               ← IAR 工作区（Workspace）
│   │   ├─ project_config/
│   │   │   ├─ cyt4bb7_cm_7_0.ewp    ← CM7_0 工程文件
│   │   │   ├─ cyt4bb7_cm_7_1.ewp    ← CM7_1 工程文件
│   │   │   └─ Debug_m7_0/
│   │   │      └─ Exe/               ← 飞控示例应用层源文件
│   │   │         ├─ control.c/h    → 控制循环（双环 PID + 混控）
│   │   │         ├─ pid.c/h        → PID 控制器
│   │   │         ├─ motor.c/h      → 电机 PWM 与 X 型混控
│   │   │         ├─ zita.c/h       → IMU 读取与姿态解算
│   │   │         ├─ pin.h           → 引脚宏定义（当前空壳）
│   │   │         ├─ init.c/h        → 应用层初始化（未启用）
│   │   │         ├─ image.c/h       → 图像处理与连通域（未启用）
│   │   │         └─ Beacon.c/h      → 信标识别（未启用）
│   │   └─ icf/
│   │      └─ linker_directives_tviibh4m.icf  ← 链接脚本
│   └─ code/                         ← 用户自定义代码目录
└─ libraries/                        ← 核心开源库（SDK + 封装层）
    ├─ doc/
    │   ├─ version.txt                ← 版本与更新日志
    │   └─ GPL3_permission_statement.txt ← 许可声明
    ├─ sdk/                           ← Cypress 官方 SDK（CMSIS + 驱动）
    ├─ zf_common/                     ← 通用工具层
    ├─ zf_driver/                     ← 片上外设驱动层
    ├─ zf_device/                     ← 外部模块驱动层
    └─ zf_components/                 ← 组件层（Seekfree Assistant）
```

### 1.3 硬件配置摘要

| 项 | 内容 |
|----|------|
| 主控芯片 | CYT4BB7CEE（Traveo-II Body High，144-pin TEQFP） |
| 内核数量 | 3 个（CM0+ + CM7_0 + CM7_1） |
| 主频 | CM7 使用 `SYSTEM_CLOCK_250M`（250 MHz） |
| FPU / DSP | CM7 内置 FPv5 FPU + DSP 指令集（可通过 `arm_math.h` 使用） |
| IMU | IMU660RA（SPI 接口，提供三轴加速度 / 角速度） |
| LED 心跳指示 | `P19_0`（在主循环中翻转） |
| 电机驱动 | 4 路 PWM，使用 TCPWM 通道 `CH30_P10_2 / CH57_P17_4 / CH11_P05_2 / CH20_P08_1` |
| 调试串口 | UART0（`debug_init()` 配置，默认阻塞式） |

---

## 2. 启动流程与系统初始化

### 2.1 启动流程（CM7_0）

```
上电复位（POR）
   │
   ▼
CM0+ 启动（startup_cm0plus.s），打开电源与时钟域
   │
   ▼
CM0+ 通过 IPC 使能 CM7_0 内核
   │
   ▼
CM7_0 内核启动（startup_cm7.s）：
   ├─ 初始化 .data / .bss 段
   ├─ 配置 MSP（主堆栈指针）
   ├─ 开启 FPU 硬件浮点单元
   └─ 跳转到 main()（project/user/main_cm7_0.c）
   │
   ▼
main() 初始化序列（见 2.2）
   │
   ▼
while(true) 主循环 + PIT_CH0 每 10 ms 节拍驱动
```

### 2.2 main() 初始化序列（`project/user/main_cm7_0.c`）

```c
// 1. 系统时钟树初始化，CM7 主频 250 MHz
clock_init(SYSTEM_CLOCK_250M);

// 2. 调试串口初始化（UART0，默认 115200，printf 重定向）
debug_init();

// 3. LED1  GPIO 初始化（P19_0，推挽输出，默认低电平）
gpio_init(LED1, GPO, GPIO_LOW, GPO_PUSH_PULL);

// 4. 周期中断定时器 PIT_CH0，每 10 ms 触发一次（即 100 Hz 控制节拍）
pit_ms_init(PIT_NUM, 10);

// 5. IMU660RA SPI 初始化（由 zf_device_imu660ra 提供）
imu660ra_init();

// 6. 等待 IMU 稳定后进行陀螺仪零偏校准（≈ 1 秒）
system_delay_ms(100);
CalibrateGyro();

// 7. 控制循环初始化：初始化全部 PID + 电机 PWM
Control_Init();

// 8. 确保初始电机输出为 0（防止上电瞬间意外输出）
Motor_Set(0, 0, 0, 0);
system_delay_ms(500);

// 9. 主循环：100 Hz 节拍驱动
while(true)
{
    if(pit_state)
    {
        pit_state = 0;
        imu660ra();               // 读取 IMU → LPF → Mahony → 欧拉角
        printf("Roll:%.2f Pitch:%.2f Yaw:%.2f\n",
               attitude.roll, attitude.pitch, attitude.yaw);
        Control_Loop();           // 姿态外环 + 角速度内环 + 混控 + 油门递增
        gpio_toggle_level(LED1);  // 心跳指示
    }
}
```

### 2.3 中断服务程序（`project/user/cm7_0_isr.c`）

| 中断处理函数 | 作用 |
|--------------|------|
| `pit0_ch0_isr()` | 10 ms 控制节拍：置位 `pit_state = 1`，清中断标志 |
| `pit0_ch1_isr() ~ pit0_ch21_isr()` | 其他通道占位（仅清中断标志；其中 `ch21` 内部调用 `tsl1401_collect_pit_handler()`） |
| `uart0_isr()` | UART0 调试串口中断（默认未启用，通过 `DEBUG_UART_USE_INTERRUPT` 宏开关） |
| `uart1_isr()` | 调用 `wireless_module_uart_handler()` 处理无线模块 |
| `uart2_isr()` | 调用 `gnss_uart_callback()` 处理 GNSS 模块 |
| `uart4_isr()` | 调用 `uart_receiver_handler()` 通用串口接收框架 |
| `gpio_*_exti_isr()` | 外部中断回调模板（当前未启用具体功能） |

---

## 3. 应用层模块详解（飞控示例）

### 3.1 整体控制框图（主数据流）

```
+------------------+
|     IMU660RA     |  (SPI 读取)
|  acc_x/y/z [g]   |
|  gyro_x/y/z [°/s]|
+--------+---------+
         |
         ▼
+-----------------------------+
|      zita.c                 |
|  1. 减去 gyro_offset[3]     |  <-- 由 CalibrateGyro() 在初始化时采样得到
|  2. apply_lpf() × 2         |  <-- 加速度 LPF α=0.2, 角速度 α=0.5
|  3. mahony_update()         |  <-- Mahony 互补滤波: q0~q3 → Euler angle
|     → attitude.roll/pitch/yaw  (°)
+--------+-----------------------------+
         |
         ▼
+-----------------------------------------------+
|                  control.c                    |
|  1. 期望姿态 desired = (0, 0, 0)              |
|  2. 外环：位置式 PID_Realize(angle)            |
|     → target_rate_roll / pitch / yaw          |
|  3. 内环：增量式 PID_Increase(gyro_filt)       |
|     → torque_roll / pitch / yaw               |
|  4. 油门递增 throttle += 2，直到 600           |
|  5. MotorMixer(throttle, torques) → PWM[1..4]|
+--------+---------------------------------------+
         |
         ▼
+-------------------------------------------+
|                motor.c                    |
|  MotorMixer X 型混控：                    |
|    m1 = throttle + roll - pitch + yaw     |  (PWM_CH1)
|    m2 = throttle + roll + pitch - yaw     |  (PWM_CH2)
|    m3 = throttle - roll + pitch + yaw     |  (PWM_CH3)
|    m4 = throttle - roll - pitch - yaw     |  (PWM_CH4)
|  钳制到 [0, 1000] → pwm_set_duty()        |
+-------------------------------------------+
```

> **说明（已修复）**：旧版本中 `Control_Loop()` 对 `m1/m2/m3/m4` 的打印始终为 0
> —— `control.c` 中是全局变量，而 `MotorMixer()` 内部使用的是同名局部变量。
> 现在由 `motor.c` 中的 `motor_out_m1 ~ motor_out_m4` 保存 **钳制后的** 占空比，
> `control.c` 的 `Control_Print()` 直接读取这些全局变量，调试输出值与
> `pwm_set_duty()` 的第二个参数一致，可用于观察真实电机输出。

### 3.2 模块文件与职责总表

| 文件 | 路径 | 职责 | 是否启用 |
|------|------|------|---------|
| main_cm7_0.c | project/user/ | CM7_0 入口、初始化序列、100 Hz 主循环（只调用 API，不关心内部细节） | ✅ |
| cm7_0_isr.c | project/user/ | PIT / UART / EXTI 中断处理（`uart4_isr()` 驱动 SBUS） | ✅ |
| sensor_pack.c / .h | Exe/ | 外设聚合层：DL1B 测距 + PMW3901 光流的 init/update 与数据暴露 | ✅ |
| control.c / .h | Exe/ | 双环 PID 控制循环、油门安全锁 armed、SBUS 期望姿态覆盖、10 Hz 调试打印 | ✅ |
| pid.c / .h | Exe/ | 位置式 / 增量式 PID（LPF 结构已移除，统一走 zita.c 的 apply_lpf） | ✅ |
| motor.c / .h | Exe/ | 电机 PWM 初始化（占空比 0 起步）、Motor_Set、X 型 MotorMixer + clamp_pwm_duty 下限保护 | ✅ |
| zita.c / .h | Exe/ | IMU 读取、陀螺仪零偏校准、apply_lpf 加速度/角速度双路 LPF、Mahony 互补滤波 | ✅ |
| zf_device_dl1b.c / .h | libraries/zf_device/ | VL53L1X 激光测距驱动（软件 I²C，已改 DL1B_SCL_PIN=P26_6, DL1B_SDA_PIN=P26_7） | ✅ |
| zf_device_pmw3901.c / .h | libraries/zf_device/ | PMW3901 光流驱动（硬件 SPI_0，P02_0..3） | ✅ |
| zf_device_uart_receiver.c/.h | libraries/zf_device/ | SBUS 100000 bps / 8E2 / 25 字节帧解析（UART4，P14_0 RX） | ✅ |
| pin.h | Exe/ | 引脚宏定义（当前为空壳，具体引脚以各 zf_device_*.h 为准） | ❌ |
| init.c / .h | Exe/ | 应用层总初始化（当前未在主程序调用） | ❌ |
| Beacon.c / .h | Exe/ | 信标/光点识别（结构体定义完整，逻辑未启用） | ❌ |
| image.c / .h | Exe/ | 图像处理/连通域（结构体定义完整，逻辑未启用） | ❌ |

---

## 4. 关键数据结构与函数参考

### 4.1 PID 控制器（`pid.h` / `pid.c`）

**结构定义**：

```c
typedef struct {
    float P, I, D, limit;      // 比例/积分/微分增益 + 输出限幅
    float Last_Error;          // 上一次误差
    float Previous_Error;      // 上上次误差（仅增量式使用）
    float Integrator;          // 积分累积值
    float output;              // 最终输出
} PID;

typedef struct {
    float alpha;               // 滤波系数 (0~1)
    float last_output;         // 上一次输出
} LowPassFilter;
```

**全局实例**：

| 实例 | 用途 | 类型 | 限幅 |
|------|------|------|------|
| `pid_angle_roll` | Roll 角度外环 | 位置式 PID | 300 |
| `pid_angle_pitch` | Pitch 角度外环 | 位置式 PID | 300 |
| `pid_angle_yaw` | Yaw 角度外环 | 位置式 PID | 100 |
| `pid_rate_roll` | Roll 角速度内环 | 增量式 PID | 500 |
| `pid_rate_pitch` | Pitch 角速度内环 | 增量式 PID | 500 |
| `pid_rate_yaw` | Yaw 角速度内环 | 增量式 PID | 300 |
| `gyro_filter_roll/pitch/yaw` | 角速度 LPF | 一阶低通 | α=0.1 |

**当前 PID 参数（`All_pid_init()` 中设置）**：

| 控制器 | P | I | D | limit |
|--------|---|---|---|-------|
| pid_angle_roll  | **0.0** | **0.0** | 0.0 | 300 |
| pid_angle_pitch | **0.0** | **0.0** | 0.0 | 300 |
| pid_angle_yaw   | **0.0** | **0.0** | 0.0 | 100 |
| pid_rate_roll   | **0.2** | **0.01** | 0.0 | 500 |
| pid_rate_pitch  | 0.0     | 0.0      | 0.0 | 500 |
| pid_rate_yaw    | 0.0     | 0.0      | 0.0 | 300 |

> ⚠️ **当前状态**：角度外环全部为 0，相当于外环被旁路；只有 Roll 角速度内环
> 非零（P=0.2，I=0.01），Pitch / Yaw 无控制作用。需按实际机架调参。

**函数清单**：

| 函数 | 原型 | 说明 |
|------|------|------|
| `PID_Init()` | `void PID_Init(PID *pid, float p, float i, float d, float limit)` | 初始化单个 PID，清零历史误差 |
| `All_pid_init()` | `void All_pid_init(void)` | 初始化全部 6 个 PID + 3 个 LPF |
| `PID_Realize()` | `float PID_Realize(PID *pid, float measurement, float setpoint)` | 位置式 PID（用于角度外环），带积分限幅与输出限幅 |
| `PID_Increase()` | `float PID_Increase(PID *pid, float measurement, float setpoint)` | 增量式 PID（输出 `+=` 增量，并被限幅钳制） |
| `initializeFilter()` | `void initializeFilter(LowPassFilter *filter, float alpha)` | 初始化一阶 LPF |
| `filterValue()` | `float filterValue(LowPassFilter *filter, float input)` | `y = α*x + (1-α)*y_prev` |

> ⚠️ **注意**：`pid.c` 中定义的 `gyro_filter_roll/pitch/yaw` 在当前代码中
> **从未被 `filterValue()` 调用**。实际角速度滤波由 `zita.c: apply_lpf()` 完成
> （α=0.5）。两套低通滤波逻辑重复，见 §7 问题列表 #1。

### 4.2 姿态数据结构与算法（`zita.h` / `zita.c`）

**结构定义**：

```c
typedef struct {
    float pitch;      // 俯仰角 [°]
    float roll;       // 横滚角 [°]
    float yaw;        // 偏航角 [°]
} attitude_t;

extern attitude_t attitude;
extern float gyro_filt[3];       // 滤波后的三轴角速度 [roll pitch yaw]

float acc_filt[3] = {0};         // 滤波后的三轴加速度
float gyro_offset[3] = {0};      // 陀螺仪零偏（上电采样得到）
float q0=1, q1=0, q2=0, q3=0;    // 当前姿态四元数
```

**关键宏与常量**：

| 宏 | 值 | 含义 |
|----|-----|------|
| ACC_LPF_ALPHA | 0.2f | 加速度计低通滤波系数 |
| GYRO_LPF_ALPHA | 0.5f | 陀螺仪低通滤波系数 |
| MAHONY_KP | 5.0f | Mahony 互补滤波比例融合系数 |
| MAHONY_KI | 0.008f | Mahony 互补滤波积分融合系数 |
| SAMPLE_PERIOD | 0.01f | 采样周期（10 ms → 100 Hz） |
| GYRO_OFFSET_SAMPLES | 500 | 陀螺仪零偏校准采样次数（每次 2 ms ≈ 1 秒） |

**函数清单**：

| 函数 | 原型 | 说明 |
|------|------|------|
| `imu660ra()` | `int imu660ra(void)` | 单次 IMU 采样 + 去零偏 + LPF + Mahony 解算，每 10 ms 调用一次 |
| `CalibrateGyro()` | `void CalibrateGyro(void)` | 陀螺仪零偏校准，静态采样 500 次取平均；**调用时必须保持飞行器静止** |
| `apply_lpf()` | `void apply_lpf(float *raw, float *filt, float alpha, int size)` | 对数组逐元素做一阶低通滤波 |
| `mahony_update()` | `void mahony_update(float gx, float gy, float gz, float ax, float ay, float az)` | Mahony 互补滤波核心：加速度计向量归一化 → 叉积误差 → PI 修正角速度 → 四元数微分方程积分 → 归一化 → 欧拉角 |

**Mahony 互补滤波要点**：

```
步骤：
  1. ax, ay, az → 归一化（单位重力向量）
  2. 由当前四元数 (q0,q1,q2,q3) 估计机体坐标系中的重力方向 (vx,vy,vz)
        vx = 2(q1*q3 - q0*q2)
        vy = 2(q0*q1 + q2*q3)
        vz = q0^2 - q1^2 - q2^2 + q3^2
  3. 误差向量：叉积 (ax, ay, az) × (vx, vy, vz)
        ex = ay*vz - az*vy
        ey = az*vx - ax*vz
        ez = ax*vy - ay*vx
  4. PI 补偿陀螺仪偏置，并更新三轴角速度
        gx += Kp*ex + Ki*∫ex
        gy += Kp*ey + Ki*∫ey
        gz += Kp*ez + Ki*∫ez
  5. 四元数微分方程（一阶积分）
        q0 += (-q1*gx - q2*gy - q3*gz) * (Δt/2)
        q1 += ( q0*gx + q2*gz - q3*gy) * (Δt/2)
        q2 += ( q0*gy - q1*gz + q3*gx) * (Δt/2)
        q3 += ( q0*gz + q1*gy - q2*gx) * (Δt/2)
  6. 四元数归一化
  7. 欧拉角（机体 XYZ：Roll-Pitch-Yaw）
        pitch = -asin(-2q1q3 + 2q0q2)     → * RAD_TO_DEG
        roll  = -atan2(2q2q3 + 2q0q1,  -2q1^2 - 2q2^2 + 1)
        yaw   =  atan2(2q1q2 + 2q0q3,  -2q2^2 - 2q3^2 + 1)
```

> ⚠️ **注意事项**：
> - `mahony_update()` 第一行有 `if(ax*ay*az == 0) return;` 的保护，
>   如果三轴任一为 0 则直接退出（防止未初始化数据参与运算）。然而该条件过于
>   严格——一旦其中任一通道数值恰好为 0（例如 IMU 数值抖动过零），则会丢掉
>   一拍解算。更稳健的做法是改用 `ax==0 && ay==0 && az==0` 或直接依赖范数判断。
> - 欧拉角符号与物理含义应结合实际安装方向与右手定则验证
>   （见 §7 问题列表 #6）。

### 4.3 电机与混控（`motor.h` / `motor.c`）

**PWM 通道定义**：

```c
#define PWM_CH1  TCPWM_CH30_P10_2   // 电机 1
#define PWM_CH2  TCPWM_CH57_P17_4   // 电机 2
#define PWM_CH3  TCPWM_CH11_P05_2   // 电机 3
#define PWM_CH4  TCPWM_CH20_P08_1   // 电机 4
```

**函数清单**：

| 函数 | 原型 | 说明 |
|------|------|------|
| `Motor_Init()` | `void Motor_Init(void)` | 初始化 4 路 PWM，50 Hz，默认占空比 500 |
| `Motor_Set()` | `void Motor_Set(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4)` | 直接设置 4 路 PWM 占空比（范围 0 ~ 1000） |
| `MotorMixer()` | `void MotorMixer(float throttle, float roll, float pitch, float yaw)` | X 型混控，根据油门与三轴扭矩计算并输出 4 路 PWM |

**X 型混控公式**（与 `MotorMixer()` 内部一致）：

| 电机 | 公式 | 对应物理动作 |
|------|------|------------|
| M1 (PWM_CH1) | `throttle + roll - pitch + yaw` | 左前（X 型左上） |
| M2 (PWM_CH2) | `throttle + roll + pitch - yaw` | 右前（X 型右上） |
| M3 (PWM_CH3) | `throttle - roll + pitch + yaw` | 左后（X 型左下） |
| M4 (PWM_CH4) | `throttle - roll - pitch - yaw` | 右后（X 型右下） |

> 输出后通过 `LIMIT(x)` 宏钳制到 `[0, 1000]`，再以 `uint16_t` 类型
> 调用 `Motor_Set()` 输出到 `pwm_set_duty()`。

### 4.4 控制循环（`control.h` / `control.c`）

**关键全局/静态变量**：

```c
// 控制内环输出（由 Control_Loop 写入；外部可通过 Control_Print / UART 查看）
static float torque_roll  = 0.0f;   // Roll 轴扭矩
static float torque_pitch = 0.0f;   // Pitch 轴扭矩
static float torque_yaw   = 0.0f;   // Yaw  轴扭矩
static float throttle     = 0.0f;   // 当前油门（0 ~ 1000，SBUS 输入时直接覆盖）

// 来自 SBUS / 上位机的 "期望姿态"（若未设置则保持 (0,0,0)）
static float sbus_des_roll, sbus_des_pitch, sbus_des_yaw;
static float sbus_throttle = -1.0f; // < 0 表示 "未接入 SBUS"，进入递增模式

// armed 安全锁（上电默认 0，通过 Control_Arm(1) 解锁）
static uint8_t armed = 0;
```

**对外 API**：

| 函数 | 说明 |
|------|------|
| `Control_Init()` | 全部 PID 初始化 / `clamp_pwm_duty` 零值启动 / armed 清零 |
| `Control_Loop()` | 每 10 ms 执行一次：**外环 PID → 内环 PID → MotorMixer → PWM**；内部按 10 Hz 调用 `Control_Print()`（避免 100 Hz 阻塞） |
| `Control_Arm(uint8_t)` | 设置油门安全锁；`0 → 1` 解锁前 throttle = 0 |
| `Control_SbusInit()` | 标记 SBUS 准备就绪；由 `main_cm7_0.c` 调用 |
| `Control_Setpoint(roll, pitch, yaw, throttle_pct)` | 外部覆盖期望姿态与油门（可由 SBUS / 上位机 / 按键调用） |
| `Control_Print()` | 把最近一次控制量通过 UART0 打印一次 |

**控制循环时序（`Control_Loop()`）**：

```
1. armed 检查：若未解锁，throttle = 0，MotorMixer 输出全 0，直接返回
2. 若 SBUS 已输入 (sbus_throttle >= 0)：
     throttle = sbus_throttle   （范围 0 ~ 1000）
   否则（无 SBUS 输入）：
     if throttle < 600: throttle += 2
3. desired = (sbus_des_roll, sbus_des_pitch, sbus_des_yaw)  // 默认 (0,0,0)
4. 角度外环（位置式 PID）
     target_rate_* = PID_Realize(&pid_angle_*, attitude.*, desired_*)
5. 角速度内环（增量式 PID）
     torque_* = PID_Increase(&pid_rate_*, gyro_filt[i], target_rate_*)
6. MotorMixer(throttle, torques) → 四路 PWM（已做 [0,1000] 钳制）
7. 每 10 次循环调用一次 Control_Print()（10 Hz 调试输出）
```

### 4.5 信标识别 / 图像处理（未启用）

#### image.h：图像处理与连通域

```
├─ 图像尺寸：原图 188 × 120，重采样 94 × 60
├─ binary_tailor()：二值化与裁剪
├─ Mean_filtering()：均值滤波
├─ 并查集 (union-find) 实现的 connected_components()
└─ ComponentInfo：连通域信息（面积 / 边界 / 重心和）
```

#### Beacon.h：信标识别结构体

```c
#define Beacon_quantity 94               // 信标数量上限
typedef struct {
    int x, y;                            // 信标中心坐标
    int y_lentgh, x_lentgh;              // 长宽（注：原拼写为 lentgh）
    int Beacon_area;                     // 信标面积
    int probability;                     // 信标置信度
} Light;

extern Light light[Beacon_quantity];     // 识别到的信标数组
extern int sign, counts, area, Beacon_probability, Light_x, Light_y;
```

| 函数 | 说明 |
|------|------|
| `xunzhaoguangbiao(char p)` | "寻找光标"主逻辑 |
| `Ideal_environment()` | 理想环境下的识别流程 |
| `type_no_sunlight_interference()` | 无强光干扰情况下的识别流程 |
| `no_Ideal_environment()` | 非理想环境下的处理流程 |

> 当前 `Beacon.c` / `image.c` 虽然在工程目录下存在，但并未在
> `main_cm7_0.c` 中被调用。它们更像是独立的视觉/定位子系统，
> 可被扩展到 CM7_1 内核中并发执行，通过 IPC 与主飞控内核通信。

---

## 5. 依赖关系（头文件 include 图）

### 5.1 统一入口头文件

`zf_common_headfile.h` 是整个项目的唯一入口头文件。任何 `.c` 文件只需
`#include "zf_common_headfile.h` 即可获得：

- Cypress SDK 全部底层头（`cy_project.h` / `cy_device_headers.h` / `arm_math.h`）
- `zf_common` 全部 7 个模块
- `zf_driver` 全部 16 个模块
- `zf_device` 全部 30+ 个模块
- `zf_components`：Seekfree Assistant

### 5.2 应用层 include 关系（当前飞控示例）

```
main_cm7_0.c ──► zf_common_headfile.h
                 (内部拉取 zf_common / zf_driver / zf_device 全部)
                        │
control.c   ──► zf_common_headfile.h
             ├─► control.h ─┬─► zita.h
             │              └─► pid.h
             ├─► pid.h
             ├─► zita.h
             └─► motor.h

pid.c       ──► zf_common_headfile.h
             └─► pid.h

motor.c     ──► zf_common_headfile.h
             └─► motor.h  ──► zf_driver_pwm.h（TCPWM）

zita.c      ──► zf_common_headfile.h
             ├─► zita.h
             └─► zf_device_imu660ra.h（imu660ra_init/get_acc/get_gyro）
```

### 5.3 主要调用链（运行时）

```
main_cm7_0.c : main()
├─ clock_init(SYSTEM_CLOCK_250M)       → zf_common_clock
├─ debug_init()                        → zf_common_debug
├─ gpio_init(LED1, ...)                → zf_driver_gpio
├─ pit_ms_init(PIT_CH0, 10)            → zf_driver_pit
├─ imu660ra_init()                     → zf_device_imu660ra
├─ CalibrateGyro()                     → zita.c（采样 500 次）
└─ Control_Init()                      → control.c
    ├─ All_pid_init()                  → pid.c（6 × PID_Init + 3 × LPF）
    └─ Motor_Init()                     → motor.c（4 × pwm_init）

main_cm7_0.c : while(true)
└─ imu660ra()                          → zita.c
│   ├─ imu660ra_get_acc()
│   ├─ imu660ra_get_gyro()
│   ├─ apply_lpf(raw_acc,  α=0.2)
│   ├─ apply_lpf(raw_gyro, α=0.5)
│   └─ mahony_update(gx,gy,gz, ax,ay,az)  → 四元数 → Euler
│
└─ Control_Loop()                      → control.c
    ├─ 3 × PID_Realize(angle, 0)
    ├─ 3 × PID_Increase(gyro, target_rate)
    ├─ throttle += 2
    ├─ MotorMixer(throttle, torques)   → motor.c
    │   └─ Motor_Set() → pwm_set_duty()
    └─ printf(...)                     → zf_common_debug（UART 阻塞）
```

---

## 6. 项目构建与运行方式

### 6.1 前置要求

1. **IAR Embedded Workbench for ARM 9.40.1**（或兼容版本）
2. 连接 **J-Link / JTAG / SWD** 调试器至 CYT4BB 核心板
3. 核心板供电（推荐 5V / 3A；电机负载较大时需独立供电）

### 6.2 构建步骤（IAR IDE）

1. 在 IAR 菜单 `File → Open → Workspace`，选择 `project/iar/cyt4bb7.eww`
2. 工作区中选择目标工程 `cyt4bb7_cm_7_0`（主飞控内核）
3. 选择配置（默认 `Debug_m7_0`）
4. `Project → Clean` 清理旧产物（重要！修改头文件后必须 Clean）
5. `Project → Make`（或按 **F7**）编译
6. 成功后在 `project/iar/project_config/Debug_m7_0/Exe/` 下生成
   - `cyt4bb7_cm_7_0.out` → ELF 调试文件
   - 对应 Hex 文件（由 IAR 工程配置决定是否生成）

### 6.3 下载与运行

1. 在 IAR 菜单 `Project → Download and Debug`（或 **Ctrl+D**）
2. 全速运行（**F5**）
3. 观察现象：
   - 核心板 `LED1 (P19_0)` 每 10 ms 翻转（心跳指示）
   - 通过调试串口可看到姿态与电机数据：
     ```
     Roll:xx.xx Pitch:xx.xx Yaw:xx.xx
     M1:xxx M2:xxx M3:xxx M4:xxx torR:xx.x torP:xx.x torY:xx.x thr:xxx
     ```

> ⚠️ **安全提示**：由于 `Control_Loop()` 中 `throttle` 会自动递增到 600，
> 在上电前请确保：**飞行器已固定在测试架上**，且桨叶方向已按 X 型混控规则
> 确认（否则上电瞬间可能出现剧烈机动或翻转）。

### 6.4 CM7_1 内核

- 当前 `main_cm7_1.c` 为空循环，可作为独立任务（例如视觉识别、信标定位或
  上位机通信协议解析）运行，通过 `zf_driver_ipc` 与 CM7_0 通信。
- CM7_1 工程 `cyt4bb7_cm_7_1.ewp` 在 IAR 中需单独编译和下载（或由 CM0+ 启动流程决定）。

### 6.5 常见构建问题排查

| 问题 | 原因 | 解决方法 |
|------|------|---------|
| 大量 "unknown type name" | SDK 头文件路径未正确加入 IAR 选项 | 检查 `Project Options → C/C++ → Preprocessor → Additional include directories`，确保 `libraries/sdk/...` 和 `libraries/zf_*/` 全部加入 |
| 编译报缺少 .c 文件 | 源文件未加入 IAR 工程组 | 在 IAR 工程树 `Right-Click Group → Add → Add Files` |
| 某些模块调用后 HardFault | 外设时钟未开启 / 引脚配置错误 / 指针越界 | 先调用对应 `xxx_init()`；检查 GPIO Alternate 功能；使用 IAR 异常分析器定位 |
| printf 无输出 | `debug_init()` 未调用 / 波特率不匹配 | `main()` 开头必须调用 `debug_init()`；检查终端波特率设置 |
| 链接器报 "section `.bss` overflow" | CM7 栈/堆不够，或全局数组过大 | 调整链接脚本 `icf` 中的 `.bss` 区域大小 |

---

## 7. 代码问题、隐患与改进建议

> ⚠️ **本节仅为分析与建议，在实际修改前请务必与我确认**。

### 7.1 已发现的代码问题（✔️ 已在当前版本修复 / ⚠️ 待后续）

| # | 文件 / 位置 | 问题描述 | 严重程度 | 状态 |
|---|-------------|----------|----------|------|
| 1 | `pid.c` 的 `gyro_filter_*` | 定义了 3 个 `LowPassFilter` 实例，但 `filterValue()` 从未被调用；实际角速度滤波由 `zita.c` 的 `apply_lpf()` 完成。两套低通滤波并存。 | ⭐⭐ | ✔️ **已移除** pid.c 中的 `LowPassFilter` 结构与相关函数，统一由 `zita.c` 的 `apply_lpf()` 处理 |
| 2 | `control.c` 顶部 | `#include "control.h"` 被重复包含两次。 | ⭐ | ✔️ **已移除**其中一行 |
| 3 | `control.c` 全局 `m1~m4` | 原 `printf("M1:%.1f ...")` 打印的全局变量与 `MotorMixer()` 内部局部变量同名，调试输出恒为 0。 | ⭐⭐⭐ | ✔️ **已改为** `motor.c` 暴露 `motor_out_m1 ~ m4`（`float`，钳制后的占空比），由 `Control_Print()` 统一打印 |
| 4 | `zita.c` `mahony_update()` 开头 | `if(ax*ay*az == 0) return;` 过于严格。 | ⭐⭐ | ✔️ **已移除**，改用 `norm = sqrt(ax*ax + ay*ay + az*az); if (norm != 0)` 再做归一化 |
| 5 | `zita.c` 四元数 → 欧拉角 符号 | `pitch = -asin(...), roll = -atan2(...)` 与物理方向是否一致，需结合 IMU 安装方向验证。 | ⭐⭐⭐ | ⚠️ 需在实际硬件上验证 |
| 6 | `motor.c` MotorMixer 注释 | 拼音 / 拼写错误（`youhou`, `youqian` 等）。 | ⭐ | ✔️ **已重写**为 "右前/左前/左后/右后 (CW/CCW)" 规范描述 |
| 7 | `control.c` throttle 自动递增 | 上电后油门从 0 递增至 600，无停机开关，飞行器意外启动会非常危险。 | ⭐⭐⭐ | ✔️ **新增 `armed` 油门安全锁**：`Control_Arm(0/1)` 切换，默认未解锁，`main_cm7_0.c` 里注释有解锁示例 |
| 8 | `control.c` 每 10 ms 调用 `printf` | UART0 阻塞发送严重占用 CPU，拖慢 10 ms 节拍。 | ⭐⭐⭐ | ✔️ **降频至 10 Hz**：`debug_counter` 计数，每 10 次循环调用 1 次 `Control_Print()` |
| 9 | `Beacon.h` / `image.h` 宏拼写 | `y_lentgh`, `x_lentgh` 应为 `length`。 | ⭐ | ⚠️ 未修改（当前未启用） |
| 10 | `Beacon.h` 中 `Celing_Boundary_*` | 应为 `Ceiling`。 | ⭐ | ⚠️ 未修改（当前未启用） |
| 11 | `main_cm7_0.c` 与 `main_cm7_1.c` | `debug_info_init()` 与 `debug_init()` 函数名不一致。 | ⭐ | ✔️ `main_cm7_0.c` 统一调用 `debug_init()` |
| 12 | 当前 PID 增益 | 角度外环全 0，Pitch/Yaw 内环全 0，Roll 内环 P=0.2、I=0.01 —— 尚未在实际机架调参。 | ⭐⭐⭐ | ⚠️ 需在真实飞行器上调参 |
| 13 | 无看门狗 / 故障安全保护 | IMU / PWM 连续失败时无降级处理。 | ⭐⭐⭐ | ⚠️ 建议下一步增加 WDT / 心跳检测 |
| 14 | `zita.c` 全局变量暴露 | `q0~q3`, `exInt/eyInt/ezInt` 等全局暴露。 | ⭐⭐ | ⚠️ 可封装为 `static` 结构体 |
| 15 | `gyro_filt[0..2]` 轴序 | `[roll,pitch,yaw]` 与 `zita.c` 内 `acc_filt[x,y,z]` 必须一致；若 IMU 安装方向不同会出现控制方向错误。 | ⭐⭐⭐ | ⚠️ 需与 IMU 安装方向一起确认 |
| 16 | `motor.c` 无下限保护（旧代码） | 旧版 `map_to_pwm_duty()` 只检查 > 1000，若 PID 输出负值会导致 `uint16_t` 溢出（表现为非常大的占空比）。 | ⭐⭐ | ✔️ **已重写为** `clamp_pwm_duty(float)`，同时对 `value < 0` 与 `value > 1000` 钳制 |

### 7.2 调参与扩展建议

| 目标 | 操作 |
|------|------|
| 调 PID 参数 | 修改 `All_pid_init()` 中的增益；建议从 Roll 角速度内环 P 项开始逐步增大 |
| 调整 Mahony 融合 | 修改 `zita.c` 顶部 `MAHONY_KP / MAHONY_KI` 宏 |
| 调整控制频率 | 修改 `pit_ms_init(PIT_NUM, 10)` 的第 2 个参数（单位 ms） |
| **接入 SBUS 遥控（已集成）** | `main_cm7_0.c` 里调用 `uart_receiver_init()`；在主循环把解析后的 6 个通道映射为 `Control_Setpoint(des_roll, des_pitch, des_yaw, throttle)` |
| 扩展定高 | 利用 TOF 激光测距 (`dl1b_distance_mm`) 作为高度反馈，新增高度环 PID |
| 扩展定点/速度反馈 | 结合 PMW3901 `pmw3901_delta_x/y` 与陀螺仪解算水平速度，新增 X/Y 位置环 |
| 扩展视觉 | 把 `Beacon.c` / `image.c` 放入 CM7_1 内核，通过 `zf_driver_ipc` 回传识别结果 |
| 降低 CPU 占用 | 当前调试打印已从 100 Hz 降至 10 Hz；如需进一步优化，可在 `Control_Print()` 改为环形缓冲 + DMA 发送 |

---

## 8. 本次新增：三个外设模块集成（核心改动清单）

> 所有改动均 **在现有工程目录内**完成，未新增目录结构。核心思路是
> **把外设模块的初始化与采样不泄漏到主循环中**。目前 `main_cm7_0.c`
> 只负责"什么时候调用 `sensor_pack_init() / `Control_Loop()`。

### 8.0 模块调用对比表

| 模块 | 驱动 | 硬件接口 | 初始化入口 | 主循环读取 | 调试输出 |
|------|------|----------|-----------|-----------|---------|
| **激光测距** | `zf_device_dl1b.h/c` | 软件 I²C（`DL1B_SCL_PIN=P26_6`, `DL1B_SDA_PIN=P26_7`）| `sensor_pack_init()` 内调用 | `sensor_pack_update()` 内部按 finsh_flag 置位后读 `dl1b_get_distance() | `sensor_pack_print_10hz()` 打印最近距离 mm |
| **光流** | `zf_device_pmw3901.h/c` | 硬件 SPI_0（`P02_0/MISO`, `P02_1/MOSI`, `P02_2/SCK`, `P02_3/CS`）| `sensor_pack_init()` 内调用 | `sensor_pack_update()` 内部每 10 ms 调 `pmw3901_get_motion()` 累计位移 | `sensor_pack_print_10hz()` 打印累计 dx/dy |
| **SBUS 遥控** | `zf_device_uart_receiver.h/c` | UART4（`UART4_RX=P14_0`，100000 bps / 8E2 / 25 字节帧）| `uart_receiver_init()` + `Control_SbusInit()` | SBUS 内部在 `uart4_isr()` 解析，写入 `uart_receiver.channel[0..5]`；主循环里把通道映射到 `Control_Setpoint(des_roll, des_pitch, des_yaw, throttle)` | `Control_Print()` 已打印期望姿态/油门/四路 PWM / armed |

### 8.1 新增函数 API 总览

| 函数 | 所在文件 | 作用 |
|------|-----------|------|
| `sensor_pack_init()` | `sensor_pack.c` | 依次调用 `dl1b_init()` + `pmw3901_init()。| `Control_Arm(enable)` | `control.c` | 设置油门安全锁；`enable=0` 时立即 `throttle=0` + 调用 `Motor_Set(0,0,0,0)` |
| `Control_IsArmed()` | `control.c` | 查询安全锁状态 |
| `Control_SbusInit()` | `control.c` | 标记 `sbus_throttle = -1`（"未接入 SBUS"）；若 `Control_Setpoint` 从未被调用，主循环进入"自动递增油门"模式 |
| `Control_Setpoint(r, p, y, thr_pct)` | `control.c` | 外部指令覆盖期望姿态与油门；`thr_pct` 自动钳制到 `[0, 1000]` |
| `Control_Print()` | `control.c` | 10 Hz 打印最近一次的 roll/pitch/yaw、期望姿态、三轴扭矩、油门、四路 PWM、armed |
| `clamp_pwm_duty(float value)` | `motor.c` | 对占空比做双向钳制，返回 `uint16_t`；保护 PID 负值溢出 |

### 8.2 引脚冲突提醒（已解决）

> **已解决**：`LED1` 原为 `P19_0` 与旧版 `DL1B_SCL_PIN = P19_0` 冲突。
> 当前 `zf_device_dl1b.h` 已改为 `P26_6 / P26_7`，与原理图 TOF 接口一致，与 LED1 不再冲突。

### 8.3 控制数据流的变化

```
新增前（单输入源）：
    IMU → zita.c → attitude → Control_Loop() → PID → MotorMixer → PWM → 电机
              (desired 永远为 (0,0,0))

新增后（多输入源，优先级：SBUS > 自动递增）：
    IMU     → zita.c → attitude → Control_Loop()
                                 │
    SBUS    → uart_receiver_init / uart4_isr
              (在用户代码中手动调用)
              │
              └──→ Control_Setpoint(des_roll, des_pitch, des_yaw, throttle)
              │   写入 sbus_des_* / sbus_throttle
              │
    DL1B    → sensor_pack_init / sensor_pack_update → sensor_pack_tof_mm()
                （当前仅采样，尚未接入控制环；可后续作为高度环反馈）
    PMW3901 → sensor_pack_init / sensor_pack_update → sensor_pack_flow_x/y()
                （当前仅采样，尚未接入控制环；可后续用于水平速度/位置环）
                                 │
                                 ▼
                         Control_Loop() → PID → MotorMixer → clamp_pwm_duty → PWM → 电机
                                 │
                                 └── Control_Print()（10 Hz）→ UART0 输出
```

> **调用方（main_cm7_0.c）** 现在 **只调用 3 个函数：`imu660ra() / sensor_pack_update() / Control_Loop()。不再直接调用 dl1b_* / pmw3901_*，避免"全局变量泄露到主循环。`

---

## 9. 版本与更新日志

详见 `libraries/doc/version.txt`。关键版本摘要：

| 版本 | 变更内容 |
|------|---------|
| **V3.9.3** | 修改按键读取异常问题（频率过低导致读取错误） |
| V3.9.2 | 新增按键读取接口、DMA 相关 API |
| V3.9.1 | 修复 ADC / Flash 读取异常 |
| **V3.9.0** | 新增 PMW3901 光流模块 |
| V3.8.0 | 新增 IMU660RC；优化 SPI 16 位传输 |
| V3.7.x | 优化中断逻辑；新增 SBUS；优化 SPI 时序 |
| V3.6.x | 新增 IPS200PRO；修复 ADC 并发读取 |
| V3.5.x | 修复 DCache 与数据一致性；优化图像分辨率 |
| V3.4.x | 新增 MT9V034 摄像头；修复 M7_1 链接脚本 |

---

## 10. 快速索引（Jump to）

| 想要做什么 | 查看哪里 |
|-----------|---------|
| 阅读飞控主入口 | `project/user/main_cm7_0.c` |
| 修改 PID 参数 | `project/iar/project_config/Debug_m7_0/Exe/pid.c` 的 `All_pid_init()` |
| 调整电机 PWM 引脚 / 钳制逻辑 | `project/iar/project_config/Debug_m7_0/Exe/motor.c` |
| 修改 IMU 解算参数 | `project/iar/project_config/Debug_m7_0/Exe/zita.c` 顶部宏定义 |
| 调整控制循环 / 油门策略 / 安全锁 | `project/iar/project_config/Debug_m7_0/Exe/control.c` 的 `Control_Loop()` |
| 接入 SBUS 遥控 | `libraries/zf_device/zf_device_uart_receiver.h`（UART4，100000/8E2）|
| 使用 TOF 测距 | `libraries/zf_device/zf_device_dl1b.h`（软件 I²C，SCL=P19_0, SDA=P19_1） |
| 使用光流 | `libraries/zf_device/zf_device_pmw3901.h`（硬件 SPI_0，P02_0..3） |
| 使用显示屏 | `libraries/zf_device/zf_device_ips114.h` / `oled.h` / `tft180.h` |
| 使用摄像头 | `libraries/zf_device/zf_device_mt9v03x.h` |
| 需要一套 SPI / UART API | `libraries/zf_driver/zf_driver_spi.h` / `zf_driver_uart.h` |
| 查看引脚可用 GPIO 列表 | `libraries/sdk/tviibh4m/hdr/gpio_cyt4bb_144_teqfp.h` |
| 修改链接脚本 / 内存分配 | `project/iar/icf/linker_directives_tviibh4m.icf` |

---

*本文档由对仓库源码的静态分析生成。若您需要针对特定模块更细化的函数文档、时序图、
仿真 / 调试配置说明，或希望按上述改进建议对代码做修改，**请在修改前与我确认
具体需求**。*

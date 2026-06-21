/*********************************************************************************************************************
* CYT4BB Opensourec Library（简称 CYT4BB 开源库）是一个基于官方 SDK 接口的二次封装源代码
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT4BB 开源库的一部分
*
* 本工程采用"模块化 + 分层驱动 + 应用控制循环"三层结构：
*
*     ┌────────────────────────────────── 应用层 ──────────────────────────────────┐
*     │  main_cm7_0.c :  只负责"调用顺序 + 10 ms 节拍"，不关心任何外设细节       │
*     └────────────────────────────────────────────────────────────────────────────┘
*                              │                │                │
*                              ▼                ▼                ▼
*                     ┌─────────────┐  ┌─────────────┐  ┌─────────────┐
*                     │  sensor_pack │  │  control    │  │  (上位机/   │
*                     │  .h/.c       │  │  .h/.c      │  │   调试协议) │
*                     │  TOF+光流     │  │  PID+混控   │  │             │
*                     │  采样、暴露数 │  │  +SBUS映射   │  │             │
*                     └─────────────┘  └─────────────┘  └─────────────┘
*                              │                │                │
*                              ▼                ▼                ▼
*     ┌──────────────────────────── 驱动层（zf_device / zf_driver）───────────────┐
*     │  imu660ra / dl1b / pmw3901 / uart_receiver / pwm / gpio / uart …         │
*     └────────────────────────────────────────────────────────────────────────────┘
*
* 调用顺序：
*     1) clock_init() / debug_init()                (系统层)
*     2) imu660ra_init() + CalibrateGyro()           (姿态传感器)
*     3) sensor_pack_init()                           (TOF + 光流)
*     4) uart_receiver_init() + Control_SbusInit()   (SBUS 遥控)
*     5) Control_Init()                               (PID + 电机)
*     6) while(1) {
*            if (pit_state) {
*                pit_state = 0;
*                imu660ra();
*                sensor_pack_update();       // ← 取代 "dl1b_get_distance() + pmw3901_get_motion()"
*                // 如果需要接入 SBUS，在这里读取 uart_receiver.channel[]
*                //   并调用 Control_Setpoint(des_roll, des_pitch, des_yaw, throttle);
*                Control_Loop();               // ← PID + MotorMixer + 10 Hz 调试
*                gpio_toggle_level(LED1);
*            }
*        }
*********************************************************************************************************************/

#include "zf_common_headfile.h"
#include "sensor_pack.h"      // TOF + 光流（对外: sensor_pack_init/update/tof_mm/flow_x/y）
#include "control.h"          // 控制（对外: Control_Init/Loop/SbusInit/Setpoint/Arm/Print）
#include "motor.h"            // 电机 PWM（对外: Motor_Init/Set/Mixer）

// ========================= 用户可调参数 =========================
#define PIT_NUM     (PIT_CH0)    // 周期中断通道（10 ms 节拍）
#define PIT_PERIOD  (10)         // 周期（ms）
#define LED1        (P19_0)      // 心跳 LED（TOF 已映射到 P06_6/P06_7，与 LED1 无冲突）
// ===============================================================

volatile uint8_t pit_state = 0;

int main(void)
{
    // ---------- 系统层初始化（顺序不建议修改） ----------
    clock_init(SYSTEM_CLOCK_250M);    // Cortex-M7 主频 250 MHz
    debug_init();                      // UART0 printf 重定向

    gpio_init(LED1, GPO, GPIO_LOW, GPO_PUSH_PULL);

    pit_ms_init(PIT_NUM, PIT_PERIOD);  // 100 Hz 节拍：驱动 IMU / 传感器 / 控制循环

    // ---------- IMU 660RA：姿态解算 ----------
    imu660ra_init();                   // IMU SPI + 内部寄存器配置
    system_delay_ms(100);              // 等待 MEMS 起振稳定
    CalibrateGyro();                   // 陀螺仪零偏采样（约 1 s）

    // ---------- 模块 1：外部传感器（TOF + PMW3901） ----------
    //   内部：dl1b_init() + pmw3901_init()，失败会以返回值标记
    (void)sensor_pack_init();

    // ---------- 模块 2：SBUS 遥控接收（UART4，100000 bps） ----------
    //   硬件：UART4_RX = P14_0，由 zf_device_uart_receiver 内部配置
    //   软件：Control_SbusInit() 仅清空期望姿态与油门，Control_Setpoint() 由外部写入
    //uart_receiver_init();
   // Control_SbusInit();

    // ---------- 控制循环初始化（PID + 电机 PWM + armed 安全锁） ----------
    Control_Init();
    Motor_Set(0, 0, 0, 0);
    system_delay_ms(550);

    // ========================================================
    //  ⚠️  安全提示：上电后 armed = 0，throttle 自动截断为 0。
    //       调试时请把飞行器固定在测试架上后再解锁：Control_Arm(1)。
    //       若已接入 SBUS，建议把"CH4（两档开关）"作为锁机通道：
    //           if (uart_receiver.channel[4] > 1500) Control_Arm(1);
    //           else                                    Control_Arm(0);
    // ========================================================
    // Control_Arm(1);   // TODO: 仅在调试架上手动解锁

    while (1)
    {
        if (pit_state)
        {
            pit_state = 0;               // 先清零，再执行（避免漏拍）

            // 1) IMU：采样 → 去零偏 → LPF → Mahony → Euler
            imu660ra();

            // 2) 外部传感器：TOF + 光流（内部判断 finsh_flag / 最小周期）
            sensor_pack_update();

            // 3) 控制循环（双环 PID + MotorMixer + 10 Hz 调试打印）
            Control_Loop();

            // 4) 心跳指示
            gpio_toggle_level(LED1);
        }
    }
}

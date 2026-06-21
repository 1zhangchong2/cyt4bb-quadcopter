/*
 * control.h
 *
 *  Created on: 2025/12/09
 *  Author : LST
 *  Brief  : 双环 PID 控制循环 + X 型四旋翼混控 + SBUS 遥控接口
 *           + 高度环 (TOF) + 水平位置环 (光流)
 *
 *  顶层数据流向（全部在 control.c 内部完成，main.c 只调用 API）：
 *     TOF(SPI/I²C)→ sensor_pack → height_mm → 高度 PID → 油门修正 ±Δthrottle
 *     光流        → sensor_pack → pos_x/y     → 位置 PID → 期望 pitch/roll
 *     SBUS(UART4) → desired_roll / pitch / yaw / throttle_base
 *     IMU(SPI_0)  → zita.c       → attitude / gyro_filt
 *     外环 PID(位置式) → 目标角速度
 *     内环 PID(增量式) → 三路扭矩
 *     MotorMixer          → 四路 PWM 占空比
 *
 *  约定：高度单位 mm；水平位置单位 "光流积分计数"（未做像素→米标定）。
 *      若测距/光流未就绪（例如 DL1B 初始化失败），高度环 & 位置环自动旁路。
 */

#ifndef CODE_CONTROL_H_
#define CODE_CONTROL_H_

#include "zf_common_headfile.h"
#include "zita.h"          // attitude_t, gyro_filt
#include "pid.h"           // PID 结构体与接口

// ===================== 安全锁 / 油门策略 =====================
//   - Control_Arm(0): 立即将 throttle 清零，MotorMixer 输出 0
//   - Control_Arm(1): 油门从 hover_throttle 按高度环修正量变化
//   - 通过 SBUS 通道或上位机指令切换
#define MOTOR_THROTTLE_MAX     (600.0f)   // 绝对油门上限（保护）
#define MOTOR_THROTTLE_STEP    (2.0f)     // 无 SBUS 时 10 ms 递增步进

// ===================== 悬停油门（可调） =====================
//   当高度环/位置环生效时，以该值为 "基础油门"，
//   pid_height 输出作为修正量叠加；高度环失能时退化为自动递增。
#define MOTOR_HOVER_THROTTLE   (300.0f)   // 典型 250 g 四旋翼在 8.4 V 下的悬停点（仅参考）

// ===================== SBUS 通道映射（可在 main.c 中调整） =====================
//   SBUS 每个通道范围 172 ~ 1811（中间约 992）
//   这里给出一个典型的 6 通道遥控映射定义：
//     CH0 → roll    (左右摇杆 → 期望滚转角 ±15°)
//     CH1 → pitch   (前后摇杆 → 期望俯仰角 ±15°)
//     CH2 → throttle(升降摇杆 → 期望油门 0 ~ 600)
//     CH3 → yaw     (旋转摇杆 → 期望偏航角速度 ±60°/s)
//     CH4 → armed   (两档开关 → 安全锁)
//     CH5 → mode    (预留，目前未使用)
//
//   注意：control 模块本身不关心"通道号"，只关心外部传入的
//   (desired, throttle)。具体哪个通道对应哪个量由调用方（main.c）决定，
//   这样可以轻松切换不同遥控器/接收机的布局。

// ===================== 对外 API =====================
// 飞控主初始化（由 main_cm7_0.c 的初始化序列调用一次）
void Control_Init(void);

// 每 10 ms 节拍调用一次的控制循环
//   内部：高度环 → 位置环 → 角度外环 → 角速度内环 → MotorMixer → 10 Hz 调试
void Control_Loop(void);

// 油门安全锁
void    Control_Arm     (uint8_t enable);
uint8_t Control_IsArmed (void);

// SBUS 初始化（软件状态清零；硬件 UART4 由 zf_device_uart_receiver 在外部调用）
void    Control_SbusInit(void);

// 由外部传入"期望姿态 + 期望油门"（通常由 SBUS 解析或上位机指令产生）
//   参数约定：roll / pitch / yaw 单位为 "°"，throttle 单位为 PWM 占空比 (0 ~ 1000)
void    Control_Setpoint(float des_roll, float des_pitch, float des_yaw, float throttle_pct);

// 高度环/位置环的目标设定（也可由外部上位机/遥控辅助通道传入）
//   des_height_mm: 目标飞行高度（mm）；若传入 0 表示"保持当前高度"
//   des_pos_x/y: 光流累计位移目标（"光流计数"单位）；若传 0 表示原地悬停
void    Control_SetHeight   (int32_t des_height_mm);
void    Control_SetPos      (int32_t des_pos_x,   int32_t des_pos_y);

// 调试辅助：把最近一次控制量通过 UART0 打印一行（10 Hz）
void    Control_Print(void);

#endif /* CODE_CONTROL_H_ */

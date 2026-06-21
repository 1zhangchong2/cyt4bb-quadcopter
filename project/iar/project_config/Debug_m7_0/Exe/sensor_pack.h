/*
 * sensor_pack.h
 *
 *  Created on: 2026/06/20
 *  Author : LST
 *  Brief  : 外设模块聚合层（TOF 测距 + PMW3901 光流）
 *
 *  设计原则：
 *    - 不把底层驱动细节暴露给 main.c / control.c
 *    - 对外只保留 init / update / get_xxx 三类接口
 *    - init()  在主函数初始化序列里调用一次
 *    - update() 在 10 ms 节拍循环里调用一次
 *
 *  接线（与 CYT4BB 开源库原理图一致）：
 *    TOF (DL1B)  : 软件 I²C，SCL=P06_6，SDA=P06_7，XS=P07_2
 *    PMW3901 光流: 硬件 SPI_0，SCLK=P02_2，MOSI=P02_1，MISO=P02_0，NCS=P02_3
 *
 *  注：SBUS 遥控接收由 control 模块内部处理（control.{h,c}）。
 */

#ifndef CODE_SENSOR_PACK_H_
#define CODE_SENSOR_PACK_H_

#include "zf_common_headfile.h"

// ========== 对外数据接口 ==========
// TOF 最近一次测距结果（mm）；若初始化失败则始终返回 0
uint16 sensor_pack_tof_mm    (void);

// PMW3901 累计位移（芯片每次 burst 读出的 delta 值累加）
int32  sensor_pack_flow_x   (void);
int32  sensor_pack_flow_y   (void);

// 模块就绪标志（1 = OK；0 = 初始化失败或尚未初始化）
uint8  sensor_pack_tof_ready (void);
uint8  sensor_pack_flow_ready(void);

// ========== 对外函数接口 ==========
// 初始化：依次调用 dl1b_init() / pmw3901_init()
uint8  sensor_pack_init       (void);

// 每 10 ms 节拍调用一次：
//   - 若 TOF 有新数据（dl1b_finsh_flag），读取最新距离
//   - 读取 PMW3901 的运动 burst，更新累计位移
void   sensor_pack_update    (void);

// 调试辅助：把传感器最新数据通过 UART0 打印一行
// （调用方自行控制频率，例如 10 Hz；不建议在控制主节拍里调用）
void   sensor_pack_print_10hz(void);

#endif /* CODE_SENSOR_PACK_H_ */

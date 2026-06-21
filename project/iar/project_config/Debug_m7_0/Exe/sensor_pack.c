/*
 * sensor_pack.c
 *
 *  将 DL1B (VL53L1X) 激光测距 与 PMW3901 光流 两个模块
 *  的初始化与采样打包成一个接口，供 control.c / main_cm7_0.c 调用。
 *
 *  设计说明：
 *    - DL1B  驱动在 zf_device_dl1b.c/.h 中实现（软件 I²C）
 *    - PMW3901 驱动在 zf_device_pmw3901.c/.h 中实现（硬件 SPI_0）
 *    - 本文件只负责"何时调用"以及"对外暴露什么数据"，
 *      不修改任何驱动内部行为。
 */

#include "zf_common_headfile.h"
#include "sensor_pack.h"

/*
 *  显式声明对 zf_device 的依赖：便于阅读与 IAR 搜索符号。
 *  若编译报 "未定义"，请先确认 zf_device 组是否已加入工程：
 *      zf_device_dl1b.c / zf_device_pmw3901.c
 */
extern uint8_t  dl1b_init         (void);
extern void     dl1b_get_distance (void);
extern uint16_t dl1b_distance_mm;
extern uint8_t  dl1b_finsh_flag;       // 驱动库作者拼写，保持原样

extern uint8_t  pmw3901_init         (void);
extern void     pmw3901_get_motion   (void);
extern int32_t  pmw3901_delta_x_i, pmw3901_delta_y_i;

// 模块初始化结果（内部使用，对外通过 sensor_pack_*_ready() 暴露）
static uint8_t g_tof_ok  = 0;
static uint8_t g_flow_ok = 0;

// 调试打印频率控制（建议外部在 10 Hz 调用即可，内部无需再次分频）
static uint16 g_print_counter = 0;

// ---------------------------------------------------------------------
// 初始化两个外部传感器
//   返回 0: 全部 OK；非零: 至少一个失败（通常为接线/电源问题）
// ---------------------------------------------------------------------
uint8 sensor_pack_init(void)
{
    uint8 fail = 0;

    // 1. DL1B (VL53L1X) 激光测距 - 软件 I²C
    //    引脚：SCL = (P19_0) , SDA = (P19_1), XS = P07_2 
    if (dl1b_init() != 0)
    {
        printf("[SENSOR] TOF init FAIL (check I2C wiring)\r\n");
        g_tof_ok = 0;
        fail = 1;
    }
    else
    {
        g_tof_ok = 1;
        printf("[SENSOR] TOF init OK\r\n");
    }

    // 2. PMW3901 光流 - 硬件 SPI_0 (P02_0..3)
    if (pmw3901_init() != 0)
    {
        printf("[SENSOR] FLOW init FAIL (check SPI wiring)\r\n");
        g_flow_ok = 0;
        fail = 1;
    }
    else
    {
        g_flow_ok = 1;
        printf("[SENSOR] FLOW init OK (min period 20ms)\r\n");
    }

    return fail;
}

// ---------------------------------------------------------------------
// 每 10 ms 节拍调用一次：更新传感器数据
// ---------------------------------------------------------------------
void sensor_pack_update(void)
{
    // DL1B：只有驱动内部"有新数据"才读取；驱动会自己设置 finsh_flag
    if (g_tof_ok && dl1b_finsh_flag)
    {
        dl1b_get_distance();
        dl1b_finsh_flag = 0;
    }

    // PMW3901：每 10 ms 调用一次，驱动内部会处理超时
    if (g_flow_ok)
    {
        pmw3901_get_motion();
    }
}

// ---------------------------------------------------------------------
// 对外数据接口（只读副本）
// ---------------------------------------------------------------------
uint16 sensor_pack_tof_mm(void)
{
    return g_tof_ok ? dl1b_distance_mm : 0u;
}

int32 sensor_pack_flow_x(void)
{
    return g_flow_ok ? pmw3901_delta_x_i : 0;
}

int32 sensor_pack_flow_y(void)
{
    return g_flow_ok ? pmw3901_delta_y_i : 0;
}

uint8 sensor_pack_tof_ready(void)  { return g_tof_ok; }
uint8 sensor_pack_flow_ready(void) { return g_flow_ok; }

// ---------------------------------------------------------------------
// 调试辅助：把传感器最新数据通过 UART0 打印一行
//   调用频率建议 10 Hz；不建议在 100 Hz 控制主节拍内直接调用。
// ---------------------------------------------------------------------
void sensor_pack_print_10hz(void)
{
    g_print_counter++;
    if (g_print_counter < 10) return;
    g_print_counter = 0;

    printf("[SENSOR] TOF=%5u mm   FlowX=%7ld  FlowY=%7ld\r\n",
           (unsigned)dl1b_distance_mm,
           (long)pmw3901_delta_x_i,
           (long)pmw3901_delta_y_i);
}

/*
 * init.c
 *
 *  Created on: 2025年12月9日
 *      Author: LST
 */

#include "zf_common_headfile.h"
#include "init.h"
// 注意：此函数引用了项目中不存在的符号（PID_Position_Init / mypid_* / Left_pwm 等）
//       已被 #if 0 隔离为参考代码，不参与当前飞控工程的编译。
//       如需启用，请先补充相应的函数/宏/变量定义。
#if 0
void All_init_(void)
{
    //pwm初始化代码
//    pwm_init(Left_pwm, 17000, 1500);// PWM通道初始化频率17KHz占空比初始为 0
//    pwm_init(Right_pwm, 17000, 1500);// PWM通道初始化频率17KHz占空比初始为 0

    //普通gpio初始化代码
    gpio_init (P21_2, GPO, 0, GPO_PUSH_PULL);//普通gpio初始化
    gpio_init (P21_3, GPO, 0, GPO_PUSH_PULL);//普通gpio初始化 正转 0 反转 1

    //编码器初始化代码



    //ips200屏幕初始化代码
    ips200_init (IPS200_TYPE_SPI);

    //位置式PID初始化代码
    PID_Position_Init(&mypid_pos, 1, 0, 0, 7000);

    //增量式PID初始化代码
    PID_Incremental_Init(&mypidL_inc, 0, 0, 0, 7000);//左轮PID
    PID_Incremental_Init(&mypidR_inc, 0, 0, 0, 7000);//右轮PID

    //摄像头初始化代码
    mt9v03x_init();

    //中断初始化代码
    pit_init(PIT_CH0, 10000);
}
#endif
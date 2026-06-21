/*********************************************************************************************************************
* CYT4BB Opensource Library（简称 CYT4BB 开源库）是一个基于官方 SDK 接口的二次封装源代码
* Copyright (c) 2022 SEEKFREE 逐飞科技
*
* 本文件是 CYT4BB 开源库的一部分
*
* CYT4BB 开源库 是免费软件
* 您可以根据自由软件基金会发布的 GPL（GNU General Public License，即 GNU通用公共许可证）的条款
* 即 GPL 的第3版（即 GPL3.0）或（您选择的）任何后来的版本，重新发布和/或修改它
*
* 本开源库的发布是希望它能发挥作用，但并未对其作任何的保证
* 甚至没有任何的适销性或针对特定用途的默示保证
* 详细信息请参阅 GPL
*
* 您应该在收到本开源库的同时收到一份 GPL 的副本
* 如果没有，请浏览 <https://www.gnu.org/licenses/>
*
* 特别声明：
* 本开源库使用 GPL3.0 开源许可证协议。以上叙述如与最新版本冲突，请以最新版本为准
* 本条款中英文版如有冲突请以 libraries/doc 文件夹下的 GPL3_permission_statement.txt 文件为准
* 许可证文件请在 libraries 文件夹下查看，或查看对应文件夹下的 LICENSE 文件
* 欢迎各位使用并传播本程序，修改或传播时请保留逐飞科技的版权声明并注明来源
*
* 文件名称          cm7_0_isr
* 公司名称          成都逐飞科技有限公司
* 版本信息          详见 libraries/doc 文件夹下 version 文件版本说明
* 编译环境          IAR 9.40.1
* 适用平台          CYT4BB
* 店铺链接          https://seekfree.taobao.com/
*
* 修改记录
* 日期              作者                备注
* 2024-1-9      pudding            first version
* 2024-5-14     pudding            增加12路pit中断， 增加外部中断说明
* 2025-2-4      pudding            优化外部中断处理逻辑，防止错误地引脚中断,优化外部输入初始化逻辑
* 2025-2-4      pudding            增加外部中断接口
********************************************************************************************************************/

#include "zf_common_headfile.h"
// 注意：必须使用 volatile，否则主循环对 pit_state 的读可能被编译器优化为只读一次。
extern volatile uint8_t pit_state;
// **************************** PIT 中断函数 ****************************
void pit0_ch0_isr()                     // 定时器通道 0 的中断处理函数      
{
     pit_state = 1;                   // 通知主循环
     pit_isr_flag_clear(PIT_CH0);   // 清除中断标志
    
}

void pit0_ch1_isr()                     // 定时器通道 1 的中断处理函数      
{
    pit_isr_flag_clear(PIT_CH1);
    
}

void pit0_ch2_isr()                     // 定时器通道 2 的中断处理函数      
{
    pit_isr_flag_clear(PIT_CH2);
    
}

void pit0_ch10_isr()                    // 定时器通道 10 的中断处理函数      
{
    pit_isr_flag_clear(PIT_CH10);
    
}

void pit0_ch11_isr()                    // 定时器通道 11 的中断处理函数      
{
    pit_isr_flag_clear(PIT_CH11);
    
}

void pit0_ch12_isr()                    // 定时器通道 12 的中断处理函数      
{
    pit_isr_flag_clear(PIT_CH12);
    
}

void pit0_ch13_isr()                    // 定时器通道 13 的中断处理函数      
{
    pit_isr_flag_clear(PIT_CH13);
    
}

void pit0_ch14_isr()                    // 定时器通道 14 的中断处理函数      
{
    pit_isr_flag_clear(PIT_CH14);
    
}

void pit0_ch15_isr()                    // 定时器通道 15 的中断处理函数      
{
    pit_isr_flag_clear(PIT_CH15);
    
}

void pit0_ch16_isr()                    // 定时器通道 16 的中断处理函数      
{
    pit_isr_flag_clear(PIT_CH16);
    
}

void pit0_ch17_isr()                    // 定时器通道 17 的中断处理函数      
{
    pit_isr_flag_clear(PIT_CH17);
    
}

void pit0_ch18_isr()                    // 定时器通道 18 的中断处理函数      
{
    pit_isr_flag_clear(PIT_CH18);
    
}

void pit0_ch19_isr()                    // 定时器通道 19 的中断处理函数      
{
    pit_isr_flag_clear(PIT_CH19);
    
}

void pit0_ch20_isr()                    // 定时器通道 20 的中断处理函数      
{
    pit_isr_flag_clear(PIT_CH20);
    
}

void pit0_ch21_isr()                    // 定时器通道 21 的中断处理函数      
{
    pit_isr_flag_clear(PIT_CH21);
    tsl1401_collect_pit_handler();
}
// **************************** PIT 中断函数 ****************************


// **************************** 串口中断函数 ****************************
// 串口0默认为调试串口
void uart0_isr (void)
{
    if(uart_isr_mask(UART_0))            // 串口0发送中断
    {
        
#if DEBUG_UART_USE_INTERRUPT             // 如果开启 debug 中断处理（debug中断处理函数
        debug_interrupr_handler();       // 调用 debug 串口接收函数，数据将会被 debug 串口库函数获取
#endif                                   // 如果修改了 DEBUG_UART_INDEX 则需要将下面内容放到对应的中断去
      
    }
    else                                 // 串口0接收中断
    {           
        
        
        
    }
}

void uart1_isr (void)
{
    if(uart_isr_mask(UART_1))            // 串口1发送中断
    {
        
        wireless_module_uart_handler();  // 无线模块统一的回调函数
      
    }
    else                                // 串口1接收中断
    {
      
        
        
    }
}

void uart2_isr (void)
{
    if(uart_isr_mask(UART_2))            // 串口2发送中断
    {
        
        gnss_uart_callback();            // GPS模块的回调函数      
        
    }
    else                                // 串口2接收中断
    {
        
        
       
    }
}

void uart3_isr (void)
{
    if(uart_isr_mask(UART_3))            // 串口3发送中断
    {
        
        
        
    }
    else                                // 串口3接收中断
    {
      
        
        
    }
}

void uart4_isr (void)
{
    if(uart_isr_mask(UART_4))            // 串口4发送中断
    {

        uart_receiver_handler();                                                                // 串口接收回调函数
       
    }
    else                                // 串口4接收中断
    {
      
        
        
    }
}

void uart5_isr (void)
{
    if(uart_isr_mask(UART_5))            // 串口5发送中断
    {
        
        
       
    }
    else                                // 串口5接收中断
    {
      
        
        
    }
}

void uart6_isr (void)
{
    if(uart_isr_mask(UART_6))            // 串口6发送中断
    {

        
       
    }
    else                                // 串口6接收中断
    {
      
        
        
    }
}
// **************************** 串口中断函数 ****************************

// **************************** 外部中断函数 ****************************
void gpio_0_exti_isr()                  // 外部 GPIO_0 中断处理函数     
{
    
  
  
}

void gpio_1_exti_isr()                  // 外部 GPIO_1 中断处理函数     
{
    if(exti_flag_get(P01_0))		// 示例P1_0引脚外部中断判断
    {

      
      
            
    }
    if(exti_flag_get(P01_1))
    {

            
            
    }
}

void gpio_2_exti_isr()                  // 外部 GPIO_2 中断处理函数     
{
    if(exti_flag_get(P02_0))
    {
            
            
            
    }
    if(exti_flag_get(P02_4))
    {
            
            
            
    }

}

void gpio_3_exti_isr()                  // 外部 GPIO_3 中断处理函数     
{



}

void gpio_4_exti_isr()                  // 外部 GPIO_4 中断处理函数     
{



}

void gpio_5_exti_isr()                  // 外部 GPIO_5 中断处理函数     
{



}


void gpio_6_exti_isr()                  // 外部 GPIO_6 中断处理函数     
{



}

void gpio_7_exti_isr()                  // 外部 GPIO_7 中断处理函数     
{



}

void gpio_8_exti_isr()                  // 外部 GPIO_8 中断处理函数     
{



}

void gpio_9_exti_isr()                  // 外部 GPIO_9 中断处理函数     
{



}

void gpio_10_exti_isr()                  // 外部 GPIO_10 中断处理函数     
{



}

void gpio_11_exti_isr()                  // 外部 GPIO_11 中断处理函数     
{



}

void gpio_12_exti_isr()                  // 外部 GPIO_12 中断处理函数     
{



}

void gpio_13_exti_isr()                  // 外部 GPIO_13 中断处理函数     
{



}

void gpio_14_exti_isr()                  // 外部 GPIO_14 中断处理函数     
{



}

void gpio_15_exti_isr()                  // 外部 GPIO_15 中断处理函数     
{



}

void gpio_16_exti_isr()                  // 外部 GPIO_16 中断处理函数     
{



}

void gpio_17_exti_isr()                  // 外部 GPIO_17 中断处理函数     
{



}

void gpio_18_exti_isr()                  // 外部 GPIO_18 中断处理函数     
{



}

void gpio_19_exti_isr()                  // 外部 GPIO_19 中断处理函数     
{



}

void gpio_20_exti_isr()                  // 外部 GPIO_20 中断处理函数     
{



}

void gpio_21_exti_isr()                  // 外部 GPIO_21 中断处理函数     
{



}

void gpio_22_exti_isr()                  // 外部 GPIO_22 中断处理函数     
{



}

void gpio_23_exti_isr()                  // 外部 GPIO_23 中断处理函数     
{



}
// **************************** 外部中断函数 ****************************

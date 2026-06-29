#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "zf_common_headfile.h"

// ====== 电机 PWM 通道定义 ======
#define PWM_CH1  (TCPWM_CH30_P10_2)
#define PWM_CH2  (TCPWM_CH57_P17_4)
#define PWM_CH3  (TCPWM_CH11_P05_2)
#define PWM_CH4  (TCPWM_CH20_P08_1)

// MotorMixer 最近一次计算出的四路占空比（钳制前的 float 值，供上层调试使用）
extern float motor_out_m1, motor_out_m2, motor_out_m3, motor_out_m4;

void Motor_Init(void);
void Motor_Set(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4);
void MotorMixer(float throttle, float roll, float pitch, float yaw);
#endif /* _MOTOR_H_ */
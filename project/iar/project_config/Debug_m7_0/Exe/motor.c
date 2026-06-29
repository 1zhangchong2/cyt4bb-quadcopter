#include "zf_common_headfile.h"
#include "motor.h"
/*左前：CCW（逆时针）
右前：CW（顺时针）

左后：CW（顺时针）
右后：CCW（逆时针）
M1 = 左前 CCW
M2 = 右前 CW
M3 = 左后 CW
M4 = 右后 CCW
*/
// 全局输出量（由 MotorMixer 写入，供 control.c 调试打印使用）
// 单位：占空比 0~1000（与 pwm_set_duty 的第二个参数同一量纲）
float motor_out_m1 = 0.0f;
float motor_out_m2 = 0.0f;
float motor_out_m3 = 0.0f;
float motor_out_m4 = 0.0f;

// 把 float 类型的占空比钳制到 [0, 1000] 并转为 uint16_t
// （注意：即使输入为负数也能安全处理，避免从 float 强转 uint16_t 溢出）
static uint16_t clamp_pwm_duty(float value) {
    if (value < 0.0f)    value = 0.0f;
    if (value > 1000.0f) value = 1000.0f;
    return (uint16_t)value;
}

void Motor_Init(void) {
    pwm_init(PWM_CH1, 50, 0);
    pwm_init(PWM_CH2, 50, 0);
    pwm_init(PWM_CH3, 50, 0);
    pwm_init(PWM_CH4, 50, 0);
}

void Motor_Set(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4) {
    pwm_set_duty(PWM_CH1, clamp_pwm_duty((float)m1));
    pwm_set_duty(PWM_CH2, clamp_pwm_duty((float)m2));
    pwm_set_duty(PWM_CH3, clamp_pwm_duty((float)m3));
    pwm_set_duty(PWM_CH4, clamp_pwm_duty((float)m4));
}

void MotorMixer(float throttle, float roll, float pitch, float yaw) {  
    //float raw_m1 = throttle + roll + pitch - yaw; // 左前 CCW  (TCPWM_CH30_P10_2)
   // float raw_m2 = throttle - roll + pitch + yaw; // 右前 CW   (TCPWM_CH57_P17_4)
    //float raw_m3 = throttle + roll - pitch + yaw; // 左后 CW   (TCPWM_CH11_P05_2)
    //float raw_m4 = throttle - roll - pitch - yaw; // 右后 CCW  (TCPWM_CH20_P08_1)
    float raw_m1 = throttle - roll - pitch - yaw;   // 左前 CCW
    float raw_m2 = throttle + roll - pitch + yaw;   // 右前 CW
    float raw_m3 = throttle - roll + pitch + yaw;   // 左后 CW
    float raw_m4 = throttle + roll + pitch - yaw;   // 右后 CCW
    // 先钳制 [0,1000]（PWM 驱动无法接受负值 / 超量程值）
    uint16_t d1 = clamp_pwm_duty(raw_m1);
    uint16_t d2 = clamp_pwm_duty(raw_m2);
    uint16_t d3 = clamp_pwm_duty(raw_m3);
    uint16_t d4 = clamp_pwm_duty(raw_m4);
    // 钳制后的值保存到全局（供上层打印/观测；float 存储以便与 control 类型一致）
    motor_out_m1 = (float)d1;
    motor_out_m2 = (float)d2;
    motor_out_m3 = (float)d3;
    motor_out_m4 = (float)d4;

    Motor_Set(d1, d2, d3, d4);
}
#include "zf_common_headfile.h"
#include "pid.h"

// ====== PID 实例定义 ======
// 角度外环（位置式 PID，输入：角度，输出：目标角速度 °/s）
PID pid_angle_roll, pid_angle_pitch, pid_angle_yaw;
// 角速度内环（增量式 PID，输入：角速度 °/s，输出：扭矩量）
PID pid_rate_roll,  pid_rate_pitch,  pid_rate_yaw;
// 高度环（位置式 PID，输入：高度 mm，输出：油门 ±修正量）
PID pid_height;
// 水平位置环（位置式 PID，输入：光流累计位移 "计数"，输出：期望 pitch / roll）
PID pid_pos_x, pid_pos_y;

// 说明：陀螺仪的低通滤波目前由 zita.c 中的 apply_lpf() 统一处理
// （GYRO_LPF_ALPHA = 0.5f），因此本文件不再保留独立的 gyro_filter_*
// 及配套的 initializeFilter() / filterValue() 死代码，避免重复逻辑。

void PID_Init(PID *pid, float p, float i, float d, float limit) {
    pid->P = p;
    pid->I = i;
    pid->D = d;
    pid->limit = limit;
    pid->Last_Error = 0.0f;
    pid->Previous_Error = 0.0f;
    pid->Integrator = 0.0f;
    pid->output = 0.0f;
}

void All_pid_init(void) {
    // ---------- 角度外环（位置式 PID）----------
    PID_Init(&pid_angle_roll,  0.0f, 0.0f,  0.0f, 300.0f);
    PID_Init(&pid_angle_pitch, 0.0f, 0.0f,  0.0f, 300.0f);
    PID_Init(&pid_angle_yaw,   0.0f, 0.0f,  0.0f, 100.0f);

    // ---------- 角速度内环（增量式 PID）----------
    PID_Init(&pid_rate_roll,   0.1f, 0.05f, 0.0f, 500.0f);
    PID_Init(&pid_rate_pitch,  0.1f, 0.05f,  0.0f, 500.0f);
    PID_Init(&pid_rate_yaw,    0.1f, 0.05f,  0.0f, 300.0f);

    // ---------- 高度环（位置式 PID：TOF → 油门修正）----------
    //   输入单位 mm，输出单位 PWM 占空比偏移量（±limit 范围）
    PID_Init(&pid_height,      0.0f, 0.0f,  0.0f, 200.0f);

    // ---------- 水平位置环（位置式 PID：光流累计位移 → 期望姿态）----------
    //   输入单位 "光流累计计数"，输出单位 °（叠加到期望 pitch/roll）
    PID_Init(&pid_pos_x,       0.00f, 0.0f, 0.0f, 15.0f);
    PID_Init(&pid_pos_y,       0.00f, 0.0f, 0.0f, 15.0f);
}

// 位置式 PID（用于角度外环）
float PID_Realize(PID *pid, float measurement, float setpoint) {
    float error = setpoint - measurement;

    pid->Integrator += pid->I * error;
    // 积分限幅（防止积分饱和）
    if (pid->Integrator >  pid->limit) pid->Integrator =  pid->limit;
    if (pid->Integrator < -pid->limit) pid->Integrator = -pid->limit;

    pid->output = pid->P * error
                + pid->Integrator
                + pid->D * (error - pid->Last_Error);

    pid->Last_Error = error;

    // 输出限幅
    if (pid->output >  pid->limit) pid->output =  pid->limit;
    if (pid->output < -pid->limit) pid->output = -pid->limit;

    return pid->output;
}

// 增量式 PID（用于角速度内环）
float PID_Increase(PID *pid, float measurement, float setpoint) {
    float error = setpoint - measurement;

    pid->output += pid->P * (error - pid->Last_Error)
                 + pid->I * error
                 + pid->D * (error - 2.0f * pid->Last_Error + pid->Previous_Error);

    pid->Previous_Error = pid->Last_Error;
    pid->Last_Error = error;

    // 对累积值本身限幅，防止长时间累积后失去响应
    if (pid->output >  pid->limit) pid->output =  pid->limit;
    if (pid->output < -pid->limit) pid->output = -pid->limit;

    return pid->output;
}
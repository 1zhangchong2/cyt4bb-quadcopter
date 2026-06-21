#ifndef CODE_PID_H_
#define CODE_PID_H_
#include "zf_common_headfile.h"

// ====== PID 结构体 ======
typedef struct {
    float P, I, D, limit;
    float Last_Error;       // 上一次误差
    float Previous_Error;   // 上上次误差（仅增量式使用）
    float Integrator;       // 积分累积值
    float output;           // 控制器输出
} PID;

// ====== 全局 PID 实例声明（在 pid.c 中定义）======
// 角度外环（位置式 PID）：把姿态误差 → 目标角速度
extern PID pid_angle_roll, pid_angle_pitch, pid_angle_yaw;
// 角速度内环（增量式 PID）：把角速度误差 → 电机扭矩
extern PID pid_rate_roll,  pid_rate_pitch,  pid_rate_yaw;
// 高度环（位置式 PID）：TOF 高度误差 → 油门修正量
extern PID pid_height;
// 水平位置环（位置式 PID）：光流累计位移误差 → 期望 pitch / roll
extern PID pid_pos_x, pid_pos_y;

// ====== API ======
void  PID_Init      (PID *pid, float p, float i, float d, float limit);
void  All_pid_init  (void);
float PID_Realize   (PID *pid, float measurement, float setpoint);   // 位置式（外环）
float PID_Increase  (PID *pid, float measurement, float setpoint);   // 增量式（内环）

// 注意：陀螺仪低通滤波当前由 zita.c 的 apply_lpf() 统一处理，
//       pid.c 中不再保留独立的 LowPassFilter 实例与相关函数。

#endif /* CODE_PID_H_ */
/*
 * control.c
 *
 *  Created on: 2025年12月9日
 *  Author : LST
 *
 *  控制分层（从外到内）：
 *    1. 高度环（位置式 PID）：TOF → 油门修正 ±Δthrottle
 *    2. 水平位置环（位置式 PID）：光流累计位移 → 期望 pitch / roll
 *    3. 角度外环（位置式 PID）：姿态误差 → 目标角速度
 *    4. 角速度内环（增量式 PID）：角速度误差 → 扭矩
 *    5. MotorMixer：throttle + torques → 四路 PWM
 *
 *  约定：高度单位 mm；水平位置单位 "光流计数"（未做像素→米标定）。
 *       若 TOF / 光流未就绪，对应环自动旁路（修正量 = 0）。
 */

#include "zf_common_headfile.h"
#include "control.h"
#include "pid.h"
#include "zita.h"            // attitude, gyro_filt
#include "motor.h"
#include "sensor_pack.h"     // sensor_pack_tof_mm / flow_x / flow_y / _ready

// ---------- 外部变量声明 ----------
extern attitude_t attitude;
extern float gyro_filt[3];          // [roll, pitch, yaw]
extern PID pid_angle_roll, pid_angle_pitch, pid_angle_yaw;
extern PID pid_rate_roll,  pid_rate_pitch,  pid_rate_yaw;
extern PID pid_height;
extern PID pid_pos_x,    pid_pos_y;

// ---------- 控制量定义 ----------
static float torque_roll  = 0.0f;
static float torque_pitch = 0.0f;
static float torque_yaw   = 0.0f;
static float throttle     = 600.0f;    // 最终送入混控的油门（含高度环修正）
static float sbus_throttle = -1.0f;   // = -1 表示"未收到 SBUS 输入"
//throttle =MOTOR_HOVER_THROTTLE;
// ---------- SBUS 期望姿态 ----------
static float sbus_des_roll  = 0.0f;
static float sbus_des_pitch = 0.0f;
static float sbus_des_yaw   = 0.0f;

// ---------- 高度环 / 位置环：目标点与"是否有外部设定"标记 ----------
static int32_t  des_height_mm   = 1300;   // 默认 1.3 m；可由 Control_SetHeight 覆盖
static int32_t  des_pos_x       = 0;
static int32_t  des_pos_y       = 0;
static uint8_t  height_locked   = 0;      // 0=未锁，1=已在 Control_Loop 内使用当前高度锁一次
static uint8_t  pos_locked      = 0;

// ---------- 调试缓存：最近一次 PID 输出 ----------
static float last_height_out   = 0.0f;    // pid_height 输出
static float last_pos_x_out    = 0.0f;    // pid_pos_x 输出（→ 期望 pitch）
static float last_pos_y_out    = 0.0f;    // pid_pos_y 输出（→ 期望 roll）

// 油门安全锁（armed）：上电未解锁
static uint8_t armed          = 0;
static uint16_t debug_counter = 0;

// ---------- 对外 API：油门安全锁 ----------
void Control_Arm(uint8_t enable) {
    if (enable == 0) {
        throttle = 0.0f;
        torque_roll = torque_pitch = torque_yaw = 0.0f;
        Motor_Set(0, 0, 0, 0);
    }
    armed = enable ? 1 : 0;
}

uint8_t Control_IsArmed(void) {
    return armed;
}

// ---------- 控制初始化 ----------
void Control_Init(void) {
    All_pid_init();

    torque_roll  = 0.0f;
    torque_pitch = 0.0f;
    torque_yaw   = 0.0f;
    throttle     = 0.0f;
    armed        = 0;
    debug_counter = 0;

    last_height_out = 0.0f;
    last_pos_x_out  = 0.0f;
    last_pos_y_out  = 0.0f;

    // 初始化时"未锁高度/位置"，由第一次有效 TOF/光流数据自动锁定
    height_locked = 0;
    pos_locked    = 0;
    des_height_mm = 1300;
    des_pos_x = 0;
    des_pos_y = 0;

    Motor_Init();
    Motor_Set(0, 0, 0, 0);
    system_delay_ms(550);
}

// ---------- SBUS 初始化 ----------
void Control_SbusInit(void) {
    sbus_des_roll  = 0.0f;
    sbus_des_pitch = 0.0f;
    sbus_des_yaw   = 0.0f;
    sbus_throttle  = -1.0f;
}

// ---------- 外部设定期望姿态 + 期望油门 ----------
void Control_Setpoint(float des_roll, float des_pitch, float des_yaw, float throttle_pct) {
    sbus_des_roll  = des_roll;
    sbus_des_pitch = des_pitch;
    sbus_des_yaw   = des_yaw;

    if      (throttle_pct < 0.0f)    sbus_throttle = 0.0f;
    else if (throttle_pct > 1000.0f) sbus_throttle = 1000.0f;
    else                             sbus_throttle = throttle_pct;
}

// ---------- 外部设定高度目标（mm） ----------
//   若传 0，表示"使用当前高度作为目标"（便于切换到悬停）。
void Control_SetHeight(int32_t des_height_mm_) {
    if (des_height_mm_ <= 0) {
        // 若测距未就绪，则放弃"自动锁定当前高度"
        if (sensor_pack_tof_ready() == 0) return;
        des_height_mm = (int32_t)sensor_pack_tof_mm();
    } else {
        des_height_mm = des_height_mm_;
    }
    height_locked = 1;
}

// ---------- 外部设定位置目标（光流计数单位） ----------
void Control_SetPos(int32_t des_pos_x_, int32_t des_pos_y_) {
    des_pos_x = des_pos_x_;
    des_pos_y = des_pos_y_;
    pos_locked = 1;
}

// ---------- 控制循环（每 10 ms 调用一次） ----------
//  当前模式：固定油门 + 姿态双环（无高度环、无光流位置环、无遥控）
void Control_Loop(void) {
    static float roll_bias = 0, pitch_bias = 0;
    static uint8_t bias_captured = 0;
    if (!bias_captured) {
        roll_bias  = attitude.roll;
        pitch_bias = attitude.pitch;
        bias_captured = 1;
    }
    float roll_comp  = attitude.roll  - roll_bias;
    float pitch_comp = attitude.pitch - pitch_bias;
    // ---- 期望姿态：目标水平 ----
    const float des_roll  = 0.0f;
    const float des_pitch = 0.0f;
    const float des_yaw   = 0.0f;

    // ---- 角度外环（位置式 PID）：姿态误差 → 目标角速度 ----
    //const float target_rate_roll  = PID_Realize(&pid_angle_roll,  roll_comp,  des_roll);
    //const float target_rate_pitch = PID_Realize(&pid_angle_pitch, pitch_comp, des_pitch);
    //const float target_rate_yaw   = PID_Realize(&pid_angle_yaw,   attitude.yaw,   des_yaw);
    static int step_cnt = 0;
    step_cnt++;
    float target_rate_roll = 0;
    if (step_cnt > 200 && step_cnt < 300) 
    {target_rate_roll = 100.0f;   // 100°/s 阶跃
    } else if (step_cnt >= 300) 
    {step_cnt = 0;
    }
    float target_rate_pitch = 0;
    float target_rate_yaw   = 0;
    //float target_rate_roll  = 0;
    // ---- 角速度内环（增量式 PID）：角速度误差 → 扭矩 ----
    torque_roll  = PID_Realize(&pid_rate_roll,  gyro_filt[0], target_rate_roll);
    torque_pitch = PID_Realize(&pid_rate_pitch, gyro_filt[1], target_rate_pitch);
    torque_yaw   = PID_Realize(&pid_rate_yaw,   gyro_filt[2], target_rate_yaw);

    // ---- 油门：直接固定 600（暂不用安全锁，后续接入遥控再恢复 armed） ----
    /*
    if (armed) {
        throttle = 600.0f;
    } else {
        throttle = 0.0f;
    }
    */
    // ---- 混控 → 四路 PWM
    throttle = 600.0f;
    MotorMixer(throttle, torque_roll, torque_pitch, torque_yaw);
    printf("%.1f,%.1f\r\n", gyro_filt[0], target_rate_roll);
    //printf("%.1f\r\n", gyro_filt[0]);
    //printf("R:%.1f P:%.1f Y:%.1f  tR:%.1f tP:%.1f tY:%.1f  thr:%.1f\r\n",
          // attitude.roll, attitude.pitch, attitude.yaw,
            //torque_roll, torque_pitch, torque_yaw, throttle);
}

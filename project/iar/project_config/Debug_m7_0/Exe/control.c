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
static float throttle     = 0.0f;    // 最终送入混控的油门（含高度环修正）
static float sbus_throttle = -1.0f;   // = -1 表示"未收到 SBUS 输入"

// ---------- SBUS 期望姿态 ----------
static float sbus_des_roll  = 0.0f;
static float sbus_des_pitch = 0.0f;
static float sbus_des_yaw   = 0.0f;

// ---------- 高度环 / 位置环：目标点与"是否有外部设定"标记 ----------
static int32_t  des_height_mm   = 1500;   // 默认 1.5 m；可由 Control_SetHeight 覆盖
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

// 打印缓存（最近一次控制量，供 Control_Print 输出）
static float last_attitude[3] = {0.0f, 0.0f, 0.0f};
static float last_torque[3]   = {0.0f, 0.0f, 0.0f};
static float last_pos_roll    = 0.0f;
static float last_pos_pitch   = 0.0f;
static float last_pos_yaw     = 0.0f;

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
    des_height_mm = 1500;
    des_pos_x = 0;
    des_pos_y = 0;

    Motor_Init();
    Motor_Set(0, 0, 0, 0);
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
void Control_Loop(void) {
    // ---------------- 高度环（TOF → 油门修正） ----------------
    float height_out = 0.0f;
    if (armed && sensor_pack_tof_ready()) {
        // 第一次进入：自动把当前高度锁定为目标，避免"从 0 猛然爬升到默认 1500mm"
        if (height_locked == 0) {
            des_height_mm = (int32_t)sensor_pack_tof_mm();
            height_locked = 1;
        }
        height_out = PID_Realize(&pid_height,
                                 (float)sensor_pack_tof_mm(),
                                 (float)des_height_mm);
    } else {
        pid_height.Integrator = 0.0f;    // 未就绪时清空积分，避免再次进入时爆冲
        height_out = 0.0f;
    }
    last_height_out = height_out;

    // ---------------- 位置环（光流 → 期望 pitch / roll） ----------------
    float pos_x_out = 0.0f;   // 作为"期望 pitch"的偏移
    float pos_y_out = 0.0f;   // 作为"期望 roll" 的偏移

    if (armed && sensor_pack_flow_ready()) {
        if (pos_locked == 0) {
            des_pos_x = sensor_pack_flow_x();
            des_pos_y = sensor_pack_flow_y();
            pos_locked = 1;
        }
        // 约定：光流 x 方向 → 前后位移 → 对应 pitch；y 方向 → 左右位移 → 对应 roll
        pos_x_out = PID_Realize(&pid_pos_x, (float)sensor_pack_flow_x(), (float)des_pos_x);
        pos_y_out = PID_Realize(&pid_pos_y, (float)sensor_pack_flow_y(), (float)des_pos_y);
    } else {
        pid_pos_x.Integrator = 0.0f;
        pid_pos_y.Integrator = 0.0f;
        pos_x_out = 0.0f;
        pos_y_out = 0.0f;
    }
    last_pos_x_out = pos_x_out;
    last_pos_y_out = pos_y_out;

    // ---------------- 期望姿态（SBUS 位置偏移叠加） ----------------
    //   若 SBUS 已经传入姿态偏移，直接把位置环输出叠加其上；
    //   否则位置环输出作为"唯一的期望姿态"来让机体保持水平。
    const float desired_roll  = sbus_des_roll  + pos_y_out;
    const float desired_pitch = sbus_des_pitch + pos_x_out;
    const float desired_yaw   = sbus_des_yaw;

    // ---------------- 油门：基础 + 高度环修正 ----------------
    if (armed) {
        float base = 0.0f;
        if (sbus_throttle >= 0.0f) {
            base = sbus_throttle;                  // SBUS 已接入 → 直接用
        } else {
            // 无 SBUS：油门由悬停点 + 缓慢递增（作为调试时的基本油门）
            if (throttle < MOTOR_HOVER_THROTTLE) {
                base = throttle + MOTOR_THROTTLE_STEP;
            } else {
                base = MOTOR_HOVER_THROTTLE;
            }
        }
        // 叠加高度环修正，并钳制在 [0, MOTOR_THROTTLE_MAX]
        float thr = base + height_out;
        if (thr < 0.0f)                    thr = 0.0f;
        if (thr > MOTOR_THROTTLE_MAX)      thr = MOTOR_THROTTLE_MAX;
        throttle = thr;
    } else {
        throttle = 0.0f;
    }

    // ---------------- 角度外环（位置式 PID） ----------------
    const float target_rate_roll  = PID_Realize(&pid_angle_roll,  attitude.roll,  desired_roll);
    const float target_rate_pitch = PID_Realize(&pid_angle_pitch, attitude.pitch, desired_pitch);
    const float target_rate_yaw   = PID_Realize(&pid_angle_yaw,   attitude.yaw,   desired_yaw);

    // ---------------- 角速度内环（增量式 PID） ----------------
    torque_roll  = PID_Increase(&pid_rate_roll,  gyro_filt[0], target_rate_roll);
    torque_pitch = PID_Increase(&pid_rate_pitch, gyro_filt[1], target_rate_pitch);
    torque_yaw   = PID_Increase(&pid_rate_yaw,   gyro_filt[2], target_rate_yaw);

    // ---------------- 混控 → 四路 PWM ----------------
    MotorMixer(throttle, torque_roll, torque_pitch, torque_yaw);

    // ---------------- 调试缓存 & 10 Hz 打印 ----------------
    last_attitude[0] = attitude.roll;
    last_attitude[1] = attitude.pitch;
    last_attitude[2] = attitude.yaw;
    last_torque[0]   = torque_roll;
    last_torque[1]   = torque_pitch;
    last_torque[2]   = torque_yaw;
    last_pos_roll    = desired_roll;
    last_pos_pitch   = desired_pitch;
    last_pos_yaw     = desired_yaw;

    debug_counter++;
    if (debug_counter >= 10) {
        debug_counter = 0;
        Control_Print();
    }
}

// ---------- 对外的调试输出（10 Hz 自动调用；也可手动） ----------
void Control_Print(void) {
    printf(
        "[CTL] Armed=%s R:%.1f P:%.1f Y:%.1f | desR:%.1f desP:%.1f desY:%.1f | "
        "TOF:%u mm hgt_out:%.1f | fx:%.1f fy:%.1f | TqR:%.1f TqP:%.1f TqY:%.1f | "
        "M1:%.0f M2:%.0f M3:%.0f M4:%.0f\r\n",
        armed ? "ON" : "OFF",
        last_attitude[0], last_attitude[1], last_attitude[2],
        last_pos_roll, last_pos_pitch, last_pos_yaw,
        (unsigned)sensor_pack_tof_mm(), last_height_out,
        last_pos_x_out, last_pos_y_out,
        last_torque[0], last_torque[1], last_torque[2],
        motor_out_m1, motor_out_m2, motor_out_m3, motor_out_m4);
}

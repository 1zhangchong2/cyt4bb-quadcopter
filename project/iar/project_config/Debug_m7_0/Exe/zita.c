#include "zf_common_headfile.h"
#include "zita.h"
float acc_filt[3] = {0};// 加速度滤波数组
float gyro_filt[3] = {0};
#define ACC_LPF_ALPHA 0.2f
#define GYRO_LPF_ALPHA 0.5f  // 陀螺仪低通滤波系数，范围 0.0~1.0，越小滤波越强
#define MAHONY_KP         5.0f    // 比例系数
#define MAHONY_KI         0.008f   // 积分系数
#define SAMPLE_PERIOD     0.01f    // 采样周期 10ms / 100Hz
#define HALF_T            (SAMPLE_PERIOD / 2.0f)
#define RAD_TO_DEG        57.29577951308232f // 弧度转角度系数
#define DEG_TO_RAD        0.017453292519943295f // 角度转弧度系数
#define GYRO_OFFSET_SAMPLES 500
//====================================================
// 四元数
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
// PI 积分项
float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;
// 陀螺仪零偏  
float gyro_offset[3] = {0};
attitude_t attitude = {0, 0, 0};
int imu660ra(void)
{
        // 读取加速度和角速度原始值
        imu660ra_get_acc();                                                     // 读取 IMU 加速度原始数据
        imu660ra_get_gyro();                                                    // 读取 IMU 角速度原始数据       
        float raw_acc[3] = {imu660ra_acc_x, imu660ra_acc_y, -imu660ra_acc_z}; // 加速度原始值（供低通滤波）
        float raw_gyro[3] = {imu660ra_gyro_x-gyro_offset[0], imu660ra_gyro_y-gyro_offset[1], imu660ra_gyro_z-gyro_offset[2]};// 减去零偏后的角速度
        apply_lpf(raw_acc, acc_filt, ACC_LPF_ALPHA, 3);// 加速度低通滤波
        apply_lpf(raw_gyro, gyro_filt, GYRO_LPF_ALPHA, 3);// 陀螺仪低通滤波
        mahony_update(gyro_filt[0]*DEG_TO_RAD, gyro_filt[1]*DEG_TO_RAD, gyro_filt[2]*DEG_TO_RAD,
                      acc_filt[0], acc_filt[1], acc_filt[2]);
        //printf("%f,%f,%f\n",attitude.pitch,attitude.roll,attitude.yaw);
        return 0;
} 
void CalibrateGyro(void) {
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    for (int i = 0; i < GYRO_OFFSET_SAMPLES; i++) {
        imu660ra_get_gyro();
        sum_x += imu660ra_gyro_x;
        sum_y += imu660ra_gyro_y;
        sum_z += imu660ra_gyro_z;
        system_delay_ms(2);
    }
    gyro_offset[0] = (float)sum_x / GYRO_OFFSET_SAMPLES;
    gyro_offset[1] = (float)sum_y / GYRO_OFFSET_SAMPLES;
    gyro_offset[2] = (float)sum_z / GYRO_OFFSET_SAMPLES;
}
void apply_lpf(float *raw, float *filt, float alpha, int size)
{
    for(int i = 0; i < size; i++) 
    {
        filt[i] = filt[i] * (1.0f - alpha) + raw[i] * alpha;
    }
}

// Mahony 互补滤波姿态更新函数
void mahony_update(float gx, float gy, float gz, float ax, float ay, float az)
{
    float norm;
    float vx, vy, vz;
    float ex, ey, ez;
    // 注：移除了 `if (ax*ay*az == 0) return;` 的错误检查
    // （对 float 精确等于 0 的比较不可靠，且语义应为"三轴全零"）
    // 后续 `norm == 0.0f` 已经正确覆盖了 IMU 未响应的除零保护场景
    // 1. 加速度归一化
    norm = sqrtf(ax*ax + ay*ay + az*az);
    if(norm == 0.0f) return; // 防止除零保护
    ax /= norm;
    ay /= norm;
    az /= norm;

    // 2. 由当前四元数计算参考重力方向向量
    vx = 2.0f * (q1*q3 - q0*q2);
    vy = 2.0f * (q0*q1 + q2*q3);
    vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

    // 3. 向量叉积得到误差项
    ex = ay*vz - az*vy;
    ey = az*vx - ax*vz;
    ez = ax*vy - ay*vx;

    // 4. PI 补偿更新校正后的角速度
    exInt += ex * MAHONY_KI * SAMPLE_PERIOD;
    eyInt += ey * MAHONY_KI * SAMPLE_PERIOD;
    ezInt += ez * MAHONY_KI * SAMPLE_PERIOD;

    gx += MAHONY_KP * ex + exInt;
    gy += MAHONY_KP * ey + eyInt;
    gz += MAHONY_KP * ez + ezInt;

    // 5. 四元数一阶微分方程更新
    q0 += (-q1*gx - q2*gy - q3*gz) * HALF_T;
    q1 += ( q0*gx + q2*gz - q3*gy) * HALF_T;
    q2 += ( q0*gy - q1*gz + q3*gx) * HALF_T;
    q3 += ( q0*gz + q1*gy - q2*gx) * HALF_T;

    // 6. 四元数归一化
    norm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 /= norm;
    q1 /= norm;
    q2 /= norm;
    q3 /= norm;

    // 7. 四元数转欧拉角
    attitude.pitch  = -asinf(-2.0f*q1*q3 + 2.0f*q0*q2) * RAD_TO_DEG;
    attitude.roll   = -atan2f(2.0f*q2*q3 + 2.0f*q0*q1, -2.0f*q1*q1 - 2.0f*q2*q2 + 1.0f) * RAD_TO_DEG;
    attitude.yaw    = atan2f(2.0f*q1*q2 + 2.0f*q0*q3, -2.0f*q2*q2 - 2.0f*q3*q3 + 1.0f) * RAD_TO_DEG;
}



#ifndef CODE_ZITA_H_
#define CODE_ZITA_H_
#include "zf_common_headfile.h"
// 姿态角结构体定义
typedef struct {
    float pitch;
    float roll;
    float yaw;
} attitude_t;

// 定义外部全局变量，在姿态.c中赋值
extern attitude_t attitude;
extern float gyro_filt[3];

int imu660ra(void);
void apply_lpf(float *raw, float *filt, float alpha, int size);
void mahony_update(float gx, float gy, float gz, float ax, float ay, float az);
void CalibrateGyro(void);
#endif /* CODE_ZITA_H_ */
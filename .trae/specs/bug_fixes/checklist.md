# CYT4BB 飞控示例 Bug 修复 - 验证清单（Checklist）

---

## 🔴 Task 1：Mahony 滤波器 zero-check 修复

- [ ] `zita.c` 中不再出现 `ax*ay*az == 0` 或类似的 float 精确等于 0 的比较
- [ ] `zita.c` 中保留了 `if (norm == 0.0f) return;`（line 69 附近）的 division-by-zero 保护
- [ ] `mahony_update()` 函数其余行逻辑完全未变（四元数更新、归一化、欧拉角转换）
- [ ] Clean build 成功，无 errors / warnings
- [ ] `attitude.pitch / roll / yaw` 在运行时持续更新（串口或调试器观察）

---

## 🟡 Task 2：移除 main() 内的 K&R 函数声明

- [ ] `main_cm7_0.c` 中不再有以 `uint8` 或 `extern void` 开头、紧跟 `xxx_init` 或 `xxx(void)` 的函数体内声明
- [ ] `main()` 中保留了对 `imu660ra_init()` 和 `CalibrateGyro()` 的 **调用**（即删除的是声明行，不是调用行）
- [ ] Clean build 成功，无 errors / warnings
- [ ] `main_cm7_0.c` 的 `#include "zf_common_headfile.h"` 仍在文件顶部（确保编译器能解析到函数名）

---

## 🟢 Task 3：移除重复的 `#include "control.h"`

- [ ] `control.c` 中 `#include "control.h"` 仅出现 **一次**
- [ ] 其余 `#include`（`zf_common_headfile.h`、`pid.h`、`zita.h` 等）未被误删
- [ ] Clean build 成功

---

## 🟠 Task 4：移除 `motor.c` 中重复的 PWM_CHx 宏

- [ ] `motor.c` 中没有任何 `#define PWM_CH[1-4]`
- [ ] `motor.h` 中保留了全部 4 个 `#define PWM_CHx (TCPWM_CHxx_Pxx_x)` 的定义
- [ ] `motor.c` 顶部保留了 `#include "motor.h"`（否则会报 undefined macro）
- [ ] `Motor_Set()` 和 `MotorMixer()` 函数的符号（+roll/-roll/+pitch/-pitch/+yaw/-yaw）未被任何修改所影响
- [ ] MotorMixer 的注释已被清理为清晰可读（中文或英文，不含 "youhou"/"youqian" 等残片）
- [ ] Clean build 成功，无 redefinition warning

---

## 🔵 Task 5：清理 `init.c` 中的 orphan 符号引用

- [ ] `init.c` 顶部加有清晰的注释，说明此文件是"参考模板/历史残片，默认不参与飞控主程序 build"
- [ ] `All_init_()` 函数被 `#if 0 ... #endif` 包裹（或者所有引用不存在符号的行被删除——根据选择的方案验证）
- [ ] 文件中已无 `PID_Position_Init`、`PID_Incremental_Init`、`mypid_pos`、`mypidL_inc`、`mypidR_inc`、`Left_pwm`、`Right_pwm` 的 **未被 `#if 0` 包裹的** 引用
- [ ] Clean build 成功（IAR 将 `init.c` 加入 build group 也不会产生任何链接/编译错误）

---

## ✅ 整体回归验证

- [ ] 完整 Clean + Make 成功（`cyt4bb7_cm_7_0` 目标）
- [ ] 无新增 warnings（与修复前 baseline 对比）
- [ ] 生成的 `.out` / `.hex` 文件可被下载
- [ ] **（硬件验证，如有条件）** 下载后观察：LED 心跳频率（100Hz）、串口 attitude 输出持续有效、电机 PWM 占空比响应控制循环一致
- [ ] 所有 5 个 Task 在 `tasks.md` 中均已标记为 `[x]`

---

## ❌ 禁止事项验证（Must NOT have done）

- [ ] 未修改 `libraries/` 目录下任何文件（SDK、zf_common、zf_driver、zf_device、zf_components）
- [ ] 未改变 PID 参数（`All_pid_init` 中的 P/I/D/limit 值）
- [ ] 未改变 `MotorMixer` 的符号逻辑（+/- roll/pitch/yaw）
- [ ] 未改变 `pit_ms_init` 参数、`debug_init` 调用、GPIO 初始化
- [ ] 未引入新的 `.h` / `.c` 文件

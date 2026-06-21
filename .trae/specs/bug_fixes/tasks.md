# CYT4BB 飞控示例 Bug 修复 - 任务分解与优先级

## 总体执行顺序

**建议按以下顺序串行执行**（每个任务由独立的 sub-agent 执行，完成后回归验证再进入下一个）：

```
Task 1 (ISSUE-1) → Task 2 (ISSUE-2) → Task 3 (ISSUE-3) → Task 4 (ISSUE-4) → Task 5 (ISSUE-5)
```

---

## [ ] Task 1：修复 `zita.c` 中 `mahony_update()` 的错误 zero-check

- **Priority**：P0
- **Depends On**：None
- **Description**：
  - 在 `project/iar/project_config/Debug_m7_0/Exe/zita.c` 中，`mahony_update()` 函数的行首检查 `if (ax*ay*az == 0) return;` 有两个缺陷：
    1. **语义错误**：本意是"IMU 未响应（全零）"，但实际检查的是"任意一轴为零"——会导致合法的重力向量被误判
    2. **浮点比较**：`== 0` 对 float 是不安全/不精确的比较
  - 修复方式：**删除整行**。因为后续 `norm = sqrtf(ax*ax + ay*ay + az*az); if (norm == 0.0f) return;` 已经正确地覆盖了"向量全零"的真实失败场景
  - 保持其余 Mahony 算法逻辑不变
- **Acceptance Criteria Addressed**：AC-1, AC-6
- **Test Requirements**：
  - `programmatic` TR-1.1：编译 `zita.c` 无 errors/warnings；对包含 `mahony_update` 的全部代码进行 clean build
  - `human-judgment` TR-1.2：代码审阅者确认 `mahony_update()` 中不再有 `ax*ay*az` 这类 float-product zero-check，且保留了 `norm == 0.0f` 的 division-by-zero 保护
- **Notes**：注意 zita.c 文件中 `ax/ay/az` 是函数形参；修改仅影响 `mahony_update` 的前几行

---

## [ ] Task 2：移除 `main_cm7_0.c` 中 main() 内的 K&R 函数声明

- **Priority**：P1
- **Depends On**：Task 1
- **Description**：
  - 在 `project/user/main_cm7_0.c` 中，`main()` 函数体内（约 line 49-50）存在：
    ```c
    uint8 imu660ra_init();
    extern void CalibrateGyro(void);
    ```
  - 这些是 K&R C 的遗留风格，且是**冗余的**：`imu660ra_init()` 已通过 `zf_device_imu660ra.h` → `zf_common_headfile.h` 的 include chain 声明；`CalibrateGyro()` 也在 `zita.h` 中声明（并被 control.h → zf_common_headfile.h 拉取）
  - 修复方式：直接**删除两行**
- **Acceptance Criteria Addressed**：AC-2, AC-6
- **Test Requirements**：
  - `programmatic` TR-2.1：`grep -n "imu660ra_init()" project/user/main_cm7_0.c` 在 `main()` 函数范围内（即 `int main` 与配对 `}` 之间）不应返回任何结果
  - `programmatic` TR-2.2：clean build 成功
- **Notes**：`main_cm7_0.c` 顶部已经 `#include "zf_common_headfile.h"`，编译器能正确解析 `imu660ra_init()` 与 `CalibrateGyro()`，无需额外声明

---

## [ ] Task 3：移除 `control.c` 中重复的 `#include "control.h"`

- **Priority**：P2
- **Depends On**：Task 2
- **Description**：
  - 在 `project/iar/project_config/Debug_m7_0/Exe/control.c` 中，`#include "control.h"` 出现在 line 9 和 line 10（连续两行完全相同）
  - 修复方式：删除 line 10，保留 line 9 一次
- **Acceptance Criteria Addressed**：AC-3, AC-6
- **Test Requirements**：
  - `programmatic` TR-3.1：对 control.c 执行 "grep -c 'include.*control.h'" 结果必须为 1
  - `programmatic` TR-3.2：clean build 成功
- **Notes**：属于最简单的 cleanup，不影响运行行为

---

## [ ] Task 4：移除 `motor.c` 中重复的 PWM_CHx 宏定义

- **Priority**：P1
- **Depends On**：Task 3
- **Description**：
  - 在 `project/iar/project_config/Debug_m7_0/Exe/motor.c` lines 3-6，存在：
    ```c
    #define PWM_CH1  (TCPWM_CH30_P10_2)
    #define PWM_CH2  (TCPWM_CH57_P17_4)
    #define PWM_CH3  (TCPWM_CH11_P05_2)
    #define PWM_CH4  (TCPWM_CH20_P08_1)
    ```
  - 相同内容已存在于 `project/iar/project_config/Debug_m7_0/Exe/motor.h` lines 6-9
  - `.c` 已 include `motor.h`（line 2），无需自行重复 define
  - **修复**：删除 motor.c 中 lines 3-6
  - **附加清理**（可选）：同时清理 `MotorMixer()` 函数内注释的混乱文本（"youhou"/"youqian" 等），替换为清晰的中文注释（如"右后"、"右前"）。**不改变任何符号逻辑**
- **Acceptance Criteria Addressed**：AC-4, AC-6
- **Test Requirements**：
  - `programmatic` TR-4.1：`grep '#define PWM_CH' project/iar/project_config/Debug_m7_0/Exe/motor.c` 返回 0 条结果；同一 grep 在 `motor.h` 返回 4 条结果
  - `programmatic` TR-4.2：clean build 成功
  - `human-judgment` TR-4.3：审阅者确认 `MotorMixer` 的符号（+roll/-roll etc.）与修改前完全一致
- **Notes**：这是"最容易引入回归的 cleanup"——必须对符号保持零修改，确保 PWM 输出通道映射不变

---

## [ ] Task 5：清理 `init.c` 中对未定义符号的引用

- **Priority**：P1
- **Depends On**：Task 4
- **Description**：
  - `project/iar/project_config/Debug_m7_0/Exe/init.c` 的 `All_init_()` 函数引用了以下项目中**不存在**的符号：
    - `PID_Position_Init()`, `PID_Incremental_Init()`（`pid.c` 中只定义了 `PID_Init()`）
    - `mypid_pos`, `mypidL_inc`, `mypidR_inc`（未在任何 `.h` / `.c` 中声明）
    - `Left_pwm`, `Right_pwm`（未定义）
    - `mt9v03x_init()`（在 `zf_device_mt9v03x.h` 中存在，但此处使用场景与 `main_cm7_0.c` 的飞控逻辑不一致）
  - 当前 `main_cm7_0.c` 未调用 `All_init_()`，所以此问题被"隐藏"。但如果有人把 `init.c` 加入 build group 并尝试调用，会直接 link fail
  - **默认修复方案（方案 A）**：用 `#if 0 ... #endif` 包裹整个 `All_init_()` 函数体，并在文件顶部加一行注释说明"此为参考模板/历史项目残片，不参与当前飞控 build。如需启用，请自行补齐对应符号定义"
- **Acceptance Criteria Addressed**：AC-5, AC-6
- **Test Requirements**：
  - `programmatic` TR-5.1：`init.c` 可成功参与 clean build（即使 `All_init_()` 被 `#if 0` 包裹后，作为空/非空编译单元都无错误；链接时也不会产生对不存在符号的引用）
  - `human-judgment` TR-5.2：审阅者确认文件顶部有清晰注释，说明此文件当前为"示例/禁用"状态
- **Notes**：如果用户偏好方案 B（彻底删除 `init.c`），请在审核阶段告知；默认采用更保守的方案 A

---

## 跨任务通用测试要求

- **每个 Task 完成后都必须**：
  1. 在 IAR 中进行 **Clean + Make**
  2. 确认无 error，无新增 warning
  3. 在 `tasks.md` 中勾选已完成的本 Task

- **全部 Task 完成后**：
  1. 进行一次完整的 Clean + Make
  2. 下载到硬件，验证 LED 心跳、IMU 输出、PWM 输出行为与修改前一致（若有条件）
  3. 在 `checklist.md` 中逐项打勾

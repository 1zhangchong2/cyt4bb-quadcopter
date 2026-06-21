# CYT4BB 飞控示例代码 潜在问题修复 - PRD (Product Requirements Document)

## 概述
CYT4BB 开源库中的 `project/iar/project_config/Debug_m7_0/Exe/` 目录下的飞控示例代码存在多个代码质量问题，包括冗余声明、重复头文件包含、宏重复定义、Mahony 姿态解算中存在缺陷的 float-equal-zero 判断，以及一个引用了大量未定义符号的 orphan 文件 `init.c`。本 PRD 规划对这些问题进行逐个修复，目标是 **在保持飞行控制逻辑不变的前提下** 提升代码的正确性、可维护性和规范性。

## 目标
- **G1（正确性）**：修复会导致特定姿态下 Mahony 滤波器被错误跳过的 bug（`if (ax*ay*az == 0)`）
- **G2（可维护性）**：移除重复/冗余/错误放置的声明和宏定义
- **G3（健壮性）**：清理 `init.c` 中不存在的符号引用（会造成 build 失败的隐患）

## 非目标
- **不修改飞行控制逻辑**：PID 参数、MotorMixer 的符号/布局、IMU 滤波系数等**保持不变**
- **不修改底层库代码**：`libraries/zf_*` / `libraries/sdk/` **不做改动**（除非有明确的 bug 报告）
- **不做性能优化**：不引入新的算法或数据结构
- **不改变 API 签名**：`main()` 调用链不变

## 背景与上下文

项目代码是一个基于 Cypress Traveo-II CYT4BB（双核 Cortex-M7）+ IMU660RA 的四旋翼飞控示例。源码采用以下调用链：

```
main_cm7_0.c
  ├─ clock_init / debug_init / gpio_init(LED1)
  ├─ pit_ms_init(PIT_CH0, 10)        ← 100Hz 节拍
  ├─ imu660ra_init() + CalibrateGyro()
  └─ Control_Init() + Motor_Set(0,0,0,0)
    └─ while(1): if(pit_state) { imu660ra(); Control_Loop(); gpio_toggle(LED1); }
         ├─ imu660ra()  → zita.c: 读IMU→LPF→Mahony→欧拉角
         └─ Control_Loop()→ control.c: 角度环PID_Realize + 角速度环PID_Increase → MotorMixer()
```

中断入口：`cm7_0_isr.c` 中 `pit0_ch0_isr()` 设置 `pit_state = 1` 并清除中断标志。

## 识别出的待修复问题

| ID | 文件 | 位置 | 问题描述 | 严重程度 |
|----|------|------|---------|---------|
| **ISSUE-1** | `zita.c` | `mahony_update()` line 63 | `if (ax*ay*az == 0)` — 对 float 做 exact equality to zero 的判断，且语义错误：**任何一个轴**为 0 就跳过，而本意是判断 IMU 是否失效（三轴均为 0）。会导致合法姿态（flat 水平时某轴接近 0）时错误地跳过 Mahony 更新。 | **高** |
| **ISSUE-2** | `main_cm7_0.c` | `main()` 内 lines 49-50 | `uint8 imu660ra_init();` 和 `extern void CalibrateGyro(void);` 放在 main() 函数体内（K&R C 遗留风格），冗余且不符合规范（两个函数的正确声明已在 `zf_common_headfile.h` 的 include chain 中）。 | **中** |
| **ISSUE-3** | `control.c` | line 10 | `#include "control.h"` 重复包含（line 9 已 include 过一次） | **低** |
| **ISSUE-4** | `motor.c` | lines 3-6 | 重复定义 `PWM_CH1..PWM_CH4` 宏 — 这些宏已在 `motor.h` 中定义，`.c` 中又写了一遍，会导致：(a) 头文件修改后不生效；(b) 潜在的 redefinition warning。 | **中** |
| **ISSUE-5** | `init.c` | 全文 | 孤儿文件：引用了 `PID_Position_Init()`、`PID_Incremental_Init()`、`mypid_pos`、`mypidL_inc`、`mypidR_inc`、`Left_pwm`、`Right_pwm` 等 **项目中不存在的符号**。如果此 `.c` 被加入 IAR 工程的 build group，会直接导致 link 失败。当前 `main_cm7_0.c` 未调用它，但属于代码债务，需清理或条件编译隔离。 | **中** |

## 功能需求（Functional Requirements）

- **FR-1**：`mahony_update()` 在任何合法加速度输入下都必须执行完整的滤波更新（即不再错误地提前 return）。
- **FR-2**：`main_cm7_0.c` 的 `main()` 函数中不得再出现 K&R 风格的函数内声明。
- **FR-3**：所有 `.c` / `.h` 文件的头文件包含不得出现重复的 `#include`。
- **FR-4**：同一个宏不得同时在 `.c` 和对应的 `.h` 中重复定义。
- **FR-5**：`init.c` 必须要么能独立编译通过，要么通过条件编译隔离，要么移除对不存在符号的引用。

## 非功能需求

- **NFR-1**：**行为等价**：除 `mahony_update()` 的 bug 修复可能改变行为（由错误→正确）外，其他改动必须保持生成代码语义 100% 不变。
- **NFR-2**：**编译零 warning**：不引入新的 warning；对原本可能产生的 redefinition warning 进行根除。
- **NFR-3**：**保持编码风格一致**：继续使用现有风格（`//` 注释、`snake_case`、空行结构等）。

## 约束

- **技术**：
  - 编译器：IAR EWARM 9.40.1（C99 + IAR 扩展）
  - 语言标准：保留对 Cortex-M7 的现有编译选项，不变更 `-std`、优化等级
  - 链接脚本：`linker_directives_tviibh4m.icf` 保持不变
  - **禁止修改 `libraries/` 目录下的任何文件**（仅修改 `project/` 下的用户代码）
- **业务**：
  - 修复后，IMU 姿态解算必须在启动 1 秒内恢复正常更新
  - PWM 通道宏 (`PWM_CH1..4`) 的最终解析值必须与修改前一致（验证 `TCPWM_CH30_P10_2` 等）
- **依赖**：仅依赖现有 `zf_common_headfile.h` 暴露的头文件；不引入新的 `.h`

## 假设

- **H1**：`pit_state = 0` 的主循环读写顺序（主循环读 `pit_state` → 置 0 → IMU → 控制）在当前实现下是 **安全的**（不修复为原子访问，因为 100Hz 主循环不会被 PIT 中断嵌套导致丢失一次）。
- **H2**：`motor.c` 中 `MotorMixer` 的符号（+roll/-roll/+pitch/-pitch/+yaw/-yaw）是用户根据自己硬件布局决定的，**不在本次修复范围**。

## 验收标准

### AC-1：Mahony 滤波器不再错误跳过
- **Given**：IMU 上电且已完成零偏校准
- **When**：IMU 返回的加速度向量在某一轴上接近零（水平静置等姿态，例如 az ≈ 1.0, ax ≈ 0.0, ay ≈ 0.0 这种完全合法的重力向量）
- **Then**：`mahony_update()` 必须执行完整更新，不得提前 return；`attitude.pitch/roll/yaw` 持续有非零更新值
- **Verification**：`human-judgment` + 串口 printf 输出
- **Notes**：必须在真实硬件上验证；通过串口观察 attitude 不再在某些姿态下冻结

### AC-2：主程序无函数内声明
- **Given**：`main_cm7_0.c`
- **When**：审阅文件内容
- **Then**：`main()` 函数体内不得出现任何 `uint8 xxx_init();` 或 `extern void xxx(void);` 形式的函数声明
- **Verification**：`programmatic`（grep "extern" / "uint8.*(" 于 main 函数体内部）

### AC-3：无重复头文件包含
- **Given**：所有被修改的 `.c` 文件
- **When**：扫描 `#include "*.h"` 行
- **Then**：同一 `.h` 在单一文件中不得出现多于一次
- **Verification**：`programmatic`（排序去重检查）

### AC-4：宏唯一来源
- **Given**：`motor.h` / `motor.c`
- **When**：搜索 `PWM_CH` 定义
- **Then**：`PWM_CH1..4` 只能在 `motor.h` 中定义一次，`motor.c` 只通过 include 获得
- **Verification**：`programmatic`（grep `#define PWM_CH`）

### AC-5：`init.c` 不再引用未定义符号
- **Given**：`init.c` 
- **When**：审阅文件
- **Then**：要么所有被调用的函数/宏都在项目中存在，要么整个文件通过 `#if 0 ... #endif` 隔离为"模板示例"
- **Verification**：`programmatic`（IAR clean build 不报错）+ `human-judgment`（注释清晰说明用途）

### AC-6：整体构建通过
- **Given**：所有修复完成后的代码
- **When**：在 IAR 中执行 "Clean + Make"（针对 `cyt4bb7_cm_7_0` 目标）
- **Then**：无 error，无新增 warning；生成的 `.out` / `.hex` 可下载运行
- **Verification**：`programmatic`（IAR build log）

## 开放问题

- [ ] **Q1**：关于 `init.c` — 是直接 `#if 0` 隔离保留为参考，还是彻底删除？本规划默认 **方案 A（#if 0 隔离 + 顶部注释说明）**，保留但不参与 build。
- [ ] **Q2**：关于 `motor.c` 中 MotorMixer 的电机编号注释（"左前/后左/youhou/youqian" 等混乱注释）—— 是否需要在本次修复中规范为清晰的中文/英文注释？本规划 **默认修复**（清理注释，不改动符号逻辑）。

> 注：以上默认方案在进入实现阶段前需用户确认。

---
*本文档是 bug-fix scope 的 PRD；包含 5 个具体 Issue，按严重程度从高→低排序，逐个修复。*

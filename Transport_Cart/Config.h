#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// ========================
// 步进电机 - 左轮
// ========================
// DIR: 方向引脚 (HIGH=前进, LOW=后退)
// PUL: 脉冲引脚 (上升沿走一步)
// EN:  使能引脚 (HIGH=脱机/禁用, LOW=使能)
#define STEPPER_L_DIR_PIN   30
#define STEPPER_L_PUL_PIN   11
#define STEPPER_L_EN_PIN    31

// 步进电机 - 右轮
#define STEPPER_R_DIR_PIN   32
#define STEPPER_R_PUL_PIN   12
#define STEPPER_R_EN_PIN    33

// 步进电机 - 转盘
#define TURNTABLE_DIR_PIN   40
#define TURNTABLE_PUL_PIN   41
#define TURNTABLE_EN_PIN    42

// ========================
// 灰度循迹传感器（前8路 + 后4路）
// ========================
// 数量
#define LS_FRONT_COUNT      8
#define LS_BACK_COUNT       4

// 前侧传感器引脚（从左到右排列: Q1(最左) ~ Q8(最右)）
const uint8_t LS_FRONT_PINS[LS_FRONT_COUNT] = {
    A0, A1, A2, A3, A4, A5, A6, A7
};

// 后侧传感器引脚（从左到右排列: H1(最左) ~ H4(最右)）
const uint8_t LS_BACK_PINS[LS_BACK_COUNT] = {
    A8, A9, A10, A11
};

// 传感器返回值：黑线上方返回 LINE_BLACK，白底上方返回 LINE_WHITE
#define LINE_BLACK          1
#define LINE_WHITE          0

// ========================
// 电动伸缩推杆 (L298N 驱动)
// ========================
// L298N IN1/IN2 控制方向, ENA 短接 5V 全速
// IN1=HIGH IN2=LOW → 伸出
// IN1=LOW  IN2=HIGH → 缩回
// IN1=LOW  IN2=LOW  → 停止
#define PUSH_ROD_IN1_PIN    22
#define PUSH_ROD_IN2_PIN    23
#define PUSH_ROD_ENA_PIN    24

// ========================
// 二维码扫描模块
// ========================
// 通过 Serial3 与二维码模块通讯 (9600bps)
// 模块为自动输出模式: 扫码成功后自动发送 内容ASCII + 回车(0x0D)
//   例: 扫"1" → 0x31 0x0D; 扫"5" → 0x35 0x0D; 数字 10~16 → 两位 ASCII + 0x0D
#define QR_SERIAL           Serial3
#define QR_BAUD_RATE        9600

// ========================
// 系统配置
// ========================
// USB 串口用于调试输出
#define DEBUG_SERIAL        Serial
#define DEBUG_BAUD_RATE     115200

// ========================
// 步进电机参数
// ========================
// 步距角 1.8°, 细分数 32 → 200 × 32 = 6400 脉冲/圈
#define STEPPER_STEPS_PER_REV        6400

// 全局运动速度倍率: 所有步进运动(车轮/转盘)的每步间隔统一除以该值
// 2 = 所有运动速度提高一倍（间隔减半）
#define SPEED_BOOST                  2

// 默认每步间隔（reset() 后的安全默认值，已按 SPEED_BOOST 折半）
#define STEPPER_DEFAULT_STEP_TIME    600

// 最小每步间隔 80us（最大速度下限，防止丢步）
#define STEPPER_MIN_STEP_TIME        80

// ========================
// 底盘运动模型
//  车型: 4轮差速
//    - 前两轮为无动力万向轮，仅支撑车体，不影响转向运动学
//    - 后两轮为步进主动轮，通过左右轮速差实现转向
//    - 旋转中心位于两后轮连线中点
// ========================
// 后轮直径 70mm
#define WHEEL_DIAMETER_MM            70.0f

// 后轮轮距 190mm（两主动轮中心间距）
#define WHEEL_BASE_MM                190.0f

// 单轮周长 = π × 70 = 219.91mm（车轮滚一圈行进的距离）
#define WHEEL_CIRCUMFERENCE_MM       (PI * WHEEL_DIAMETER_MM)

// 每毫米步数 = 6400 / 219.91 ≈ 29.10（直行距离换算系数）
// 用法: 目标步数 = 目标距离(mm) × WHEEL_STEPS_PER_MM
#define WHEEL_STEPS_PER_MM           (STEPPER_STEPS_PER_REV / WHEEL_CIRCUMFERENCE_MM)

// 原地旋转运动学:
//   旋转中心在两后轮连线中点，每轮到中心的距离 = WHEEL_BASE_MM / 2 = 95mm
//   旋转 θ 度时每轮行进的弧长 = π × WHEEL_BASE_MM × θ / 360
//   旋转 1° 每轮步数 = π × 190 / 360 × 29.10 ≈ 48.25
//   旋转 90° 每轮步数 = π × 190 / 4 × 29.10 ≈ 4342

// ========================
// 转盘参数
// ========================
// 转盘 72° 对应步数 = 6400 × 72/360 = 1280
// 144° = 2560 步, 216° = 3840 步（由代码中乘以倍数得到）
#define TURNTABLE_STEPS_PER_72DEG   ((long)(STEPPER_STEPS_PER_REV * 72.0f / 360.0f))

// 颜色识别模块在转盘上的位置: 与车头开口的夹角（正对另一侧 = 180°）
// 车头仓的物块转 TURNTABLE_SENSOR_DEG 即到识别模块处
#define TURNTABLE_SENSOR_DEG        180.0f

// ========================
// TCS230 颜色传感器（8 针模块）
//   S0/S1/S2/S3/OUT/OE + VCC/GND，板上白光 LED 常亮（无独立 LED 脚）
//   OUT 输出频率与所选通道光强成正比，用外部中断计数测频
// ========================
#define COLOR_S0_PIN    50
#define COLOR_S1_PIN    51
#define COLOR_S2_PIN    52
#define COLOR_S3_PIN    53
#define COLOR_OE_PIN    49
#define COLOR_OUT_PIN   2       // Mega 外部中断 INT0 (D2)

// 频率缩放: S0=LOW, S1=HIGH → 2% 输出（拉长脉冲，降低中断频率）
// 单通道采样窗口 (us): 4 通道一轮约 4 × 窗口
#define COLOR_SAMPLE_WINDOW_US   100000UL

// ========================
// 循迹修正参数
// ========================
// 修正死区: |偏移| 小于该值时视为居中，不修正
// 消除直线上传感器量化噪声引起的持续微小修正/来回摆动
#define TRACK_DEADBAND              12

// 线性修正边界: 死区~边界内按比例修正，超过边界直接用最大修正（弯道强力介入）
#define TRACK_CORRECTION_BOUND      25

// 最大修正量 (us): 偏移=边界时慢轮额外延迟量
// 值越大回正力越强，但过大容易过冲导致来回摆动（速度翻倍后已从1000调低到600）
#define TRACK_MAX_CORRECTION_US     600

#endif

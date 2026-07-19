// 底盘运动控制模块
// 组合 Stepper + LineSensor 实现高级运动控制
// 基于状态机: CS_IDLE → CS_MOVING_DIST / CS_TRACKING / CS_ROTATING
// 所有接口均为非阻塞，需在 loop() 中调用 update() 驱动

#ifndef CHASSIS_H
#define CHASSIS_H

#include <Arduino.h>
#include "Config.h"
#include "Stepper.h"
#include "LineSensor.h"

// 底盘状态枚举
typedef enum {
    CS_IDLE,           // 空闲，可接受新指令
    CS_MOVING_DIST,    // 定步移动中（直行/弧线）
    CS_TRACKING,       // 循迹中
    CS_ROTATING,       // 旋转中
    CS_STOPPED         // 紧急停止，需 reset() 恢复
} ChassisState;

class Chassis {
public:
    Chassis();

    // 初始化状态机
    void init();

    // 状态机更新函数，必须在 loop() 中反复调用
    void update();

    // ===================== 基本运动控制 =====================

    // 定步直行: steps=+前进/-后退, stepUs=每步间隔(us)
    bool moveDistance(long steps, unsigned long stepUs);

    // 定点旋转: steps=+CCW/-CW, stepUs=每步间隔(us)
    bool rotate(long steps, unsigned long stepUs);

    // 定角度旋转: degrees=+CCW/-CW, stepUs=每步间隔(us)
    // 自动根据 WHEEL_BASE_MM 和 WHEEL_STEPS_PER_MM 换算步数
    bool rotateAngle(float degrees, unsigned long stepUs);

    // 定距离直行: mm=+前进/-后退, stepUs=每步间隔(us)
    // 自动根据 WHEEL_STEPS_PER_MM 换算步数
    bool moveMm(float mm, unsigned long stepUs);

    // 差分驱动: 左右轮独立步数和速度
    // stepsL/R: 左/右轮步数; usL/R: 左/右每步间隔(us)
    bool moveDiff(long stepsL, long stepsR, unsigned long usL, unsigned long usR);

    // 持续旋转（不设目标，需手动停止）
    void rotateContinuous(bool clockwise, unsigned long stepUs);

    // 停止持续旋转
    void stopContinuous();

    // ===================== 循迹运动控制 =====================

    // 前向循迹: 基于前8路传感器, steps=最大步数
    // baseUs: 居中时两轮每步间隔, 修正量自动按偏移比例计算
    bool trackForward(long steps, unsigned long baseUs);

    // 后向循迹: 基于后4路传感器
    bool trackBackward(long steps, unsigned long baseUs);

    // 前向圆弧循迹: 基于前8路传感器，维持恒定内外轮速差
    // clockwise: true=顺时针, false=逆时针
    // innerUs: 内侧轮每步间隔; outerUs: 外侧轮每步间隔
    bool trackArcForward(long steps, bool clockwise, unsigned long innerUs, unsigned long outerUs);

    // 后向圆弧循迹: 基于后4路传感器
    bool trackArcBackward(long steps, bool clockwise, unsigned long innerUs, unsigned long outerUs);

    // ===================== 状态查询 =====================

    // 当前状态
    ChassisState state() const { return _state; }

    // 是否空闲或已停止（可接受新指令）
    bool isDone() const { return _state == CS_IDLE || _state == CS_STOPPED; }

    // ===================== 控制 =====================

    // 完全重置底盘和电机状态
    void reset();

    // 紧急停止所有电机
    void emergencyStop();

private:
    ChassisState _state;             // 当前状态

    // 定步运动参数
    long _targetStepsL;              // 左轮目标步数
    long _targetStepsR;              // 右轮目标步数
    unsigned long _stepUsL;          // 左轮每步间隔
    unsigned long _stepUsR;          // 右轮每步间隔

    bool _trackControlled;           // 是否处于循迹模式
    unsigned long _trackBaseL;       // 左轮循迹基础步间隔
    unsigned long _trackBaseR;       // 右轮循迹基础步间隔

    bool _rotateCont;                // 是否持续旋转
    bool _rotateContDir;             // 持续旋转方向

    // 内部: 常规定步运动执行逻辑
    void _applyMotion();

    // 内部: 循迹运动执行逻辑
    void _applyTracking();
};

// 全局底盘实例
extern Chassis Cart;

// 全局初始化
void Chassis_Init();

#endif

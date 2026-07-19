#ifndef STEPPER_H
#define STEPPER_H

#include <Arduino.h>

// 步进电机驱动类
// 使用脉冲+方向控制模式，所有操作均为非阻塞
// 需在 loop() 中反复调用 runTo() / runCycle() 来持续输出脉冲
class Stepper {
public:
    // 构造函数
    // dirPin: 方向引脚号
    // pulPin: 脉冲引脚号（上升沿触发一步）
    // enPin:  使能引脚号（HIGH=脱机/禁用, LOW=使能）
    Stepper(uint8_t dirPin, uint8_t pulPin, uint8_t enPin);

    // 初始化引脚模式，设置默认电平
    void init();

    // 设置目标步数并尝试走一步（非阻塞）
    // target > 0: 前进; target < 0: 后退
    // 返回值: true=已到达目标; false=尚未到达, 需要继续调用
    bool runTo(long target);

    // 设置目标步数+指定每步间隔时间（非阻塞）
    // stepIntervalUs: 每两步之间的最小间隔 (微秒)
    // 值越小转速越快; 建议范围 80~2000
    bool runTo(long target, unsigned long stepIntervalUs);

    // 持续旋转（不设目标步数，非阻塞）
    // dir: 旋转方向 (true=前进, false=后退)
    // stepIntervalUs: 每步间隔 (微秒)
    void runCycle(bool dir, unsigned long stepIntervalUs);

    // 重置内部状态: 步数清零, 目标清零, 解除停止, 重新使能电机
    // 在新运动开始前必须调用
    void reset();

    // 查询是否已到达目标步数
    bool isAtTarget() const;

    // 紧急停止: 脱机电机并标记停止，后续 runTo() 直接返回 true
    // 需调用 reset() 或 clearStop() 恢复
    void emergencyStop();

    // 解除紧急停止状态
    void clearStop();

    // 获取当前累计步数（正=累计前进, 负=累计后退）
    long currentStep() const { return _currentStep; }

    // 设置方向反转（电机轴朝向相反时使用）
    void setDirInvert(bool invert) { _dirInvert = invert; }

private:
    uint8_t _dirPin;           // 方向引脚
    uint8_t _pulPin;           // 脉冲引脚
    uint8_t _enPin;            // 使能引脚

    unsigned long _stepIntervalUs;  // 当前设定的每步间隔 (us)
    unsigned long _lastStepUs;      // 上一步发出的时间戳 (micros)
    long _currentStep;              // 当前累计步数
    long _targetStep;               // 目标步数
    bool _stopped;                  // 紧急停止标记
    bool _dirInvert;                // 方向反转（电机镜像安装时=true）

    // 发出一个脉冲+方向信号
    // dir: true=前进, false=后退
    void _pulse(bool dir);

    // 判断距上一步是否已过 stepIntervalUs（即是否可以走下一步）
    bool _canStep() const;
};

// 左右轮全局实例
extern Stepper MotorL;
extern Stepper MotorR;

// 全局初始化函数（初始化左右两个电机）
void Stepper_Init();

#endif

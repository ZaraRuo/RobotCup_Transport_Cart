#include "Stepper.h"
#include "Config.h"

// 两轮全局实例，引脚由 Config.h 定义
Stepper MotorL(STEPPER_L_DIR_PIN, STEPPER_L_PUL_PIN, STEPPER_L_EN_PIN);
Stepper MotorR(STEPPER_R_DIR_PIN, STEPPER_R_PUL_PIN, STEPPER_R_EN_PIN);

// 构造函数: 绑定引脚，设置默认值
// 默认每步间隔使用 STEPPER_DEFAULT_STEP_TIME (1200us)，非常缓慢
// 实际使用时通过 runTo(target, stepUs) 覆盖
Stepper::Stepper(uint8_t dirPin, uint8_t pulPin, uint8_t enPin)
    : _dirPin(dirPin), _pulPin(pulPin), _enPin(enPin),
      _stepIntervalUs(STEPPER_DEFAULT_STEP_TIME),
      _lastStepUs(0), _currentStep(0), _targetStep(0), _stopped(false),
      _dirInvert(false) {
}

// 初始化引脚为输出，默认电平:
//   DIR=LOW, PUL=LOW, EN=LOW(使能)
void Stepper::init() {
    pinMode(_dirPin, OUTPUT);
    pinMode(_pulPin, OUTPUT);
    pinMode(_enPin, OUTPUT);
    digitalWrite(_dirPin, LOW);
    digitalWrite(_pulPin, LOW);
    digitalWrite(_enPin, LOW);
}

// 发出一个脉冲: 先设 DIR 方向 → PUL 拉高 → 延时 1us → PUL 拉低
// 步进电机驱动器在 PUL 上升沿检测方向并走一步
// dir=true 时步数+1（前进），dir=false 时步数-1（后退）
void Stepper::_pulse(bool dir) {
    _lastStepUs = micros();
    _currentStep += (dir ? 1 : -1);
    digitalWrite(_dirPin, _dirInvert ? !dir : dir);
    digitalWrite(_pulPin, HIGH);
    delayMicroseconds(1);
    digitalWrite(_pulPin, LOW);
}

// 判断当前时间距离上一步是否已经超过 stepIntervalUs
// 用于精确控制步进速度
bool Stepper::_canStep() const {
    return (micros() - _lastStepUs) >= _stepIntervalUs;
}

// 判断当前步数是否等于目标步数
bool Stepper::isAtTarget() const {
    return _currentStep == _targetStep;
}

// 设置目标并尝试走一步（非阻塞）
// 每次调用最多走一步；需在循环中反复调用直到返回 true
// target: 绝对目标步数; 内部通过比较 _currentStep 与 target 自动判断方向
// 如果处于紧急停止状态，直接返回 true 不动作
bool Stepper::runTo(long target) {
    if (_stopped) return true;
    _targetStep = target;
    if (isAtTarget()) return true;
    if (_canStep()) {
        _pulse(_currentStep < target);
    }
    return isAtTarget();
}

// 重载版本: 同时设置目标步数和每步间隔时间
// stepIntervalUs 越小转速越快，最小不低于 STEPPER_MIN_STEP_TIME (80us)
bool Stepper::runTo(long target, unsigned long stepIntervalUs) {
    _stepIntervalUs = stepIntervalUs;
    return runTo(target);
}

// 持续旋转模式: 不设目标，每步间隔到期就走一步
// 适合手动控制停止的场景(如传感器触发停止)
// 需在 loop() 中反复调用以维持旋转
void Stepper::runCycle(bool dir, unsigned long stepIntervalUs) {
    if (_stopped) return;
    _stepIntervalUs = stepIntervalUs;
    if (_canStep()) {
        _pulse(dir);
    }
}

// 重置所有内部状态为新运动做准备
// 必须在新运动开始前调用，以清除上一次运动的步数计数和目标
void Stepper::reset() {
    _lastStepUs = 0;
    _currentStep = 0;
    _targetStep = 0;
    _stepIntervalUs = STEPPER_DEFAULT_STEP_TIME;
    _stopped = false;
    digitalWrite(_enPin, LOW);
}

// 紧急停止: 设置停止标记，后续 runTo()/runCycle() 将不发出脉冲
// 通过将 _targetStep 设为 _currentStep 使 runTo() 立即返回 true
void Stepper::emergencyStop() {
    _stopped = true;
    _targetStep = _currentStep;
    digitalWrite(_enPin, HIGH);
}

// 解除紧急停止标记，恢复可运动状态
void Stepper::clearStop() {
    _stopped = false;
    digitalWrite(_enPin, LOW);
}

// 全局初始化: 初始化左右两轮
void Stepper_Init() {
    MotorL.init();
    MotorR.init();
    MotorR.setDirInvert(true);
}

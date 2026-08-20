// 底盘运动控制模块
// 组合 Stepper + LineSensor，实现直行、旋转、循迹等高级运动
// 基于状态机架构，非阻塞式，需在 loop() 中反复调用 update()

#include "Chassis.h"

// 全局底盘实例
Chassis Cart;

// 构造函数: 初始化所有状态变量为默认值
Chassis::Chassis()
    : _state(CS_IDLE), _targetStepsL(0), _targetStepsR(0),
      _stepUsL(0), _stepUsR(0), _trackControlled(false),
      _trackBaseL(0), _trackBaseR(0),
      _rotateCont(false), _rotateContDir(false) {
}

// 初始化: 将状态机置于空闲状态
void Chassis::init() {
    _state = CS_IDLE;
}

// 底盘状态机更新函数，必须在 loop() 中反复调用
// 根据当前状态分发到对应的运动控制逻辑:
//   CS_IDLE/CS_STOPPED → 不动作
//   CS_MOVING_DIST     → 直线定步移动
//   CS_TRACKING         → 循迹移动
//   CS_ROTATING         → 旋转（定点/持续）
void Chassis::update() {
    switch (_state) {
    case CS_MOVING_DIST:
    case CS_TRACKING:
        if (_trackControlled) {
            _applyTracking();   // 循迹控制
        } else {
            _applyMotion();     // 常规定步控制
        }
        break;
    case CS_ROTATING:
        if (_rotateCont) {
            MotorL.runCycle(_rotateContDir, _stepUsL);
            MotorR.runCycle(!_rotateContDir, _stepUsR);
        } else {
            _applyMotion();     // 定点旋转模式
        }
        break;
    default:
        break;
    }
}

// 通用定步运动: 左右轮各走各自的目标步数
// 两个电机分别独立运行 runTo()，全部到达后状态回到 CS_IDLE
void Chassis::_applyMotion() {
    bool lDone = MotorL.runTo(_targetStepsL, _stepUsL);
    bool rDone = MotorR.runTo(_targetStepsR, _stepUsR);
    if (lDone && rDone) {
        _state = CS_IDLE;
    }
}

// 循迹运动控制: 读取传感器偏移量，动态调节左右轮速度
//
// 比例修正逻辑:
//   死区内(|偏移|≤TRACK_DEADBAND)视为居中，不修正 → 消除直线上持续微调/摆动
//   死区~边界(TRACK_CORRECTION_BOUND)内按比例修正，偏移越大内侧轮减速越多
//   超过边界直接用最大修正 TRACK_MAX_CORRECTION_US（弯道强力介入）
//
// 前后向一致: 共用同一套基础速度和修正逻辑
void Chassis::_applyTracking() {
    LineSensors.read();

    bool forward = (_targetStepsL > 0 && _targetStepsR > 0);
    int8_t offset = forward ? LineSensors.frontOffset() : LineSensors.backOffset();

    unsigned long usL = _trackBaseL;
    unsigned long usR = _trackBaseR;

    int8_t mag = offset > 0 ? offset : -offset;
    if (mag > TRACK_DEADBAND) {   // 死区内不修正
        unsigned long correction;
        if (mag <= TRACK_CORRECTION_BOUND) {
            correction = (unsigned long)mag * TRACK_MAX_CORRECTION_US / TRACK_CORRECTION_BOUND;
        } else {
            correction = TRACK_MAX_CORRECTION_US;
        }

        if (offset > 0) {
            // 线偏右 → 减速右侧轮 → 车右转回正（后退时左右对调）
            (forward ? usR : usL) += correction;
        } else {
            // 线偏左 → 减速左侧轮 → 车左转回正
            (forward ? usL : usR) += correction;
        }
    }

    MotorL.runCycle(forward, usL);
    MotorR.runCycle(forward, usR);

    // 步数达到目标时终止（左右轮【都】到达才停，弯道上以慢轮为准；
    // 与 TaskFlow 收料里程碑"两轮都到"的判据一致）
    bool done;
    if (_targetStepsL > 0) {
        done = MotorL.currentStep() >= _targetStepsL && MotorR.currentStep() >= _targetStepsR;
    } else {
        done = MotorL.currentStep() <= _targetStepsL && MotorR.currentStep() <= _targetStepsR;
    }
    if (done) {
        _state = CS_IDLE;
    }
}

// ===================== 基本运动控制接口 =====================

// 定步直行: 左右轮同步等速
// steps: +前进, -后退
// stepUs: 每步间隔 (us)，越小越快
// 返回值: true=运动已开始; false=忙(上一运动未完成)
bool Chassis::moveDistance(long steps, unsigned long stepUs) {
    return moveDiff(steps, steps, stepUs, stepUs);
}

// 定点旋转: 左右轮反向等速，原地自旋
// steps: +CCW(左转) = 左轮后退、右轮前进; -CW(右转) = 左轮前进、右轮后退
// 注意: steps 表示每轮的步数，非角度
bool Chassis::rotate(long steps, unsigned long stepUs) {
    _rotateCont = false;
    return moveDiff(-steps, steps, stepUs, stepUs);
}

// 定角度旋转: 自动换算步数
// degrees: +=CCW, -=CW
// stepUs: 每步间隔 (us)
bool Chassis::rotateAngle(float degrees, unsigned long stepUs) {
    // 弧长 = π × 轮距 × 角度/360
    float arcLen = PI * WHEEL_BASE_MM * (fabs(degrees) / 360.0f);
    long steps = (long)(arcLen * WHEEL_STEPS_PER_MM);
    if (degrees > 0) {
        return rotate(steps, stepUs);     // CCW
    } else {
        return rotate(-steps, stepUs);    // CW
    }
}

// 定距离直行: 自动换算步数
// mm: +前进, -后退
// stepUs: 每步间隔 (us)
bool Chassis::moveMm(float mm, unsigned long stepUs) {
    long steps = (long)(fabs(mm) * WHEEL_STEPS_PER_MM);
    if (mm > 0) {
        return moveDistance(steps, stepUs);
    } else {
        return moveDistance(-steps, stepUs);
    }
}

// 差分驱动: 左右轮各自不同的步数和速度
// 可实现弧线行驶、精确转向等
// 返回值: true=运动已开始; false=忙
bool Chassis::moveDiff(long stepsL, long stepsR, unsigned long usL, unsigned long usR) {
    if (_state != CS_IDLE) return false;
    MotorL.reset();
    MotorR.reset();
    _targetStepsL = stepsL;
    _targetStepsR = stepsR;
    _stepUsL = usL;
    _stepUsR = usR;
    _trackControlled = false;
    _state = CS_MOVING_DIST;
    return true;
}

// 持续旋转: 不设目标步数，需手动停止
// clockwise: true=顺时针(从上方看), false=逆时针
void Chassis::rotateContinuous(bool clockwise, unsigned long stepUs) {
    MotorL.reset();
    MotorR.reset();
    _rotateCont = true;
    _rotateContDir = clockwise;
    _stepUsL = stepUs;
    _stepUsR = stepUs;
    _state = CS_ROTATING;
}

// 停止持续旋转
void Chassis::stopContinuous() {
    if (_rotateCont) {
        _state = CS_IDLE;
        _rotateCont = false;
    }
}

// ===================== 循迹运动控制接口 =====================

// 前向循迹: 基于前侧8路传感器追踪黑线
// steps: 最大步数（防止走丢后无限运行）
// baseUs: 居中时左右轮每步间隔，修正量自动按比例计算
bool Chassis::trackForward(long steps, unsigned long baseUs) {
    if (_state != CS_IDLE) return false;
    MotorL.reset();
    MotorR.reset();
    _targetStepsL = steps;
    _targetStepsR = steps;
    _trackControlled = true;
    _trackBaseL = baseUs;
    _trackBaseR = baseUs;
    _state = CS_TRACKING;
    return true;
}

// 后向循迹: 基于后侧4路传感器追踪黑线
bool Chassis::trackBackward(long steps, unsigned long baseUs) {
    return trackForward(-steps, baseUs);
}

// 前向循迹（按毫米）: 自动换算 mm → 步数后循迹
bool Chassis::trackForwardMm(float mm, unsigned long baseUs) {
    if (mm < 0) return false;
    long steps = (long)(mm * WHEEL_STEPS_PER_MM);   // 每毫米 ≈ 29.10 步
    return trackForward(steps, baseUs);
}

// 后向循迹（按毫米）: 自动换算 mm → 步数后循迹
bool Chassis::trackBackwardMm(float mm, unsigned long baseUs) {
    if (mm < 0) return false;
    long steps = (long)(mm * WHEEL_STEPS_PER_MM);   // 每毫米 ≈ 29.10 步
    return trackBackward(steps, baseUs);
}

// 前向圆弧循迹: 基于前侧8路传感器，维持恒定内外轮速差
// clockwise=true:  顺时针圆弧，左轮外侧(快) / 右轮内侧(慢)
// clockwise=false: 逆时针圆弧，右轮外侧(快) / 左轮内侧(慢)
// innerUs: 内侧轮每步间隔（大=慢）; outerUs: 外侧轮每步间隔（小=快）
bool Chassis::trackArcForward(long steps, bool clockwise, unsigned long innerUs, unsigned long outerUs) {
    if (_state != CS_IDLE) return false;
    MotorL.reset();
    MotorR.reset();
    _targetStepsL = steps;
    _targetStepsR = steps;
    _trackControlled = true;
    if (clockwise) {
        _trackBaseL = outerUs;
        _trackBaseR = innerUs;
    } else {
        _trackBaseL = innerUs;
        _trackBaseR = outerUs;
    }
    _state = CS_TRACKING;
    return true;
}

// 后向圆弧循迹: 基于后侧4路传感器
bool Chassis::trackArcBackward(long steps, bool clockwise, unsigned long innerUs, unsigned long outerUs) {
    return trackArcForward(-steps, clockwise, innerUs, outerUs);
}

// ===================== 控制接口 =====================

// 完全重置底盘状态和电机
void Chassis::reset() {
    _state = CS_IDLE;
    MotorL.reset();
    MotorR.reset();
    _trackControlled = false;
    _rotateCont = false;
}

// 紧急停止所有电机
void Chassis::emergencyStop() {
    _state = CS_STOPPED;
    MotorL.emergencyStop();
    MotorR.emergencyStop();
}

// 全局初始化
void Chassis_Init() {
    Cart.init();
}

// 转盘步进电机 + 5 仓位管理 + 颜色识别定位
// 仓位编号 0~4（车头推进方向顺序），车头开口为 0 号参考；
// 内部自跟踪绝对角度 _deg（上电=0，+72° = 车头仓号+1 方向），
// 支持任意角度旋转(rotateDegrees)、按仓最短路径定位到车头(rotateToCell)
// 或到颜色识别模块(rotateToSensor)、占用标记与每仓颜色记录。

#include "Turntable.h"

// 全局实例
Turntable TurntableMotor;

// 角度差归一到 (-180, 180] 的最短旋转量
float Turntable::_normDelta(float d) {
    while (d > 180.0f) d -= 360.0f;
    while (d <= -180.0f) d += 360.0f;
    return d;
}

Turntable::Turntable()
    : _motor(TURNTABLE_DIR_PIN, TURNTABLE_PUL_PIN, TURNTABLE_EN_PIN),
      _moving(false), _targetSteps(0), _deg(0.0f), _pendingDeg(0.0f) {
    for (uint8_t i = 0; i < CELL_COUNT; i++) {
        _occupied[i] = false;
        _color[i] = 0;
    }
}

void Turntable::init() {
    _motor.init();
    _moving = false;
    _pendingDeg = 0.0f;
    _deg = 0.0f;                        // 上电默认 0 号仓对准车头开口
    for (uint8_t i = 0; i < CELL_COUNT; i++) {
        _occupied[i] = false;
        _color[i] = 0;
    }
}

// 状态机驱动: 旋转完成后累计绝对角度
void Turntable::update() {
    if (!_moving) return;
    if (_motor.runTo(_targetSteps)) {
        _moving = false;
        _deg += _pendingDeg;
        _pendingDeg = 0.0f;
    }
}

// 旋转 multiples × 72°（非阻塞）
bool Turntable::rotateMultiples(int multiples, unsigned long stepUs) {
    return rotateDegrees((float)multiples * 72.0f, stepUs);
}

// 旋转任意角度（非阻塞）
bool Turntable::rotateDegrees(float degrees, unsigned long stepUs) {
    if (_moving) return false;
    _motor.reset();
    _targetSteps = (long)(degrees / 360.0f * STEPPER_STEPS_PER_REV);
    _pendingDeg = degrees;
    _motor.runTo(_targetSteps, stepUs);
    _moving = true;
    return true;
}

// 指定仓位转到车头开口（最短路径，非阻塞）
bool Turntable::rotateToCell(uint8_t cell, unsigned long stepUs) {
    if (_moving) return false;
    float target = (float)(cell % CELL_COUNT) * 72.0f;
    float delta = _normDelta(target - _deg);
    if (delta > -0.1f && delta < 0.1f) return true;   // 已在车头
    return rotateDegrees(delta, stepUs);
}

// 指定仓位转到颜色识别模块处（最短路径，非阻塞）
// 车头仓 → 恰好转 TURNTABLE_SENSOR_DEG(180°)
bool Turntable::rotateToSensor(uint8_t cell, unsigned long stepUs) {
    if (_moving) return false;
    float target = (float)(cell % CELL_COUNT) * 72.0f + TURNTABLE_SENSOR_DEG;
    float delta = _normDelta(target - _deg);
    if (delta > -0.1f && delta < 0.1f) return true;   // 已对准识别模块
    return rotateDegrees(delta, stepUs);
}

// 复位（仅复位运动状态，不清仓位数据/角度）
void Turntable::reset() {
    _motor.reset();
    _moving = false;
    _pendingDeg = 0.0f;
}

void Turntable::emergencyStop() {
    _motor.emergencyStop();
    _moving = false;
    _pendingDeg = 0.0f;
}

// ===================== 仓位管理 =====================

// 当前车头开口最近的仓位号（72° 整数倍对准时精确）
uint8_t Turntable::frontCell() const {
    float cells = _deg / 72.0f;
    int idx = (int)(cells + (cells >= 0 ? 0.5f : -0.5f));   // 四舍五入(远离0)
    idx %= CELL_COUNT;
    if (idx < 0) idx += CELL_COUNT;
    return (uint8_t)idx;
}

bool Turntable::isOccupied(uint8_t cell) const {
    if (cell >= CELL_COUNT) return false;
    return _occupied[cell];
}

// 标记当前车头仓位已收入一个物块
void Turntable::occupyFront() {
    _occupied[frontCell()] = true;
}

void Turntable::clearCell(uint8_t cell) {
    if (cell >= CELL_COUNT) return;
    _occupied[cell] = false;
    _color[cell] = 0;
}

void Turntable::setCellColor(uint8_t cell, uint8_t color) {
    if (cell >= CELL_COUNT) return;
    _color[cell] = color;
}

uint8_t Turntable::cellColor(uint8_t cell) const {
    if (cell >= CELL_COUNT) return 0;
    return _color[cell];
}

// 全局初始化
void Turntable_Init() {
    TurntableMotor.init();
}

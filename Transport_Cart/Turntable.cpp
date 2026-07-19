#include "Turntable.h"

Turntable TurntableMotor;

Turntable::Turntable()
    : _motor(TURNTABLE_DIR_PIN, TURNTABLE_PUL_PIN, TURNTABLE_EN_PIN),
      _moving(false), _targetSteps(0) {
}

void Turntable::init() {
    _motor.init();
    _moving = false;
}

void Turntable::update() {
    if (!_moving) return;
    if (_motor.runTo(_targetSteps)) {
        _moving = false;
    }
}

bool Turntable::rotateMultiples(int multiples, unsigned long stepUs) {
    if (_moving) return false;
    _motor.reset();
    _targetSteps = (long)multiples * TURNTABLE_STEPS_PER_72DEG;
    _motor.runTo(_targetSteps, stepUs);
    _moving = true;
    return true;
}

bool Turntable::isDone() const {
    return !_moving;
}

void Turntable::reset() {
    _motor.reset();
    _moving = false;
    _targetSteps = 0;
}

void Turntable::emergencyStop() {
    _motor.emergencyStop();
    _moving = false;
}

void Turntable_Init() {
    TurntableMotor.init();
}

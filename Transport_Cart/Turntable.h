#ifndef TURNTABLE_H
#define TURNTABLE_H

#include <Arduino.h>
#include "Config.h"
#include "Stepper.h"

class Turntable {
public:
    Turntable();

    void init();

    void update();

    bool rotateMultiples(int multiples, unsigned long stepUs = 200);

    bool isDone() const;

    void reset();

    void emergencyStop();

private:
    Stepper _motor;
    bool _moving;
    long _targetSteps;
};

extern Turntable TurntableMotor;

void Turntable_Init();

#endif

// 电动伸缩推杆驱动模块 (L298N H桥)
// IN1/IN2: 方向控制, ENA: 使能 (短接5V全速或PWM调速)
// IN1=HIGH IN2=LOW → 伸出
// IN1=LOW  IN2=HIGH → 缩回
// IN1=LOW  IN2=LOW  → 停止

#ifndef PUSH_ROD_H
#define PUSH_ROD_H

#include <Arduino.h>
#include "Config.h"

class PushRod {
public:
    PushRod();

    void init();

    void extend();

    void retract();

    void toggle();

    void stop();

    bool isExtended() const { return _extended; }

    void pulse(uint16_t durationMs);

private:
    bool _extended;
};

extern PushRod Rod;

void PushRod_Init();

#endif

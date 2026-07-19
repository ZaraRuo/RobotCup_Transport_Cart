#include "PushRod.h"

PushRod Rod;

PushRod::PushRod() : _extended(false) {
}

void PushRod::init() {
    pinMode(PUSH_ROD_IN1_PIN, OUTPUT);
    pinMode(PUSH_ROD_IN2_PIN, OUTPUT);
    pinMode(PUSH_ROD_ENA_PIN, OUTPUT);
    digitalWrite(PUSH_ROD_IN1_PIN, LOW);
    digitalWrite(PUSH_ROD_IN2_PIN, LOW);
    digitalWrite(PUSH_ROD_ENA_PIN, HIGH);
    _extended = false;
}

void PushRod::extend() {
    digitalWrite(PUSH_ROD_IN1_PIN, HIGH);
    digitalWrite(PUSH_ROD_IN2_PIN, LOW);
    _extended = true;
}

void PushRod::retract() {
    digitalWrite(PUSH_ROD_IN1_PIN, LOW);
    digitalWrite(PUSH_ROD_IN2_PIN, HIGH);
    _extended = false;
}

void PushRod::toggle() {
    if (_extended) {
        retract();
    } else {
        extend();
    }
}

void PushRod::stop() {
    digitalWrite(PUSH_ROD_IN1_PIN, LOW);
    digitalWrite(PUSH_ROD_IN2_PIN, LOW);
}

void PushRod::pulse(uint16_t durationMs) {
    extend();
    delay(durationMs);
    retract();
}

void PushRod_Init() {
    Rod.init();
}

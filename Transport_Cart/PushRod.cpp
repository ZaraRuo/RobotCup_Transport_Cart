// 电动伸缩推杆驱动模块 (L298N H桥)
// 对外接口: extendToTop(ms) / retractToBottom(ms)，时长由调用者输入
// IN1/IN2: 方向控制, ENA: 全速使能 (HIGH)
// IN1=HIGH IN2=LOW → 伸出
// IN1=LOW  IN2=HIGH → 缩回
// IN1=LOW  IN2=LOW  → 停止

#include "PushRod.h"

// 全局实例
PushRod Rod;

PushRod::PushRod() {
}

void PushRod::init() {
    pinMode(PUSH_ROD_IN1_PIN, OUTPUT);
    pinMode(PUSH_ROD_IN2_PIN, OUTPUT);
    pinMode(PUSH_ROD_ENA_PIN, OUTPUT);
    digitalWrite(PUSH_ROD_IN1_PIN, LOW);
    digitalWrite(PUSH_ROD_IN2_PIN, LOW);
    digitalWrite(PUSH_ROD_ENA_PIN, HIGH);   // 全速使能
}

// 伸出: 运行 ms 毫秒后自动停止（阻塞）
void PushRod::extendToTop(unsigned long ms) {
    _run(true, ms);
}

// 缩回: 运行 ms 毫秒后自动停止（阻塞）
void PushRod::retractToBottom(unsigned long ms) {
    _run(false, ms);
}

// 阻塞运行指定时长后停止
void PushRod::_run(bool extend, unsigned long ms) {
    digitalWrite(PUSH_ROD_IN1_PIN, extend ? HIGH : LOW);
    digitalWrite(PUSH_ROD_IN2_PIN, extend ? LOW : HIGH);
    delay(ms);
    digitalWrite(PUSH_ROD_IN1_PIN, LOW);
    digitalWrite(PUSH_ROD_IN2_PIN, LOW);
}

// 全局初始化
void PushRod_Init() {
    Rod.init();
}

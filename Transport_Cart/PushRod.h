// 电动伸缩推杆驱动模块 (L298N H桥)
// 对外提供 2 个运动接口，时长由调用者输入（毫秒）：
//   extendToTop(ms)     伸出 ms 毫秒后自动停止
//   retractToBottom(ms) 缩回 ms 毫秒后自动停止
// 说明:
//   - 推杆无位置/行程反馈，靠运行时间控制行程，时长由调用处指定
//   - 若电机中途先顶到机械限位，剩余时间会堵转顶住限位，属预期行为，
//     请保证输入时长 ≥ 实际所需用时
//   - 两个运动接口均为阻塞实现（内部 delay），调用期间主循环暂停，
//     适合"机器人停稳后推放货物"的场景

#ifndef PUSH_ROD_H
#define PUSH_ROD_H

#include <Arduino.h>
#include "Config.h"

class PushRod {
public:
    PushRod();

    // 初始化引脚并置为停止状态
    void init();

    // 伸出：运行 ms 毫秒后自动停止（阻塞）
    void extendToTop(unsigned long ms);

    // 缩回：运行 ms 毫秒后自动停止（阻塞）
    void retractToBottom(unsigned long ms);

private:
    // 按方向运行指定时长后停止（阻塞）
    // extend=true  → IN1=HIGH/IN2=LOW 伸出
    // extend=false → IN1=LOW/IN2=HIGH 缩回
    void _run(bool extend, unsigned long ms);
};

// 全局实例
extern PushRod Rod;

// 全局初始化
void PushRod_Init();

#endif

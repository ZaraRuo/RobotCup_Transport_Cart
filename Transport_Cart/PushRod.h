// 电动伸缩推杆驱动模块 (L298N H桥)
// 对外只提供 2 个运动接口：
//   extendToTop()     伸到顶：持续伸出 PUSH_ROD_TRAVEL_MS (10s) 后自动停止
//   retractToBottom() 伸到底：持续缩回 PUSH_ROD_TRAVEL_MS (10s) 后自动停止
// 说明:
//   - 推杆无位置/行程反馈，靠固定运行时间覆盖全程，时间在 Config.h 中调整
//   - 若电机中途先顶到机械限位，剩余时间会堵转顶住限位，属预期行为，
//     请保证 PUSH_ROD_TRAVEL_MS ≥ 实际全程用时
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

    // 伸到顶（运行 10s 后自动停止，阻塞）
    void extendToTop();

    // 伸到底（运行 10s 后自动停止，阻塞）
    void retractToBottom();

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

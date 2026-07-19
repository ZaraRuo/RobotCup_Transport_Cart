// 12路灰度循迹传感器采集与处理
// 前侧 8 路 (Q1~Q8) — 前进循迹 + 圆弧循迹
// 后侧 4 路 (H1~H4) — 后退循迹
// 偏移量计算基于加权质心法，返回 -100~+100

#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <Arduino.h>
#include "Config.h"

class LineSensor {
public:
    LineSensor();

    // 初始化所有引脚为输入
    void init();

    // 读取全部 12 路传感器值（必须在使用 getter 前调用）
    void read();

    // 前侧黑线中心偏移量 (-100~+100)
    // +100=线在最右侧, -100=线在最左侧, 0=居中或无黑线
    int8_t frontOffset() const;

    // 后侧黑线中心偏移量 (-100~+100)
    int8_t backOffset() const;

    // 获取前侧指定通道原始值 (0=白, 1=黑)
    uint8_t front(uint8_t index) const;

    // 获取后侧指定通道原始值
    uint8_t back(uint8_t index) const;

    // 前侧是否有任意传感器检测到黑线
    bool frontSeesLine() const;

    // 后侧是否有任意传感器检测到黑线
    bool backSeesLine() const;

    // 前侧指定通道是否为黑
    bool frontBlack(uint8_t index) const;

    // 后侧指定通道是否为黑
    bool backBlack(uint8_t index) const;

    // 串口打印所有传感器值
    void printDebug() const;

private:
    uint8_t _frontValues[LS_FRONT_COUNT];  // 前8路缓存
    uint8_t _backValues[LS_BACK_COUNT];    // 后4路缓存
};

extern LineSensor LineSensors;

void LineSensor_Init();

#endif

// TCS230 颜色传感器模块（8 针版）
//   S0/S1/S2/S3/OUT/OE + VCC/GND，板上白光 LED 常亮（无独立 LED 脚）
//
// 测量原理（最佳实践，非阻塞）:
//   TCS230 的 OUT 输出频率与所选滤波通道的光强成正比。
//   用 S2/S3 切换 Clear/红/绿/蓝 4 个通道，每个通道在固定时间窗口
//   (COLOR_SAMPLE_WINDOW_US) 内通过外部中断统计 OUT 上升沿个数，
//   计数值即正比于该通道频率(光强)。
//   测量完成后按白平衡系数归一化(白色表面≈255)，再按阈值分类
//   白/黑/蓝/绿/红。
//
// 使用方式:
//   ColorSensor_Init()   — 初始化（引脚 + 中断）
//   ColorSensors.begin() — 开始一轮 4 通道测量（立即返回）
//   ColorSensors.update()— 在 loop() 中反复调用驱动状态机
//   ColorSensors.available() 为 true 时读取 color()/rawXxx()
//   换环境光照后务必执行 beginWhiteBalance()（对准白色表面）重新标定

#ifndef COLOR_SENSOR_H
#define COLOR_SENSOR_H

#include <Arduino.h>
#include "Config.h"

class ColorSensor {
public:
    // 颜色编码（与旧项目一致，便于流程复用）
    enum Color {
        COLOR_NONE  = 0,
        COLOR_GREEN = 1,
        COLOR_WHITE = 2,
        COLOR_RED   = 3,
        COLOR_BLACK = 4,
        COLOR_BLUE  = 5
    };

    ColorSensor();

    // 初始化引脚并挂接 OUT 上升沿中断
    void init();

    // 开始一轮测量（非阻塞）
    void begin();

    // 状态机驱动，必须在 loop() 中反复调用
    void update();

    // 本轮测量是否完成（读取后需 begin() 开始下一轮）
    bool available() const { return _available; }

    // 上一轮识别结果
    Color color() const { return _color; }

    // 各通道原始计数值（窗口内上升沿数，正比于频率/光强）
    uint32_t rawRed() const   { return _rawRed; }
    uint32_t rawGreen() const { return _rawGreen; }
    uint32_t rawBlue() const  { return _rawBlue; }
    uint32_t rawClear() const { return _rawClear; }

    // 白平衡归一化值 (0~255)
    float normRed() const   { return _normR; }
    float normGreen() const { return _normG; }
    float normBlue() const  { return _normB; }

    // 开始白平衡标定：将传感器对准白色表面后调用，
    // 下一轮测量完成时自动用白参考计算系数（白≈255）
    void beginWhiteBalance();

    // 手动设置白平衡系数（默认 1.0，即不做校正）
    void setWhiteBalance(float r, float g, float b);

    // 复位到空闲
    void reset();

    // OUT 上升沿中断服务函数（静态）
    static void pulseIsr();

private:
    enum SampleChannel {
        CH_CLEAR = 0,
        CH_RED,
        CH_GREEN,
        CH_BLUE,
        CH_DONE
    };

    SampleChannel _channel;         // 当前采样通道
    unsigned long _windowStart;     // 当前窗口起始时刻 (micros)

    uint32_t _rawRed, _rawGreen, _rawBlue, _rawClear;
    float _normR, _normG, _normB;
    float _coeffR, _coeffG, _coeffB;    // 白平衡系数
    bool _available;                // 本轮测量完成标记
    bool _wbPending;                // 等待以白色为参考计算系数
    Color _color;                   // 最近识别结果

    // 切换 S2/S3 滤波通道
    void _selectChannel(SampleChannel ch);
    // 一轮测量完成：应用白平衡系数并分类
    void _finish();
    // 颜色分类
    Color _classify(float r, float g, float b) const;
};

// 全局单例
extern ColorSensor ColorSensors;

// 全局初始化
void ColorSensor_Init();

#endif

// TCS230 颜色传感器驱动（非阻塞，中断计数法）
//
// 原理:
//   TCS230 OUT 输出方波频率与当前滤波通道光强成正比。
//   S2/S3 组合选择通道: (H,L)=Clear, (L,L)=红, (H,H)=绿, (L,H)=蓝。
//   每个通道在 COLOR_SAMPLE_WINDOW_US 窗口内统计 OUT 上升沿数，
//   计数值正比于频率，即正比于该通道光强。
//   测量完成后按白平衡系数归一化: norm = raw × (255 / white_raw)，
//   白色表面各通道归一化值约 255，再按阈值分类。
//
// 与阻塞式 pulseIn 相比（旧代码）:
//   1. 不阻塞主循环，可无缝集成到本项目的非阻塞状态机架构;
//   2. 窗口计数 = 真实频率统计，抗干扰更好、更稳定;
//   3. 中断由硬件完成，测量精度不受 loop 耗时影响。

#include "ColorSensor.h"

// OUT 上升沿计数（ISR 中累加，采样状态机按窗口清零读取）
static volatile uint32_t s_pulseCount = 0;

// 全局单例
ColorSensor ColorSensors;

ColorSensor::ColorSensor()
    : _channel(CH_DONE), _windowStart(0),
      _rawRed(0), _rawGreen(0), _rawBlue(0), _rawClear(0),
      _normR(0), _normG(0), _normB(0),
      _coeffR(1.0f), _coeffG(1.0f), _coeffB(1.0f),
      _available(false), _wbPending(false), _color(COLOR_NONE) {
}

void ColorSensor::init() {
    pinMode(COLOR_S0_PIN, OUTPUT);
    pinMode(COLOR_S1_PIN, OUTPUT);
    pinMode(COLOR_S2_PIN, OUTPUT);
    pinMode(COLOR_S3_PIN, OUTPUT);
    pinMode(COLOR_OE_PIN, OUTPUT);
    pinMode(COLOR_OUT_PIN, INPUT);

    // 输出频率缩放 2%: S0=LOW, S1=HIGH（拉长脉冲，降低中断频率）
    digitalWrite(COLOR_S0_PIN, LOW);
    digitalWrite(COLOR_S1_PIN, HIGH);

    // OE 低有效: LOW = 开启频率输出
    digitalWrite(COLOR_OE_PIN, LOW);

    // OUT 上升沿中断计数（Mega D2 = INT0）
    attachInterrupt(digitalPinToInterrupt(COLOR_OUT_PIN), ColorSensor::pulseIsr, RISING);

    _channel = CH_DONE;
    _available = false;
}

// ISR: 统计 OUT 上升沿
void ColorSensor::pulseIsr() {
    s_pulseCount++;
}

// 开始一轮测量: 清零计数，从 Clear 通道开始采样
void ColorSensor::begin() {
    noInterrupts();
    s_pulseCount = 0;
    interrupts();
    _available = false;
    _channel = CH_CLEAR;
    _selectChannel(CH_CLEAR);
    _windowStart = micros();
}

// 状态机: 每个窗口结束后记录当前通道计数并切换下一通道
void ColorSensor::update() {
    if (_channel == CH_DONE) return;   // 空闲，等待 begin()

    unsigned long now = micros();
    if (now - _windowStart < COLOR_SAMPLE_WINDOW_US) return;

    // 窗口结束: 读取计数并清零
    noInterrupts();
    uint32_t cnt = s_pulseCount;
    s_pulseCount = 0;
    interrupts();

    switch (_channel) {
    case CH_CLEAR: _rawClear = cnt; _channel = CH_RED;   _selectChannel(CH_RED);   break;
    case CH_RED:   _rawRed   = cnt; _channel = CH_GREEN; _selectChannel(CH_GREEN); break;
    case CH_GREEN: _rawGreen = cnt; _channel = CH_BLUE;  _selectChannel(CH_BLUE);  break;
    case CH_BLUE:  _rawBlue  = cnt; _channel = CH_DONE;  _finish();                break;
    default: break;
    }
    _windowStart = now;
}

// 一轮测量完成: 若处于白平衡标定，用白参考计算系数；然后归一化并分类
void ColorSensor::_finish() {
    if (_wbPending) {
        _coeffR = 255.0f / (float)(_rawRed   > 0 ? _rawRed   : 1);
        _coeffG = 255.0f / (float)(_rawGreen > 0 ? _rawGreen : 1);
        _coeffB = 255.0f / (float)(_rawBlue  > 0 ? _rawBlue  : 1);
        _wbPending = false;
    }

    _normR = (float)_rawRed   * _coeffR;
    _normG = (float)_rawGreen * _coeffG;
    _normB = (float)_rawBlue  * _coeffB;
    _color = _classify(_normR, _normG, _normB);
    _available = true;
}

// 颜色分类（阈值在归一化 0~255 空间）
// 顺序重要: 先判白/黑，再按通道优势判蓝/绿/红
ColorSensor::Color ColorSensor::_classify(float r, float g, float b) const {
    if (r > 200.0f && g > 200.0f && b > 200.0f) return COLOR_WHITE;  // 白
    if (b > r + 10.0f && b > g + 10.0f)          return COLOR_BLUE;   // 蓝
    if (g > b + 2.0f && g > r + 2.0f)            return COLOR_GREEN;  // 绿
    if (r > g + 20.0f && r > b + 20.0f)          return COLOR_RED;    // 红
    return COLOR_BLACK;  // 未知 → 强制回退到黑，避免返回 COLOR_NONE
}

// S2/S3 通道选择: (S2,S3)
void ColorSensor::_selectChannel(SampleChannel ch) {
    switch (ch) {
    case CH_CLEAR: digitalWrite(COLOR_S2_PIN, HIGH); digitalWrite(COLOR_S3_PIN, LOW);  break;  // Clear
    case CH_RED:   digitalWrite(COLOR_S2_PIN, LOW);  digitalWrite(COLOR_S3_PIN, LOW);  break;  // 红
    case CH_GREEN: digitalWrite(COLOR_S2_PIN, HIGH); digitalWrite(COLOR_S3_PIN, HIGH); break;  // 绿
    case CH_BLUE:  digitalWrite(COLOR_S2_PIN, LOW);  digitalWrite(COLOR_S3_PIN, HIGH); break;  // 蓝
    default: break;
    }
}

// 白平衡: 对准白色表面后调用，重新开始一轮测量作为白参考
void ColorSensor::beginWhiteBalance() {
    _wbPending = true;
    begin();
}

void ColorSensor::setWhiteBalance(float r, float g, float b) {
    _coeffR = r;
    _coeffG = g;
    _coeffB = b;
}

void ColorSensor::reset() {
    noInterrupts();
    s_pulseCount = 0;
    interrupts();
    _channel = CH_DONE;
    _available = false;
    _wbPending = false;
}

// 全局初始化
void ColorSensor_Init() {
    ColorSensors.init();
}

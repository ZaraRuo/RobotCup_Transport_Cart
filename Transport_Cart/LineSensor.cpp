// 12路灰度循迹传感器采集与处理模块
// 前侧 8 路 (Q1~Q8) 用于前进循迹和圆弧循迹
// 后侧 4 路 (H1~H4) 用于后退循迹
// 所有传感器返回数字量: 1=黑线, 0=白底

#include "LineSensor.h"

// 全局单例
LineSensor LineSensors;

// 构造函数: 将读写缓冲区归零
LineSensor::LineSensor() {
    memset(_frontValues, 0, sizeof(_frontValues));
    memset(_backValues, 0, sizeof(_backValues));
}

// 初始化所有传感器引脚为输入模式
void LineSensor::init() {
    for (uint8_t i = 0; i < LS_FRONT_COUNT; i++) {
        pinMode(LS_FRONT_PINS[i], INPUT);
    }
    for (uint8_t i = 0; i < LS_BACK_COUNT; i++) {
        pinMode(LS_BACK_PINS[i], INPUT);
    }
}

// 读取全部 12 路传感器值
// 必须先调用此函数，后续 getter 才能返回最新数据
void LineSensor::read() {
    for (uint8_t i = 0; i < LS_FRONT_COUNT; i++) {
        _frontValues[i] = digitalRead(LS_FRONT_PINS[i]);
    }
    for (uint8_t i = 0; i < LS_BACK_COUNT; i++) {
        _backValues[i] = digitalRead(LS_BACK_PINS[i]);
    }
}

// 前侧黑线偏移量计算（加权质心法）
// 8 路传感器编号映射到坐标: -4, -3, -2, -1, +1, +2, +3, +4
// 取所有检测到黑线的传感器坐标的加权平均，然后归一到 [-100, +100]
//   +100 = 线在最右侧（Q8）
//   -100 = 线在最左侧（Q1）
//   0 = 线居中 或 无黑线
int8_t LineSensor::frontOffset() const {
    int32_t sum = 0;
    int32_t count = 0;
    int8_t half = LS_FRONT_COUNT / 2;

    for (uint8_t i = 0; i < LS_FRONT_COUNT; i++) {
        if (_frontValues[i] == LINE_BLACK) {
            sum += (-half + (int32_t)i + (i >= half ? 1 : 0));
            count++;
        }
    }
    if (count == 0) return 0;
    int8_t raw = (int8_t)(sum / count);
    return raw * (100 / half);
}

// 后侧黑线偏移量计算（同 frontOffset 的算法）
// 4 路传感器坐标: -2, -1, +1, +2
// 归一到 [-100, +100]
int8_t LineSensor::backOffset() const {
    int32_t sum = 0;
    int32_t count = 0;
    int8_t half = LS_BACK_COUNT / 2;

    for (uint8_t i = 0; i < LS_BACK_COUNT; i++) {
        if (_backValues[i] == LINE_BLACK) {
            sum += (-half + (int32_t)i + (i >= half ? 1 : 0));
            count++;
        }
    }
    if (count == 0) return 0;
    int8_t raw = (int8_t)(sum / count);
    return raw * (100 / half);
}

// 获取前侧指定通道原始值（0=白, 1=黑）
uint8_t LineSensor::front(uint8_t index) const {
    if (index >= LS_FRONT_COUNT) return 0;
    return _frontValues[index];
}

// 获取后侧指定通道原始值
uint8_t LineSensor::back(uint8_t index) const {
    if (index >= LS_BACK_COUNT) return 0;
    return _backValues[index];
}

// 前侧是否有任意传感器检测到黑线
bool LineSensor::frontSeesLine() const {
    for (uint8_t i = 0; i < LS_FRONT_COUNT; i++) {
        if (_frontValues[i] == LINE_BLACK) return true;
    }
    return false;
}

// 后侧是否有任意传感器检测到黑线
bool LineSensor::backSeesLine() const {
    for (uint8_t i = 0; i < LS_BACK_COUNT; i++) {
        if (_backValues[i] == LINE_BLACK) return true;
    }
    return false;
}

// 前侧指定通道是否为黑
bool LineSensor::frontBlack(uint8_t index) const {
    return front(index) == LINE_BLACK;
}

// 后侧指定通道是否为黑
bool LineSensor::backBlack(uint8_t index) const {
    return back(index) == LINE_BLACK;
}

// 调试打印: 前8路 + 后4路原始值 + 前后偏移量
void LineSensor::printDebug() const {
    DEBUG_SERIAL.print(F("Front: "));
    for (uint8_t i = 0; i < LS_FRONT_COUNT; i++) {
        DEBUG_SERIAL.print((int)_frontValues[i]);
        DEBUG_SERIAL.print(' ');
    }
    DEBUG_SERIAL.print(F(" | Back: "));
    for (uint8_t i = 0; i < LS_BACK_COUNT; i++) {
        DEBUG_SERIAL.print((int)_backValues[i]);
        DEBUG_SERIAL.print(' ');
    }
    DEBUG_SERIAL.print(F(" | OffF="));
    DEBUG_SERIAL.print((int)frontOffset());
    DEBUG_SERIAL.print(F(" OffB="));
    DEBUG_SERIAL.println((int)backOffset());
}

// 全局初始化
void LineSensor_Init() {
    LineSensors.init();
}

// 二维码扫描模块
// 通过 Serial3 与二维码模块通信 (9600bps)
// 模块为自动输出模式：每次扫码成功自动发送 内容ASCII + 回车(0x0D)
//   例：扫"1" → 0x31 0x0D；扫"5" → 0x35 0x0D；10~16 为两位 ASCII
//
// 接收逻辑（刻意保持简单）：
//   1. 逐字节累积到 _buf
//   2. 遇到 CR(0x0D) 或 LF(0x0A) 视为帧结束 → atoi 解析 → 标记可用 → 清空缓冲
//   3. 缓冲只在这两处被清空：帧结束 / flush()。无超时清空、无中途清空，
//      避免误清有效数据（历史教训：过度防错机制曾导致数据被错误清除）
//   4. 不做任何调试输出；原始字节通过 _byteListener 回调交给调用方

#include "QRScanner.h"
#include <stdlib.h>   // atoi

// 全局扫描器实例
QRScanner Scanner;

// 构造函数
QRScanner::QRScanner()
    : _bufLen(0), _result(0), _hasNew(false), _byteListener(NULL) {
    _buf[0] = '\0';
}

// 初始化 Serial3 串口
void QRScanner::init() {
    QR_SERIAL.begin(QR_BAUD_RATE);
}

// 注册字节监听回调
void QRScanner::setByteListener(void (*listener)(uint8_t)) {
    _byteListener = listener;
}

// 非阻塞更新函数，必须在 loop() 中反复调用
void QRScanner::update() {
    while (QR_SERIAL.available() > 0) {
        uint8_t b = (uint8_t)QR_SERIAL.read();

        // 原始字节回调（由调用方实现，如调试阶段打印 HEX）
        if (_byteListener) {
            _byteListener(b);
        }

        char c = (char)b;
        if (c == '\r' || c == '\n') {
            // 帧结束: 解析本帧（缓冲只在此时清空）
            if (_bufLen > 0) {
                _buf[_bufLen] = '\0';
                _result = atoi(_buf);
                _hasNew = true;
            }
            _bufLen = 0;
        } else if (_bufLen < QR_MAX_FRAME_LEN) {
            // 累积到缓冲；超长时只忽略多余字符，不清空已有数据
            _buf[_bufLen++] = c;
        }
    }
}

// 检查是否有新的扫码结果
// 返回 true: 有新数据，同时清除标记（缓冲已在帧结束时清空）
bool QRScanner::available() {
    if (_hasNew) {
        _hasNew = false;
        return true;
    }
    return false;
}

// 清空所有缓冲区和状态（任务切换时丢弃旧数据用）
void QRScanner::flush() {
    while (QR_SERIAL.available() > 0) {
        QR_SERIAL.read();
    }
    _bufLen = 0;
    _result = 0;
    _hasNew = false;
}

// 全局初始化
void QRScanner_Init() {
    Scanner.init();
}

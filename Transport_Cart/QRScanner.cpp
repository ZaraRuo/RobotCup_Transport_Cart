// 二维码扫描模块
// 通过 Serial3 与二维码模块通信 (9600bps)
// 模块为自动输出模式：每次扫码成功自动发送 内容ASCII + 回车(0x0D)
//   例：扫"1" → 0x31 0x0D；扫"5" → 0x35 0x0D；10~16 为两位 ASCII
// 非阻塞解析：按 CR/LF 分帧，帧结束将内容转为整数（1~16）

#include "QRScanner.h"

// 全局扫描器实例
QRScanner Scanner;

// 构造函数: 初始化所有成员为默认值
QRScanner::QRScanner() : _result(0), _hasNew(false), _lastByteTime(0) {
}

// 初始化 Serial3 串口，预分配字符串缓冲区
void QRScanner::init() {
    QR_SERIAL.begin(QR_BAUD_RATE);
    _buffer.reserve(16);
}

// 非阻塞更新函数，必须在 loop() 中反复调用
// 逻辑:
//   1. 从 Serial3 读取所有可用字节
//   2. 遇到 CR(0x0D) 或 LF(0x0A) 视为一帧结束，将帧内容转为整数
//      （模块只发送 内容ASCII + 0x0D，处理 LF 是为了兼容 \r\n 结尾）
//   3. 空帧（如 \r\n 中的 \n）直接忽略
//   4. 半帧超时保护：不完整帧长时间无结束符则丢弃，防止噪声数据残留
void QRScanner::update() {
    while (QR_SERIAL.available() > 0) {
        char c = (char)QR_SERIAL.read();
        _lastByteTime = micros();

        if (c == '\r' || c == '\n') {
            // 帧结束: 解析本帧内容
            if (_buffer.length() > 0) {
                _result = _buffer.toInt();
                _hasNew = true;
            }
            _buffer = "";
        } else {
            _buffer += c;
            // 防异常数据撑爆缓冲区
            if (_buffer.length() > MAX_FRAME_LEN) {
                _buffer = "";
            }
        }
    }

    // 半帧超时保护: 丢弃残留的不完整帧
    if (_buffer.length() > 0 && (micros() - _lastByteTime) > FRAME_TIMEOUT_US) {
        _buffer = "";
    }
}

// 检查是否有新的扫码结果
// 返回 true: 有新数据，同时自动清除标记和缓冲区
// 返回 false: 无新数据
bool QRScanner::available() {
    if (_hasNew) {
        _hasNew = false;
        _buffer = "";
        return true;
    }
    return false;
}

// 清空所有缓冲区和状态
// 用于在任务间切换时丢弃旧数据
void QRScanner::flush() {
    while (QR_SERIAL.available() > 0) {
        QR_SERIAL.read();
    }
    _buffer = "";
    _result = 0;
    _hasNew = false;
}

// 全局初始化
void QRScanner_Init() {
    Scanner.init();
}

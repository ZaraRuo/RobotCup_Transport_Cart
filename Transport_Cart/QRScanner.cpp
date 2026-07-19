// 二维码扫描模块
// 通过 Serial3 与二维码模块通信
// 非阻塞模式: 在 loop() 中调用 update() 持续接收数据
// 超时机制: 连续 200ms 无新字节则判定传输完成，解析结果

#include "QRScanner.h"

// 全局扫描器实例
QRScanner Scanner;

// 构造函数: 初始化所有成员为默认值
QRScanner::QRScanner() : _result(0), _hasNew(false), _lastByteTime(0) {
}

// 初始化 Serial3 串口，预分配字符串缓冲区
void QRScanner::init() {
    QR_SERIAL.begin(QR_BAUD_RATE);
    _buffer.reserve(32);
}

// 非阻塞更新函数，必须在 loop() 中反复调用
// 逻辑:
//   1. 从 Serial3 读取所有可用字节，追加到 _buffer
//   2. 每次读到新字节时刷新 _lastByteTime
//   3. 当缓冲区非空且距离最后一个字节超过 READ_TIMEOUT_US (200ms)
//      则认为本次传输完成，将 _buffer 解析为整数
void QRScanner::update() {
    // 读取串口缓冲区的所有字节
    while (QR_SERIAL.available() > 0) {
        char c = (char)QR_SERIAL.read();
        _buffer += c;
        _lastByteTime = micros();
    }

    // 超时判定: 有数据且距最后一字节超 200ms 则完成
    if (_buffer.length() > 0) {
        if (micros() - _lastByteTime > READ_TIMEOUT_US) {
            _result = _buffer.toInt();  // 字符串转整数
            _hasNew = true;             // 标记有新数据
        }
    }
}

// 发送触发指令，启动一次扫码
// 指令: 7E 00 08 01 00 02 01 AB CD（写标志位 0x0002 bit0=1 触发扫描）
void QRScanner::triggerScan() {
    flush();  // 清除旧数据，包括 ACK 回应
    uint8_t cmd[] = {0x7E, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, 0xAB, 0xCD};
    QR_SERIAL.write(cmd, sizeof(cmd));
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

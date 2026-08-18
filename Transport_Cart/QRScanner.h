// 二维码扫描模块
// 通过 Serial3 与二维码模块通信 (9600bps)，非阻塞异步读取
// 模块为自动输出模式：每次扫码成功自动发送 内容ASCII + 回车(0x0D)
//   例：扫"1"  → 0x31 0x0D
//       扫"5"  → 0x35 0x0D
//       扫"12" → 0x31 0x32 0x0D   （10~16 为两位 ASCII）
// 本模块按 CR/LF 分帧缓冲，帧结束解析为整数（1~16）

#ifndef QR_SCANNER_H
#define QR_SCANNER_H

#include <Arduino.h>
#include "Config.h"

class QRScanner {
public:
    QRScanner();

    // 初始化串口
    void init();

    // 非阻塞更新: 从串口读取字节并按帧解析
    // 必须在 loop() 中反复调用
    void update();

    // 获取最近一次解析成功的值（0 = 无数据）
    int result() const { return _result; }

    // 是否有新的扫码结果可用
    // 读取后自动清除标记和缓冲区
    bool available();

    // 清空串口缓冲区和内部状态
    void flush();

private:
    String _buffer;              // 当前帧数据缓冲区
    int _result;                 // 解析后的整数值
    bool _hasNew;                // 是否有新数据可用标记
    unsigned long _lastByteTime; // 最后一个字节的到达时间 (micros)

    // 半帧超时阈值: 收到不完整帧且长时间无结束符则丢弃（防串口噪声卡死）
    static const unsigned long FRAME_TIMEOUT_US = 500000;
    // 单帧最大长度（"16" + CR 足够，防异常数据撑爆缓冲区）
    static const uint8_t MAX_FRAME_LEN = 8;
};

// 全局扫描器实例
extern QRScanner Scanner;

// 全局初始化
void QRScanner_Init();

#endif

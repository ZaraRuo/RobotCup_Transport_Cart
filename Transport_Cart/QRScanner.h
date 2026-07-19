// 二维码扫描模块
// 通过 Serial3 与二维码模块通信，非阻塞异步读取
// 模块持续发送扫码结果字符串，本模块缓冲并解析为整数

#ifndef QR_SCANNER_H
#define QR_SCANNER_H

#include <Arduino.h>
#include "Config.h"

class QRScanner {
public:
    QRScanner();

    // 初始化串口
    void init();

    // 非阻塞更新: 从串口缓冲区读取并累积数据
    // 必须在 loop() 中反复调用
    void update();

    // 发送触发指令，启动一次扫码
    // 发送前会自动清空缓冲区
    void triggerScan();

    // 获取最近一次解码成功的值（0 = 无数据）
    int result() const { return _result; }

    // 获取原始接收字符串
    const String& rawString() const { return _buffer; }

    // 是否有新的扫码结果可用
    // 读取后自动清除标记和缓冲区
    bool available();

    // 清空串口缓冲区和内部状态
    void flush();

private:
    String _buffer;          // 接收数据缓冲区
    int _result;             // 解析后的整数值
    bool _hasNew;            // 是否有新数据可用标记
    unsigned long _lastByteTime;  // 最后一个字节的到达时间 (micros)

    // 超时阈值: 200ms 内无新字节则判定本次传输结束
    static const unsigned long READ_TIMEOUT_US = 200000;
};

// 全局扫描器实例
extern QRScanner Scanner;

// 全局初始化
void QRScanner_Init();

#endif

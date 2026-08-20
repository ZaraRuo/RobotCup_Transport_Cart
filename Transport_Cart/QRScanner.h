// 二维码扫描模块
// 通过 Serial3 与二维码模块通信 (9600bps)，非阻塞异步读取
// 模块为自动输出模式：每次扫码成功自动发送 内容ASCII + 回车(0x0D)
//   例：扫"1"  → 0x31 0x0D
//       扫"5"  → 0x35 0x0D
//       扫"12" → 0x31 0x32 0x0D   （10~16 为两位 ASCII）
//
// 接收逻辑刻意保持最简单（吸取教训：复杂的防错机制会误清有效数据）：
//   - 逐字节累积到固定字符缓冲
//   - 仅在收到 CR/LF 结束符时才清空缓冲并解析一帧
//   - 无超时清空、无长度溢出清空等中途清空逻辑
//   - 帧内容用 atoi 解析为整数（1~16），非法内容得 0，由调用方过滤
//   - 本模块不做任何串口/调试输出；如需观察原始字节（如调试阶段打印 HEX），
//     由调用方通过 setByteListener() 注册回调实现

#ifndef QR_SCANNER_H
#define QR_SCANNER_H

#include <Arduino.h>
#include "Config.h"

class QRScanner {
public:
    QRScanner();

    // 初始化串口
    void init();

    // 非阻塞更新: 从串口读取字节并按帧解析，必须在 loop() 中反复调用
    void update();

    // 最近一次解析成功的值（0 = 无数据 / 非法帧）
    int result() const { return _result; }

    // 是否有新的扫码结果可用（读取后自动清除标记）
    bool available();

    // 清空串口缓冲区和内部状态
    void flush();

    // 注册字节监听回调（调试用）：每收到一个原始字节就调用一次，参数为字节值。
    // 回调由调用方实现（例如在调试阶段里打印 HEX），本模块自身不做任何输出。
    // 传 NULL 可取消监听。
    void setByteListener(void (*listener)(uint8_t));

private:
    // 单帧最大字节数（"16" + 结束符足够；超长时只忽略多余字符，不清空已有数据）
    // 注意: 必须声明在 _buf 之前（类内成员按声明顺序解析）
    static const uint8_t QR_MAX_FRAME_LEN = 8;

    char _buf[QR_MAX_FRAME_LEN + 1];  // 帧缓冲，仅在结束符处清空
    uint8_t _bufLen;                  // 当前帧有效长度
    int _result;                      // 解析结果
    bool _hasNew;                     // 新结果标记
    void (*_byteListener)(uint8_t);   // 原始字节回调（可为 NULL）
};

// 全局扫描器实例
extern QRScanner Scanner;

// 全局初始化
void QRScanner_Init();

#endif

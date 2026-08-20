// ============================================================================
//  搬运机器人 — 主入口
//  包含: 调试模式配置、模块初始化、分阶段调试测试
//
//  使用方法:
//    1. 修改 MAIN_DEBUG 和 DEBUG_STAGE 选择调试阶段
//    2. 编译后烧录到 Arduino Mega 2560
//    3. 观察串口监视器 (115200bps) 输出
//    4. DEBUG_MOTOR_BYPASS=1 可在脱离电机时验证逻辑
// ============================================================================

// ============================================================================
//  调试模式配置
// ============================================================================
//  MAIN_DEBUG=1   -> 调试模式
//    DEBUG_STAGE=1: 推杆基础测试 (伸到顶/伸到底, 各10s)
//    DEBUG_STAGE=2: 灰度传感器读数 (12路原始值 + 偏移量)
//    DEBUG_STAGE=3: 步进电机单路测试 (左轮/右轮, 正转/反转)
//    DEBUG_STAGE=4: 底盘直行测试 (前进300mm/后退150mm/旋转±90°)
//    DEBUG_STAGE=5: 底盘前向循迹 (前进循迹)
//    DEBUG_STAGE=6: 底盘后向循迹 (后退循迹)
//    DEBUG_STAGE=7: 二维码扫描测试 (轮询接收 + 打印结果)
//    DEBUG_STAGE=8: 原地旋转测试 (45/90/180度 CW+CCW, 标定用)
//    DEBUG_STAGE=9: 转盘 + 仓位管理测试 (72°旋转 + 仓位占用/定位)
//    DEBUG_STAGE=10: 颜色传感器测试 (TCS230, 实时识别 + 串口白平衡)
//    DEBUG_STAGE=11: 转盘+颜色识别联动测试 (车头仓转180°识别, 任务1流程)
//    DEBUG_STAGE=12: 任务流程分段测试 (复用正式流程同一套代码)
//  MAIN_DEBUG=0   -> 完整工作模式 (后续添加任务流程)
// ============================================================================
#define MAIN_DEBUG           1
#define DEBUG_STAGE          7

// 电机旁路模式（纯算法调试，不驱动实际电机）
// DEBUG_MOTOR_BYPASS=1: 所有电机控制指令只打印日志，不输出脉冲
#define DEBUG_MOTOR_BYPASS   0

// ============================================================================
//  系统头文件
// ============================================================================
#include <Arduino.h>
#include "Config.h"
#include "Stepper.h"
#include "LineSensor.h"
#include "Chassis.h"
#include "PushRod.h"
#include "QRScanner.h"
#include "Turntable.h"
#include "ColorSensor.h"
#include "TaskFlow.h"

// ============================================================================
//  电机旁路宏 — 当 DEBUG_MOTOR_BYPASS=1 时生效
//  用日志替代实际电机脉冲输出，适合在没有硬件时验证控制逻辑
// ============================================================================
#if DEBUG_MOTOR_BYPASS
#define MOTOR_BYPASS_PREFIX "[MOTOR_BYPASS] "

// 直行旁路日志
static void MotorBypass_LogMove(const char *desc, long steps, unsigned long us) {
    DEBUG_SERIAL.print(MOTOR_BYPASS_PREFIX);
    DEBUG_SERIAL.print(desc);
    DEBUG_SERIAL.print(" steps=");
    DEBUG_SERIAL.print(steps);
    DEBUG_SERIAL.print(" us=");
    DEBUG_SERIAL.println(us);
}

// 差分驱动旁路日志
static void MotorBypass_LogDiff(const char *desc, long stepsL, long stepsR, unsigned long usL, unsigned long usR) {
    DEBUG_SERIAL.print(MOTOR_BYPASS_PREFIX);
    DEBUG_SERIAL.print(desc);
    DEBUG_SERIAL.print(" L=");
    DEBUG_SERIAL.print(stepsL);
    DEBUG_SERIAL.print("(");
    DEBUG_SERIAL.print(usL);
    DEBUG_SERIAL.print("us) R=");
    DEBUG_SERIAL.print(stepsR);
    DEBUG_SERIAL.print("(");
    DEBUG_SERIAL.print(usR);
    DEBUG_SERIAL.println("us)");
}
#endif

// ============================================================================
//  工具函数
// ============================================================================

// 开机信息打印: 项目名、主控型号
void printBootInfo() {
    DEBUG_SERIAL.println(F("=================================="));
    DEBUG_SERIAL.println(F("  Transport Cart v1.0"));
    DEBUG_SERIAL.println(F("  Board: Arduino Mega 2560"));
    DEBUG_SERIAL.println(F("=================================="));
}

// 打印一行传感器完整状态: 前8路 + 后4路 + 偏移量
// 用于 Stage 2 实时滚动观察
void printSensorRow(uint32_t tick) {
    LineSensors.read();

    char buf[80];
    int offF = (int)LineSensors.frontOffset();
    int offB = (int)LineSensors.backOffset();

    snprintf(buf, sizeof(buf),
        "%lu: %d%d%d%d%d%d%d%d | %d%d%d%d | F%+d B%+d",
        tick,
        LineSensors.front(0), LineSensors.front(1),
        LineSensors.front(2), LineSensors.front(3),
        LineSensors.front(4), LineSensors.front(5),
        LineSensors.front(6), LineSensors.front(7),
        LineSensors.back(0), LineSensors.back(1),
        LineSensors.back(2), LineSensors.back(3),
        offF, offB);
    DEBUG_SERIAL.println(buf);
}

// ============================================================================
//  调试辅助: 单电机测试
//  仅在 Stage 3 及以上编译，避免 Stage 1/2 中引用未定义的电机变量
// ============================================================================
#if MAIN_DEBUG && DEBUG_STAGE >= 3
// 对指定电机执行 +3000 步前进 → 500ms 暂停 → -3000 步后退
// 用于验证单个电机的接线、方向和基本功能
static void _motorTestSingle(const char *name, Stepper &motor) {
    motor.reset();
#if DEBUG_MOTOR_BYPASS
    for (int t = 0; t < 3000; t++) {
        motor.runCycle(true, 200);
        delay(1);
    }
#else
    while (!motor.runTo(3000, 200)) { }
#endif
    delay(500);

    motor.reset();
#if DEBUG_MOTOR_BYPASS
    for (int t = 0; t < 3000; t++) {
        motor.runCycle(false, 200);
        delay(1);
    }
#else
    while (!motor.runTo(-3000, 200)) { }
#endif
    delay(500);
}
#endif

// ============================================================================
//  Stage 7 辅助函数（仅 DEBUG_STAGE==7 时编译）
//  调试输出全部放在调试阶段实现，功能模块(QRScanner)内不做任何串口打印
// ============================================================================
#if MAIN_DEBUG && DEBUG_STAGE == 7
// 字节监听回调: 打印 Serial3 收到的原始字节 (HEX)，由 Scanner 每字节回调
// 仅为观察，不改变任何接收/解析逻辑（与生产走同一段代码）
static void qrByteEcho(uint8_t b) {
    DEBUG_SERIAL.print(F("[RX3] 0x"));
    if (b < 0x10) DEBUG_SERIAL.print('0');
    DEBUG_SERIAL.println(b, HEX);
}
#endif

// 颜色码转名称（Stage 10/11 用）
#if MAIN_DEBUG && (DEBUG_STAGE == 10 || DEBUG_STAGE == 11)
static const char *colorNameStr(uint8_t col) {
    switch (col) {
    case ColorSensor::COLOR_NONE:  return "NONE";
    case ColorSensor::COLOR_GREEN: return "GREEN";
    case ColorSensor::COLOR_WHITE: return "WHITE";
    case ColorSensor::COLOR_RED:   return "RED";
    case ColorSensor::COLOR_BLACK: return "BLACK";
    case ColorSensor::COLOR_BLUE:  return "BLUE";
    default:                       return "?";
    }
}
#endif

// ============================================================================
//  setup() — 系统初始化入口
//  执行顺序: 串口 → 各模块初始化 → 打印模式信息
// ============================================================================
void setup() {
    // 初始化调试串口
    DEBUG_SERIAL.begin(DEBUG_BAUD_RATE);
    delay(500);
    printBootInfo();

    // 依序初始化所有硬件模块
    Stepper_Init();      // 步进电机（左右）
    LineSensor_Init();   // 灰度循迹传感器
    PushRod_Init();      // 电动推杆
    QRScanner_Init();    // 二维码扫描
    Turntable_Init();    // 转盘步进电机
    ColorSensor_Init();  // TCS230 颜色传感器
    Chassis_Init();      // 底盘状态机

    DEBUG_SERIAL.println(F("[OK] All modules initialized."));

#if MAIN_DEBUG
    DEBUG_SERIAL.print(F("[MODE] Debug, Stage="));
    DEBUG_SERIAL.println(DEBUG_STAGE);
#if DEBUG_MOTOR_BYPASS
    DEBUG_SERIAL.println(F("[MODE] Motor Bypass ON (no actual motor movement)"));
#endif
#else
    DEBUG_SERIAL.println(F("[MODE] Normal"));
    TaskFlow_Init();
#endif
}

// ============================================================================
//  loop() — 主循环
//  调试模式: 根据 DEBUG_STAGE 执行对应的测试流程
//  正常模式: 运行底盘和扫码器更新（后续添加任务流程）
// ============================================================================
void loop() {
#if MAIN_DEBUG

    // ==================================================================
    //  Stage 1: 推杆基础测试
    //  验证: 推杆伸到顶/伸到底的完整行程与运行时间(10s)
    //  流程: 伸到顶(10s) → 停1s → 伸到底(10s) → 停1s，重复2次
    // ==================================================================
#if DEBUG_STAGE == 1
    {
        for (int i = 0; i < 2; i++) {
            Rod.extendToTop();
            delay(1000);
            Rod.retractToBottom();
            delay(1000);
        }
        while (1) { delay(1000); }
    }

    // ==================================================================
    //  Stage 2: 灰度传感器读数
    //  验证: 12路传感器返回值、黑/白阈值、偏移量计算
    //  操作: 手动将车放在黑线上方，观察哪些传感器读到 BLACK
    //  输出: 每 200ms 一行，包含所有原始值 + 偏移量
    // ==================================================================
#elif DEBUG_STAGE == 2
    {
        DEBUG_SERIAL.println(F("\r\n========================================"));
        DEBUG_SERIAL.println(F("  Stage 2: Line Sensor Readout"));
        DEBUG_SERIAL.println(F("========================================"));
        DEBUG_SERIAL.println(F("  F: front 8ch  B: back 4ch"));
        DEBUG_SERIAL.println(F("  1=Black 0=White  Off=Offset"));
        DEBUG_SERIAL.println(F("  [ACTION] Move chassis over line, observe"));
        DEBUG_SERIAL.println(F("========================================\r\n"));

        uint32_t tick = 0;
        while (1) {
            delay(500);
            tick++;
            printSensorRow(tick);
        }
    }

    // ==================================================================
    //  Stage 3: 步进电机单路测试
    //  验证: 左右电机独立正转/反转，判断接线和方向是否正确
    //  流程: 左轮正转3000→反转3000 | 右轮正转3000→反转3000
    // ==================================================================
#elif DEBUG_STAGE == 3
    {
        _motorTestSingle("L", MotorL);
        _motorTestSingle("R", MotorR);
        while (1) { delay(1000); }
    }

    // ==================================================================
    //  Stage 4: 底盘直行测试 (不循迹)
    //  验证: 左右轮同步性、距离换算精度
    //  流程: 前进300mm → 后退150mm → 右转90° → 左转90°
    // ==================================================================
#elif DEBUG_STAGE == 4
    {
#if DEBUG_MOTOR_BYPASS
        MotorBypass_LogMove("Forward 300mm", (long)(300 * WHEEL_STEPS_PER_MM), 160);
#else
        Cart.moveMm(300, 160);
        while (!Cart.isDone()) { Cart.update(); }
#endif
        delay(1000);

#if DEBUG_MOTOR_BYPASS
        MotorBypass_LogMove("Backward 150mm", (long)(-150 * WHEEL_STEPS_PER_MM), 240);
#else
        Cart.moveMm(-150, 240);
        while (!Cart.isDone()) { Cart.update(); }
#endif
        delay(1000);

        long rotSteps = (long)(PI * WHEEL_BASE_MM * 90.0f / 360.0f * WHEEL_STEPS_PER_MM);

#if DEBUG_MOTOR_BYPASS
        MotorBypass_LogDiff("Rotate CW 90deg", -rotSteps, rotSteps, 160, 160);
#else
        Cart.rotateAngle(-90, 160);
        while (!Cart.isDone()) { Cart.update(); }
#endif
        delay(1000);

#if DEBUG_MOTOR_BYPASS
        MotorBypass_LogDiff("Rotate CCW 90deg", rotSteps, -rotSteps, 160, 160);
#else
        Cart.rotateAngle(90, 160);
        while (!Cart.isDone()) { Cart.update(); }
#endif

        while (1) { delay(1000); }
    }

    // ==================================================================
    //  Stage 5: 底盘前向循迹
    //  验证: 前侧8路传感器 + 循迹控制参数
    //  操作: 将车放在黑线起点，车自动循迹
    //  逻辑: 线偏→同侧轮减速→车转向修正
    // ==================================================================
#elif DEBUG_STAGE == 5
    {
#if DEBUG_MOTOR_BYPASS
        MotorBypass_LogMove("TrackForward(300000)", 300000, 0);
#else
        Cart.trackForward(300000, 80);
        while (!Cart.isDone()) { Cart.update(); }
#endif
        while (1) { delay(1000); }
    }

    // ==================================================================
    //  Stage 6: 底盘后向循迹
    //  验证: 后侧4路传感器 + 后向循迹逻辑
    //  操作: 将车反向放在黑线上，观察后退循迹效果
    //  逻辑: 与前向共用统一修正公式，后退时左右对调
    // ==================================================================
#elif DEBUG_STAGE == 6
    {
#if DEBUG_MOTOR_BYPASS
        MotorBypass_LogMove("TrackBackward(10000)", 10000, 0);
#else
        Cart.trackBackward(10000, 80);
        while (!Cart.isDone()) { Cart.update(); }
#endif
        while (1) { delay(1000); }
    }

    // ==================================================================
    //  Stage 7: 二维码扫描测试
    //  验证: Serial3 通讯、模块输出格式、数据解析
    //  接收走与正常模式完全相同的代码路径: Scanner.update() 读 Serial3，
    //  输出: 每个原始字节 [RX3] 0xXX (仅观察) + 每帧 [QR]: 数字
    // ==================================================================
#elif DEBUG_STAGE == 7
    {
        DEBUG_SERIAL.println(F("\r\n========================================"));
        DEBUG_SERIAL.println(F("  Stage 7: QR Code Scan"));
        DEBUG_SERIAL.println(F("========================================\r\n"));

        // 注册字节监听回调（打印原始字节 HEX）——调试输出全部在调试阶段实现
        Scanner.setByteListener(qrByteEcho);

        while (1) {
            Scanner.update();

            if (Scanner.available()) {
                DEBUG_SERIAL.print(F("[QR]: "));
                DEBUG_SERIAL.println(Scanner.result());
            }
        }
    }

    // ==================================================================
    //  Stage 8: 原地旋转测试
    //  验证: rotateAngle 角度精度，标定 WHEEL_BASE_MM
    //  流程: 每组都是 CCW 转出 → CW 转回原位
    //    45° → 回 → 90° → 回 → 180° → 回
    //  标定方法:
    //    转过量(角度大于目标) → 增大 WHEEL_BASE_MM
    //    转不够(角度小于目标) → 减小 WHEEL_BASE_MM
    // ==================================================================
#elif DEBUG_STAGE == 8
    {
#if DEBUG_MOTOR_BYPASS
        DEBUG_SERIAL.print(MOTOR_BYPASS_PREFIX);
        DEBUG_SERIAL.println(F("rotateAngle(45, 160)"));
#else
        Cart.rotateAngle(45, 160);
        while (!Cart.isDone()) { Cart.update(); }
#endif
        delay(2000);

#if DEBUG_MOTOR_BYPASS
        DEBUG_SERIAL.print(MOTOR_BYPASS_PREFIX);
        DEBUG_SERIAL.println(F("rotateAngle(-45, 160)"));
#else
        Cart.rotateAngle(-45, 160);
        while (!Cart.isDone()) { Cart.update(); }
#endif
        delay(2000);

#if DEBUG_MOTOR_BYPASS
        DEBUG_SERIAL.print(MOTOR_BYPASS_PREFIX);
        DEBUG_SERIAL.println(F("rotateAngle(90, 160)"));
#else
        Cart.rotateAngle(90, 160);
        while (!Cart.isDone()) { Cart.update(); }
#endif
        delay(2000);

#if DEBUG_MOTOR_BYPASS
        DEBUG_SERIAL.print(MOTOR_BYPASS_PREFIX);
        DEBUG_SERIAL.println(F("rotateAngle(-90, 160)"));
#else
        Cart.rotateAngle(-90, 160);
        while (!Cart.isDone()) { Cart.update(); }
#endif
        delay(2000);

#if DEBUG_MOTOR_BYPASS
        DEBUG_SERIAL.print(MOTOR_BYPASS_PREFIX);
        DEBUG_SERIAL.println(F("rotateAngle(180, 160)"));
#else
        Cart.rotateAngle(180, 160);
        while (!Cart.isDone()) { Cart.update(); }
#endif
        delay(2000);

#if DEBUG_MOTOR_BYPASS
        DEBUG_SERIAL.print(MOTOR_BYPASS_PREFIX);
        DEBUG_SERIAL.println(F("rotateAngle(-180, 160)"));
#else
        Cart.rotateAngle(-180, 160);
        while (!Cart.isDone()) { Cart.update(); }
#endif

        while (1) { delay(1000); }
    }

    // ==================================================================
    //  Stage 9: 转盘 + 仓位管理测试
    //  验证: 72°旋转精度、仓位索引推进、占用标记、rotateToCell 最短路径
    //  流程:
    //    1) 传统 72/144/216° CCW→CW 往返（旋转精度）
    //    2) 仓位演示: 依次"占用车头仓→转一格"，共 5 次，打印各仓占用
    //    3) rotateToCell(2) 最短路径定位到 2 号仓
    //  说明: 上电后 0 号仓应对准车头开口；若转向相反需调整电机接线/换向约定
    // ==================================================================
#elif DEBUG_STAGE == 9
    {
        // ---- 1) 旋转精度往返 ----
        TurntableMotor.rotateMultiples(1, 200);
        while (!TurntableMotor.isDone()) { TurntableMotor.update(); }
        delay(1000);
        TurntableMotor.rotateMultiples(-1, 200);
        while (!TurntableMotor.isDone()) { TurntableMotor.update(); }
        delay(1000);

        TurntableMotor.rotateMultiples(2, 200);
        while (!TurntableMotor.isDone()) { TurntableMotor.update(); }
        delay(1000);
        TurntableMotor.rotateMultiples(-2, 200);
        while (!TurntableMotor.isDone()) { TurntableMotor.update(); }
        delay(1000);

        TurntableMotor.rotateMultiples(3, 200);
        while (!TurntableMotor.isDone()) { TurntableMotor.update(); }
        delay(1000);
        TurntableMotor.rotateMultiples(-3, 200);
        while (!TurntableMotor.isDone()) { TurntableMotor.update(); }
        delay(1000);

        // ---- 2) 仓位管理演示: 依次占用车头仓并转一格 ----
        DEBUG_SERIAL.println(F("[TT] cell demo: occupy front -> rotate 1 cell x5"));
        for (uint8_t i = 0; i < Turntable::CELL_COUNT; i++) {
            TurntableMotor.occupyFront();   // 模拟收入一个物块
            DEBUG_SERIAL.print(F("[TT] occupied cell "));
            DEBUG_SERIAL.print(TurntableMotor.frontCell());
            DEBUG_SERIAL.print(F(", front now "));
            TurntableMotor.rotateMultiples(1, 200);
            while (!TurntableMotor.isDone()) { TurntableMotor.update(); }
            DEBUG_SERIAL.println(TurntableMotor.frontCell());
        }
        // 打印各仓占用
        DEBUG_SERIAL.print(F("[TT] cells:"));
        for (uint8_t c = 0; c < Turntable::CELL_COUNT; c++) {
            DEBUG_SERIAL.print(F(" "));
            DEBUG_SERIAL.print(c);
            DEBUG_SERIAL.print(TurntableMotor.isOccupied(c) ? F("(X)") : F("(.)"));
        }
        DEBUG_SERIAL.println();
        delay(1500);

        // ---- 3) rotateToCell 最短路径 ----
        DEBUG_SERIAL.print(F("[TT] rotateToCell(2), front="));
        DEBUG_SERIAL.print(TurntableMotor.frontCell());
        TurntableMotor.rotateToCell(2, 200);
        while (!TurntableMotor.isDone()) { TurntableMotor.update(); }
        DEBUG_SERIAL.print(F(" -> "));
        DEBUG_SERIAL.println(TurntableMotor.frontCell());

        while (1) { delay(1000); }
    }

    // ==================================================================
    //  Stage 10: TCS230 颜色传感器测试
    //  验证: 传感器接线、白平衡标定、五色识别准确性
    //  操作: 将传感器探头贴近物料表面（避免环境光直射）
    //  输出: 每轮(约 4×100ms)打印一次 C/R/G/B 原始计数值 +
    //        白平衡归一化值 + 识别颜色
    //  指令: 串口发送 'w' 执行白平衡（先把传感器对准白色表面再发送）
    // ==================================================================
#elif DEBUG_STAGE == 10
    {
        DEBUG_SERIAL.println(F("\r\n========================================"));
        DEBUG_SERIAL.println(F("  Stage 10: TCS230 Color Sensor"));
        DEBUG_SERIAL.println(F("========================================"));
        DEBUG_SERIAL.println(F("  [ACTION] Hold sensor close to a colored block"));
        DEBUG_SERIAL.println(F("  [CMD] 'w' = white balance over WHITE surface"));
        DEBUG_SERIAL.println(F("  Output: raw counts + normalized RGB + color"));
        DEBUG_SERIAL.println(F("========================================\r\n"));

        ColorSensors.begin();

        while (1) {
            ColorSensors.update();

            if (ColorSensors.available()) {
                const char *name;
                switch (ColorSensors.color()) {
                case ColorSensor::COLOR_NONE:  name = "NONE";  break;
                case ColorSensor::COLOR_GREEN: name = "GREEN"; break;
                case ColorSensor::COLOR_WHITE: name = "WHITE"; break;
                case ColorSensor::COLOR_RED:   name = "RED";   break;
                case ColorSensor::COLOR_BLACK: name = "BLACK"; break;
                case ColorSensor::COLOR_BLUE:  name = "BLUE";  break;
                default:                       name = "?";     break;
                }

                DEBUG_SERIAL.print(F("  C="));
                DEBUG_SERIAL.print(ColorSensors.rawClear());
                DEBUG_SERIAL.print(F(" R="));
                DEBUG_SERIAL.print(ColorSensors.rawRed());
                DEBUG_SERIAL.print(F(" G="));
                DEBUG_SERIAL.print(ColorSensors.rawGreen());
                DEBUG_SERIAL.print(F(" B="));
                DEBUG_SERIAL.print(ColorSensors.rawBlue());
                DEBUG_SERIAL.print(F(" | nR="));
                DEBUG_SERIAL.print((int)ColorSensors.normRed());
                DEBUG_SERIAL.print(F(" nG="));
                DEBUG_SERIAL.print((int)ColorSensors.normGreen());
                DEBUG_SERIAL.print(F(" nB="));
                DEBUG_SERIAL.print((int)ColorSensors.normBlue());
                DEBUG_SERIAL.print(F(" | color="));
                DEBUG_SERIAL.println(name);

                ColorSensors.begin();   // 开始下一轮测量
            }

            // 串口指令: 'w' 触发白平衡标定（传感器须对准白色表面）
            if (DEBUG_SERIAL.available()) {
                char c = (char)DEBUG_SERIAL.read();
                if (c == 'w' || c == 'W') {
                    DEBUG_SERIAL.println(F("[WB] Hold sensor over WHITE, measuring..."));
                    ColorSensors.beginWhiteBalance();
                }
            }
        }
    }

    // ==================================================================
    //  Stage 11: 转盘 + 颜色识别联动测试（任务1流程之一）
    //  验证: 车头仓物块转 180° 到颜色识别模块处识别，记录颜色到该仓
    //  操作: 先把一个已知颜色的物块放入车头开口仓位
    //  指令: 'd' 识别车头仓（转180°→测量→记录）
    //        's' 依次扫描所有已占用仓位并记录颜色
    //        'w' 白平衡（传感器对准白色表面）
    //        'p' 打印各仓位占用/颜色
    //  注意: 识别前请先做白平衡('w')，否则阈值可能不准
    // ==================================================================
#elif DEBUG_STAGE == 11
    {
        DEBUG_SERIAL.println(F("\r\n========================================"));
        DEBUG_SERIAL.println(F("  Stage 11: Turntable + Color Sensor"));
        DEBUG_SERIAL.println(F("========================================"));
        DEBUG_SERIAL.println(F("  [1] put a colored block in FRONT cell"));
        DEBUG_SERIAL.println(F("  [CMD] 'd'=detect front cell (rotate 180deg)"));
        DEBUG_SERIAL.println(F("        's'=scan all occupied cells"));
        DEBUG_SERIAL.println(F("        'w'=white balance over WHITE"));
        DEBUG_SERIAL.println(F("        'p'=print cell status"));
        DEBUG_SERIAL.println(F("========================================\r\n"));

        while (1) {
            TurntableMotor.update();
            ColorSensors.update();

            if (DEBUG_SERIAL.available()) {
                char c = (char)DEBUG_SERIAL.read();

                if (c == 'd' || c == 'D') {
                    // 车头仓 → 转180°到识别模块 → 识别 → 记录颜色到该仓
                    uint8_t cell = TurntableMotor.frontCell();
                    DEBUG_SERIAL.print(F("[T1] detect front cell "));
                    DEBUG_SERIAL.println(cell);
                    TurntableMotor.rotateToSensor(cell, 200);
                    while (!TurntableMotor.isDone()) {
                        TurntableMotor.update();
                        ColorSensors.update();
                    }

                    ColorSensors.begin();
                    unsigned long t0 = millis();
                    while (!ColorSensors.available() && (millis() - t0) < 3000UL) {
                        ColorSensors.update();
                        TurntableMotor.update();
                    }
                    if (ColorSensors.available()) {
                        uint8_t col = (uint8_t)ColorSensors.color();
                        TurntableMotor.setCellColor(cell, col);
                        DEBUG_SERIAL.print(F("[T1] cell "));
                        DEBUG_SERIAL.print(cell);
                        DEBUG_SERIAL.print(F(" color="));
                        DEBUG_SERIAL.println(colorNameStr(col));
                    } else {
                        DEBUG_SERIAL.println(F("[T1] color measure timeout"));
                    }
                } else if (c == 's' || c == 'S') {
                    // 依次扫描所有已占用仓位并记录颜色
                    for (uint8_t cell = 0; cell < Turntable::CELL_COUNT; cell++) {
                        if (!TurntableMotor.isOccupied(cell)) continue;
                        TurntableMotor.rotateToSensor(cell, 200);
                        while (!TurntableMotor.isDone()) {
                            TurntableMotor.update();
                            ColorSensors.update();
                        }
                        ColorSensors.begin();
                        unsigned long t0 = millis();
                        while (!ColorSensors.available() && (millis() - t0) < 3000UL) {
                            ColorSensors.update();
                            TurntableMotor.update();
                        }
                        if (ColorSensors.available()) {
                            uint8_t col = (uint8_t)ColorSensors.color();
                            TurntableMotor.setCellColor(cell, col);
                            DEBUG_SERIAL.print(F("[T1] cell "));
                            DEBUG_SERIAL.print(cell);
                            DEBUG_SERIAL.print(F(" = "));
                            DEBUG_SERIAL.println(colorNameStr(col));
                        } else {
                            DEBUG_SERIAL.print(F("[T1] cell "));
                            DEBUG_SERIAL.print(cell);
                            DEBUG_SERIAL.println(F(" timeout"));
                        }
                    }
                    DEBUG_SERIAL.println(F("[T1] scan done"));
                } else if (c == 'w' || c == 'W') {
                    DEBUG_SERIAL.println(F("[WB] Hold sensor over WHITE, measuring..."));
                    ColorSensors.beginWhiteBalance();
                } else if (c == 'p' || c == 'P') {
                    DEBUG_SERIAL.print(F("[TT] front="));
                    DEBUG_SERIAL.print(TurntableMotor.frontCell());
                    DEBUG_SERIAL.print(F(" cells:"));
                    for (uint8_t cc = 0; cc < Turntable::CELL_COUNT; cc++) {
                        DEBUG_SERIAL.print(F(" "));
                        DEBUG_SERIAL.print(cc);
                        DEBUG_SERIAL.print(TurntableMotor.isOccupied(cc)
                            ? F(":") : F(":."));
                        if (TurntableMotor.isOccupied(cc)) {
                            DEBUG_SERIAL.print(colorNameStr(TurntableMotor.cellColor(cc)));
                        }
                    }
                    DEBUG_SERIAL.println();
                }
            }
        }
    }

    // ==================================================================
    //  Stage 12: 任务流程分段测试（复用正式流程的同一套代码）
    //  测试方法与正式流程完全相同——测试通过即证明完整流程对应部分可用
    //  指令:
    //    a = T2_LeaveHome()        b = T2_MoveToQRZone()
    //    c = T2_MoveToPodium()     （自动注入方案1/仓位 A0 B1 C2）
    //    d = T1_MoveToQRZone()     e = T1_PlaceAllBlocks()
    //        （自动注入方案1/各仓颜色 黑白红绿蓝）
    //    f = T1_ReturnHome()
    //    g = 任务2连续收料阶段（真实状态机，收满3块+36°遮蔽后自动停）
    //    h = 任务1连续收料阶段（真实状态机，收满5块后自动停）
    //    i = 任务1逐仓颜色识别阶段（识别完自动停）
    //    j = 任务1 36°遮蔽阶段（转完自动停）
    //  阶段测试(g~j)期间请持续观察串口日志，阶段结束 _state 回 IDLE
    // ==================================================================
#elif DEBUG_STAGE == 12
    {
        DEBUG_SERIAL.println(F("\r\n========================================"));
        DEBUG_SERIAL.println(F("  Stage 12: TaskFlow Segment Test"));
        DEBUG_SERIAL.println(F("  Same code as formal flow (T2->T1)"));
        DEBUG_SERIAL.println(F("========================================"));
        DEBUG_SERIAL.println(F("  a=T2_LeaveHome   b=T2_MoveToQRZone"));
        DEBUG_SERIAL.println(F("  c=T2_MoveToPodium(plan1,A0B1C2)"));
        DEBUG_SERIAL.println(F("  d=T1_MoveToQRZone"));
        DEBUG_SERIAL.println(F("  e=T1_PlaceAllBlocks(plan1,BWRGB)"));
        DEBUG_SERIAL.println(F("  f=T1_ReturnHome"));
        DEBUG_SERIAL.println(F("  g=T2 collect    h=T1 collect"));
        DEBUG_SERIAL.println(F("  i=T1 detect     j=T1 shield36"));
        DEBUG_SERIAL.println(F("========================================\r\n"));

        while (1) {
            // 驱动流程状态机: 阶段测试进行时推进；空闲时仅泵动各模块
            TaskFlow_Update();

            if (DEBUG_SERIAL.available()) {
                char c = (char)DEBUG_SERIAL.read();
                switch (c) {
                case 'a':
                    DEBUG_SERIAL.println(F("[TEST] T2_LeaveHome()"));
                    T2_LeaveHome();
                    break;
                case 'b':
                    DEBUG_SERIAL.println(F("[TEST] T2_MoveToQRZone()"));
                    T2_MoveToQRZone();
                    break;
                case 'c':
                    DEBUG_SERIAL.println(F("[TEST] T2_MoveToPodium() plan=1 cells A0 B1 C2"));
                    TaskFlow_DebugSetT2Place(1, 0, 1, 2);   // 注入模拟数据
                    T2_MoveToPodium(0);                     // 与正式流程同一函数
                    break;
                case 'd':
                    DEBUG_SERIAL.println(F("[TEST] T1_MoveToQRZone()"));
                    T1_MoveToQRZone();
                    break;
                case 'e': {
                    uint8_t colors[5] = {
                        ColorSensor::COLOR_BLACK, ColorSensor::COLOR_WHITE,
                        ColorSensor::COLOR_RED,   ColorSensor::COLOR_GREEN,
                        ColorSensor::COLOR_BLUE
                    };
                    DEBUG_SERIAL.println(F("[TEST] T1_PlaceAllBlocks() plan=1 cells=黑白红绿蓝"));
                    TaskFlow_DebugSetT1Place(1, colors);    // 注入模拟数据
                    T1_PlaceAllBlocks();                    // 与正式流程同一函数
                    break;
                }
                case 'f':
                    DEBUG_SERIAL.println(F("[TEST] T1_ReturnHome()"));
                    T1_ReturnHome();
                    break;
                case 'g':
                    DEBUG_SERIAL.println(F("[TEST] T2 collect phase start (plan=1)"));
                    TaskFlow_DebugRunCollectT2(1);
                    break;
                case 'h':
                    DEBUG_SERIAL.println(F("[TEST] T1 collect phase start (plan=1)"));
                    TaskFlow_DebugRunCollectT1(1);
                    break;
                case 'i':
                    DEBUG_SERIAL.println(F("[TEST] T1 detect phase start"));
                    TaskFlow_DebugRunDetectT1();
                    break;
                case 'j':
                    DEBUG_SERIAL.println(F("[TEST] T1 shield(36deg) phase start"));
                    TaskFlow_DebugRunShieldT1();
                    break;
                default:
                    break;
                }
            }
        }
    }

#endif

#else
    // ==================================================================
    //  Normal Mode (MAIN_DEBUG=0)
    //  运行任务流程，流程内部自动调度各模块
    // ==================================================================
    TaskFlow_Update();
    delay(1);
#endif
}

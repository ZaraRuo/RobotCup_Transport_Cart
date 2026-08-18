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
//    DEBUG_STAGE=1: 推杆基础测试 (伸缩/脉冲)
//    DEBUG_STAGE=2: 灰度传感器读数 (12路原始值 + 偏移量)
//    DEBUG_STAGE=3: 步进电机单路测试 (左轮/右轮, 正转/反转)
//    DEBUG_STAGE=4: 底盘直行测试 (前进300mm/后退150mm/旋转±90°)
//    DEBUG_STAGE=5: 底盘前向循迹 (前进循迹)
//    DEBUG_STAGE=6: 底盘后向循迹 (后退循迹)
//    DEBUG_STAGE=7: 二维码扫描测试 (轮询接收 + 打印结果)
//    DEBUG_STAGE=8: 原地旋转测试 (45/90/180度 CW+CCW, 标定用)
//    DEBUG_STAGE=9: 转盘步进电机测试 (72/144/216度 CCW+CW)
//    DEBUG_STAGE=10: 颜色传感器测试 (TCS230, 实时识别 + 串口白平衡)
//  MAIN_DEBUG=0   -> 完整工作模式 (后续添加任务流程)
// ============================================================================
#define MAIN_DEBUG           1
#define DEBUG_STAGE          10

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
    //  验证: 推杆能否正常伸缩，物理行程是否正确
    //  流程: 伸2秒→缩1秒→脉冲500ms → 重复3次
    // ==================================================================
#if DEBUG_STAGE == 1
    {
        for (int i = 0; i < 3; i++) {
            Rod.extend();
            delay(2000);
            Rod.retract();
            delay(1000);
            Rod.pulse(500);
            delay(1500);
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
    //  操作: 将二维码放在模块前方，观察串口输出
    //  机制: 模块自动输出模式——扫码成功自动发送 内容ASCII+CR(0x0D)，
    //        Scanner.update() 按 CR/LF 分帧并解析为整数 (1~16)
    // ==================================================================
#elif DEBUG_STAGE == 7
    {
        DEBUG_SERIAL.println(F("\r\n========================================"));
        DEBUG_SERIAL.println(F("  Stage 7: QR Code Scan"));
        DEBUG_SERIAL.println(F("========================================"));
        DEBUG_SERIAL.println(F("  Auto mode: module sends content + CR on scan"));
        DEBUG_SERIAL.println(F("  [ACTION] Present QR code to scanner"));
        DEBUG_SERIAL.println(F("========================================\r\n"));

        while (1) {
            Scanner.update();

            if (Scanner.available()) {
                DEBUG_SERIAL.print(F("[QR] detected: "));
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
    //  Stage 9: 转盘步进电机测试
    //  验证: 转盘旋转精度、72°步数换算
    //  流程: CCW 72°→CW回 → CCW 144°→CW回 → CCW 216°→CW回
    // ==================================================================
#elif DEBUG_STAGE == 9
    {
        // 72° CCW → return
        TurntableMotor.rotateMultiples(1, 200);
        while (!TurntableMotor.isDone()) { TurntableMotor.update(); }
        delay(2000);
        TurntableMotor.rotateMultiples(-1, 200);
        while (!TurntableMotor.isDone()) { TurntableMotor.update(); }
        delay(2000);

        // 144° CCW → return
        TurntableMotor.rotateMultiples(2, 200);
        while (!TurntableMotor.isDone()) { TurntableMotor.update(); }
        delay(2000);
        TurntableMotor.rotateMultiples(-2, 200);
        while (!TurntableMotor.isDone()) { TurntableMotor.update(); }
        delay(2000);

        // 216° CCW → return
        TurntableMotor.rotateMultiples(3, 200);
        while (!TurntableMotor.isDone()) { TurntableMotor.update(); }
        delay(2000);
        TurntableMotor.rotateMultiples(-3, 200);
        while (!TurntableMotor.isDone()) { TurntableMotor.update(); }

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

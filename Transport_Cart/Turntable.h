// 转盘步进电机 + 5 仓位管理 + 颜色识别定位
//
// 机械结构:
//   转盘贴地圆盘，5 等分仓位（每仓 72°），上封闭、下开口，车头前方为开口，
//   物块从开口进入被罩在对应仓位格内。
//   颜色识别模块(TCS230)固定在转盘上、位于车头开口正对侧（180°，
//   见 Config.h 的 TURNTABLE_SENSOR_DEG）：车头仓内的物块转 180° 即到
//   识别模块处，可对其做颜色识别。
//
// 仓位编号约定（现场已确认）:
//   - 上电 0 号仓对准车头开口
//   - rotateMultiples(+1)（+72°）后，车头仓位号 +1
//     （若实测转向相反，需调整电机接线方向或换向约定）
//   - 仓位号按车头推进方向顺序编号 0~4
//
// 角度模型:
//   内部自跟踪绝对角度 _deg（上电为 0，+72° = 车头仓号 +1 的方向），
//   任意角度旋转后仍能精确计算指定仓位转到车头(rotateToCell)或
//   转到颜色识别模块(rotateToSensor)的最短旋转。
//
// 功能:
//   - rotateMultiples(n)   旋转 n×72°（非阻塞，取料推进用）
//   - rotateDegrees(deg)   旋转任意角度（非阻塞，180° 识别定位用）
//   - rotateToCell(cell)   指定仓位转到车头开口（最短路径）
//   - rotateToSensor(cell) 指定仓位转到颜色识别模块处（最短路径）
//   - 仓位占用标记、每仓物块颜色记录（任务1颜色识别用）
//
// 取料示例（任务1/2 通用）:
//   TurntableMotor.occupyFront();            // 物块已进入车头仓
//   TurntableMotor.rotateMultiples(1);       // 下一空仓到车头
//
// 颜色识别示例（任务1流程）:
//   TurntableMotor.rotateToSensor(cell);     // 该仓转到识别模块处
//   ... 等 ColorSensors 一轮测量完成 ...
//   TurntableMotor.setCellColor(cell, col);  // 记录颜色到该仓

#ifndef TURNTABLE_H
#define TURNTABLE_H

#include <Arduino.h>
#include "Config.h"
#include "Stepper.h"

class Turntable {
public:
    static const uint8_t CELL_COUNT = 5;   // 仓位数（5 等分）

    Turntable();

    void init();

    // 状态机驱动（loop 中反复调用）；旋转完成后自动累计绝对角度
    void update();

    // 旋转 multiples × 72°（非阻塞）
    bool rotateMultiples(int multiples, unsigned long stepUs = 200);

    // 旋转任意角度（非阻塞），供 180° 识别定位等使用
    bool rotateDegrees(float degrees, unsigned long stepUs = 200);

    // 旋转使指定仓位转到车头开口（最短路径，非阻塞）
    bool rotateToCell(uint8_t cell, unsigned long stepUs = 200);

    // 旋转使指定仓位转到颜色识别模块处（最短路径，非阻塞）
    // 车头仓 → 恰好转 TURNTABLE_SENSOR_DEG(180°)
    bool rotateToSensor(uint8_t cell, unsigned long stepUs = 200);

    // 是否已到达目标（旋转完成）
    bool isDone() const { return !_moving; }

    // 复位（仅复位运动状态，不清仓位数据/角度）
    void reset();

    // 紧急停止
    void emergencyStop();

    // ===================== 仓位管理 =====================

    // 当前车头开口最近的仓位号（72° 整数倍对准时精确）
    uint8_t frontCell() const;

    // 指定仓位是否已收入物块
    bool isOccupied(uint8_t cell) const;

    // 标记当前车头仓位已收入一个物块
    void occupyFront();

    // 清空指定仓位（占用+颜色）
    void clearCell(uint8_t cell);

    // 记录指定仓位物块的颜色（ColorSensor::Color 码，0=无）
    void setCellColor(uint8_t cell, uint8_t color);

    // 读取指定仓位物块颜色（0=无）
    uint8_t cellColor(uint8_t cell) const;

private:
    // 角度差归一到 (-180, 180] 的最短旋转量
    static float _normDelta(float d);

    Stepper _motor;
    bool _moving;
    long _targetSteps;
    float _deg;         // 转盘绝对角度（上电=0；+72° = 车头仓号+1 方向）
    float _pendingDeg;  // 本次旋转完成后应累计的角度
    bool _occupied[CELL_COUNT];  // 各仓位占用标记
    uint8_t _color[CELL_COUNT];  // 各仓位物块颜色（ColorSensor::Color 码）
};

// 全局实例
extern Turntable TurntableMotor;

// 全局初始化
void Turntable_Init();

#endif

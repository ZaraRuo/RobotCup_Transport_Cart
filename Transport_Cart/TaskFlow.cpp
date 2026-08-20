// ============================================================================
// TaskFlow.cpp — 任务流程（状态机）
//   执行顺序: 任务2(颁奖) → 任务1(搬运物料)（规则允许自选，先2后1）
//   状态机: 每个 STEP_xxx 是一个环节；loop 每圈推进当前环节的一小步。
//   带【用户动作】的函数需按实测填写（见各函数注释），其余逻辑已实现。
//
//   任务2规则: 扫码(1~6)得到 3 个摆放点(编号1/2/3)上各摆的是 A/B/C 哪个物块；
//   按路径拾取顺序 点3→点2→点1 收块；摆放固定（与方案无关）:
//   A→冠军台, B→亚军台, C→季军台。
//
// ============================================================================
// 摆放机制与规则（已由用户确认，勿违背）:
//   1) 推杆的作用是【抬升转盘】，用于适配颁奖台高度：
//      冠军台高36mm、亚军台高18mm → 需伸长推杆抬升转盘；
//      季军台贴地(0mm) → 不需要推杆。
//   2) 放料动作本身 = 车辆前进到目标 → 物块所在仓位转到车头开口 →
//      车辆后退 → 物块留下（车头"罩"从物块上滑脱）。推杆不是放料动作。
//   3) 任务2摆放（固定，与方案无关）: A→冠军台, B→亚军台, C→季军台；
//      方案表只用于收料时得知"哪个摆放点上是哪个物块"。
//   4) 任务1摆放: 5个物块全部收取后统一颜色检测，记录"哪个仓是什么颜色"；
//      按扫任务1二维码得到的 颜色物块↔位置A~E 对应关系摆放；
//      每块: 车辆前进到目标位置 → 该颜色物块所在仓转到车头 → 后退放料。
// ============================================================================

#include "TaskFlow.h"
#include "Config.h"
#include "Chassis.h"
#include "QRScanner.h"
#include "Turntable.h"
#include "PushRod.h"
#include "ColorSensor.h"

// ============================================================================
// 一、常量定义
// ============================================================================

// 任务2 物块编号
#define BLK_A  0   // 物块 A
#define BLK_B  1   // 物块 B
#define BLK_C  2   // 物块 C

// 颜色码（与 ColorSensor::Color 一致）
#define COL_NONE  0    // 无/未识别
#define COL_GREEN 1    // 绿
#define COL_WHITE 2    // 白
#define COL_RED   3    // 红
#define COL_BLACK 4    // 黑
#define COL_BLUE  5    // 蓝

// 颁奖台: 0=冠军台, 1=亚军台, 2=季军台（规则: A→冠军 B→亚军 C→季军）
// 任务1目标区: A=0, B=1, C=2, D=3, E=4

// ============================================================================
// 二、方案表
// ============================================================================

// ---- 任务2 方案表 ----
// 扫到二维码 N(1~6) 后，查 T2_PLAN[N-1] 得到: 地图上 3 个摆放点各摆的是哪个物块。
// 每行 = {摆放点1, 摆放点2, 摆放点3} 上的物块（A/B/C）
// 用途: 按拾取顺序 点3→点2→点1 收块时，用本表得知"刚收到的这个物块是谁"。
// 摆放规则固定(与方案无关): A→冠军台, B→亚军台, C→季军台。
static const uint8_t T2_PLAN[6][3] = {
    {BLK_A, BLK_B, BLK_C},   // 1: 点1=A 点2=B 点3=C
    {BLK_A, BLK_C, BLK_B},   // 2: 点1=A 点2=C 点3=B
    {BLK_B, BLK_A, BLK_C},   // 3: 点1=B 点2=A 点3=C
    {BLK_B, BLK_C, BLK_A},   // 4: 点1=B 点2=C 点3=A
    {BLK_C, BLK_A, BLK_B},   // 5: 点1=C 点2=A 点3=B
    {BLK_C, BLK_B, BLK_A},   // 6: 点1=C 点2=B 点3=A
};

// ---- 任务1 方案表 ----
// 扫到二维码 N(1~16) 后，查 T1_PLAN[N-1] 得到: 5 个目标位置各放什么颜色。
// 每行 = {A位, B位, C位, D位, E位} 的颜色（数据来自《补充说明》任务1表）
static const uint8_t T1_PLAN[16][5] = {
    {COL_BLACK, COL_WHITE, COL_RED,   COL_GREEN, COL_BLUE},  //  1: 黑白红绿蓝
    {COL_WHITE, COL_BLACK, COL_RED,   COL_GREEN, COL_BLUE},  //  2: 白黑红绿蓝
    {COL_WHITE, COL_BLACK, COL_GREEN, COL_RED,   COL_BLUE},  //  3: 白黑绿红蓝
    {COL_BLUE,  COL_WHITE, COL_BLACK, COL_RED,   COL_GREEN}, //  4: 蓝白黑红绿
    {COL_WHITE, COL_RED,   COL_BLUE,  COL_BLACK, COL_GREEN}, //  5: 白红蓝黑绿
    {COL_BLACK, COL_RED,   COL_BLUE,  COL_WHITE, COL_GREEN}, //  6: 黑红蓝白绿
    {COL_BLUE,  COL_GREEN, COL_BLACK, COL_WHITE, COL_RED},   //  7: 蓝绿黑白红
    {COL_GREEN, COL_WHITE, COL_BLUE,  COL_BLACK, COL_RED},   //  8: 绿白蓝黑红
    {COL_WHITE, COL_GREEN, COL_BLACK, COL_BLUE,  COL_RED},   //  9: 白绿黑蓝红
    {COL_BLACK, COL_RED,   COL_BLUE,  COL_GREEN, COL_WHITE}, // 10: 黑红蓝绿白
    {COL_RED,   COL_BLUE,  COL_GREEN, COL_BLACK, COL_WHITE}, // 11: 红蓝绿黑白
    {COL_GREEN, COL_RED,   COL_BLACK, COL_BLUE,  COL_WHITE}, // 12: 绿红黑蓝白
    {COL_WHITE, COL_RED,   COL_BLUE,  COL_GREEN, COL_BLACK}, // 13: 白红蓝绿黑
    {COL_RED,   COL_GREEN, COL_WHITE, COL_BLUE,  COL_BLACK}, // 14: 红绿白蓝黑
    {COL_BLUE,  COL_WHITE, COL_GREEN, COL_RED,   COL_BLACK}, // 15: 蓝白绿红黑
    {COL_GREEN, COL_BLUE,  COL_RED,   COL_WHITE, COL_BLACK}, // 16: 绿蓝红白黑
};

// ============================================================================
// 三、步骤枚举（按实际执行顺序: 任务2 → 任务1）
// ============================================================================
// 每个值 = 任务剧本里的一行；"调用 xxx()"= 需用户填写动作的环节
typedef enum {
    STEP_IDLE,
    // —— 任务2（先执行）——
    STEP_T2_LEAVE_HOME,   // 出HOME                        → T2_LeaveHome()
    STEP_T2_TO_QR,        // 去右二维码区                   → T2_MoveToQRZone()
    STEP_T2_SCAN,         // 扫码等方案(1~6)                [已实现]
    STEP_T2_TRACK_COLLECT,// 持续循迹收块(里程碑触发转盘)   [已实现]
    STEP_T2_TO_PODIUM,    // 摆放全部3块到颁奖台            → T2_MoveToPodium()
    // —— 任务1（后执行）——
    STEP_T1_TO_QR,        // 去左二维码区                  → T1_MoveToQRZone()
    STEP_T1_SCAN,         // 扫码等方案(1~16)              [已实现]
    STEP_T1_TRACK_COLLECT,// 持续循迹收块(里程碑触发转盘)   [已实现]
    STEP_T1_DETECT_CELL,  // 该仓转180°到识别模块          [已实现]
    STEP_T1_DETECT_WAIT,  // 等颜色测量并记录              [已实现]
    STEP_T1_SHIELD,       // 识别完后转36°遮蔽车头开口     [已实现]
    STEP_T1_TO_TARGET,    // 摆放全部5块(一次性)          → T1_PlaceAllBlocks()
    STEP_T1_RETURN_HOME,  // 返回HOME（全部完成）           → T1_ReturnHome()
    STEP_DONE
} TaskStep;

// ============================================================================
// 四、状态变量（按执行顺序分组）
// ============================================================================
static TaskFlowState _state = TF_IDLE;   // 流程状态: 空闲/运行/完成
static TaskStep _step = STEP_IDLE;       // 当前环节（状态机走到哪了）

// —— 任务2 ——
static uint8_t _plan = 0;                // 任务2方案号(0~5)  = 扫码值-1
static uint8_t _block = BLK_A;           // 任务2当前处理的物块(A/B/C)
static uint8_t _pickIdx = 0;             // 任务2拾取计数(0~2, 顺序 点3→点2→点1)
static uint8_t _cellOfBlock[3];          // 任务2各物块收进转盘时所在的仓号

// —— 任务1 ——
static uint8_t _t1Plan = 0;              // 任务1方案号(0~15) = 扫码值-1
static uint8_t _collectIdx = 0;          // 任务1已收物块计数(0~4)
static uint8_t _detectIdx = 0;           // 任务1正在识别的仓号(0~4)

// —— 通用 ——
static unsigned long _scanStartMs = 0;   // 扫码开始时刻（用于10s超时）
static unsigned long _detectT0 = 0;      // 颜色测量开始时刻（用于3s超时）
static bool _busy = false;               // 是否正在等一个非阻塞动作完成
static bool _finalHalf = false;          // 收料结束后 36° 遮蔽旋转是否已执行
static TaskStep _debugStopAfter = STEP_DONE; // 调试: 阶段运行到此步骤即停止

// ============================================================================
// 五、摆放配合辅助函数（供用户动作函数调用，无需修改）
// ============================================================================

// 把指定物块(A/B/C)所在仓转到车头开口（放料位），等待转盘到位（任务2用）
static void _alignBlockCell(uint8_t block) {
    TurntableMotor.rotateToCell(_cellOfBlock[block], 200); // 该仓转到车头开口
    while (!TurntableMotor.isDone()) { TurntableMotor.update(); } // 等转完
}

// 查找装有指定颜色的仓号（任务1颜色识别阶段已用 setCellColor 记录）
static uint8_t _findCellByColor(uint8_t color) {
    for (uint8_t c = 0; c < Turntable::CELL_COUNT; c++) {   // 遍历5个仓
        if (TurntableMotor.cellColor(c) == color) return c;  // 找到即返回
    }
    return 0;                          // 找不到 → 默认0号仓
}

// 把装有指定颜色的物块仓转到车头开口（放料位），等待转盘到位（任务1用）
static void _alignColorCell(uint8_t color) {
    uint8_t cell = _findCellByColor(color);        // 定位该颜色物块的仓号
    TurntableMotor.rotateToCell(cell, 200);        // 该仓转到车头开口
    while (!TurntableMotor.isDone()) { TurntableMotor.update(); } // 等转完
}

// ============================================================================
// 六、用户动作区（按执行顺序: 任务2 → 任务1）
// ============================================================================
//   以下 6 个函数由你按实测填写。
//   约定: 这些函数是【阻塞式】的——必须"启动运动并等它做完"再返回：
//       Cart.moveMm(300, 160);                     // 启动: 前进300mm
//       while (!Cart.isDone()) { Cart.update(); }  // 等待走完
//   可用接口: Cart.moveMm / rotateAngle / trackForwardMm / trackBackwardMm
//            （定距循迹，单位mm）/ trackArc*；速度 stepUs 用 160~240 较稳。
// ============================================================================

// ---------------------------------------------------------------
// 任务2 运动原语
// ---------------------------------------------------------------

// 【任务2-1】出 HOME：车驶离 HOME，车头对准"去右二维码区"的方向
void T2_LeaveHome() {
    // 先定距前进
    Cart.moveMm(333.0f, 800);  // 前进 333mm，每步间隔 800us
    while (!Cart.isDone()) { Cart.update(); }
    
    // 再右转 90°
    Cart.rotateAngle(-90.0f, 800);  // 右转 90°（负值=顺时针）
    while (!Cart.isDone()) { Cart.update(); }
}

// 【任务2-2】去右二维码区：停到二维码正前方，让扫码模块能扫到
void T2_MoveToQRZone() {
    // 循迹前进至右二维码区（定距循迹，单位 mm）
    Cart.trackForwardMm(537.4f, 800);
    while (!Cart.isDone()) { Cart.update(); }
}

// 【任务2-3】摆放全部 3 个物块到颁奖台（完整流程）
//   包含：初始定位 + 3次放料（移动 + 推杆抬升 + 转盘对位 + 后退放料）
void T2_MoveToPodium(uint8_t podium) {
    // ========== 初始定位 ==========
    // 左转90度
    Cart.rotateAngle(90.0f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // 定距直行（到内圈）
    Cart.moveMm(400.0f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // 右转90度
    Cart.rotateAngle(-90.0f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // 定距循迹前进
    Cart.trackForwardMm(909.0f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // 右转90度
    Cart.rotateAngle(-90.0f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // ========== 放第1块（亚军台 = B 块） ==========
    // 推杆缩回: 转盘落地（放料准备）
    Rod.retractToBottom();
    
    // 定距直行（到亚军台）
    Cart.moveMm(247.0f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // 把 B 物块所在仓转到车头开口（放料位）
    _alignBlockCell(BLK_B);
    
    // 伸长推杆 = 抬升转盘，适配亚军台(18mm)高度
    Rod.extendToTop();
    
    // 定距后退 = 放料（车头"罩"滑脱，物块留在亚军台）
    Cart.moveMm(-247.0f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // ========== 放第2块（冠军台 = A 块） ==========
    // 左转90度
    Cart.rotateAngle(90.0f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // 定距循迹前进
    Cart.trackForwardMm(269.6f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // 右转90度
    Cart.rotateAngle(-90.0f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // 推杆缩回: 转盘落地（放料准备）
    Rod.retractToBottom();
    
    // 定距直行（到冠军台）
    Cart.moveMm(247.0f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // 把 A 物块所在仓转到车头开口（放料位）
    _alignBlockCell(BLK_A);
    
    // 伸长推杆 = 抬升转盘，适配冠军台(36mm)高度
    Rod.extendToTop();
    
    // 定距后退 = 放料（物块留在冠军台）
    Cart.moveMm(-247.0f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // ========== 放第3块（季军台 = C 块，贴地无高度，不需推杆） ==========
    // 左转90度
    Cart.rotateAngle(90.0f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // 定距循迹前进
    Cart.trackForwardMm(270.2f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // 右转90度
    Cart.rotateAngle(-90.0f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // 定距直行（到季军台）
    Cart.moveMm(247.0f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // 把 C 物块所在仓转到车头开口（放料位）
    _alignBlockCell(BLK_C);
    
    // 季军台贴地，不需抬升推杆，直接后退放料
    // 定距后退 = 放料（物块留在季军台）
    Cart.moveMm(-1677.0f, 800);
    while (!Cart.isDone()) { Cart.update(); }
}

// ---------------------------------------------------------------
// 任务1 运动原语
// ---------------------------------------------------------------

// 【任务1-1】去左二维码区：停到二维码正前方，让扫码模块能扫到
void T1_MoveToQRZone() {
    // 左转90度
    Cart.rotateAngle(90.0f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // 定距循迹前进
    Cart.trackForwardMm(235.6f, 800);  // 占位值，需实测
    while (!Cart.isDone()) { Cart.update(); }
}

// 【任务1-2】摆放全部 5 个物块到目标位置（完整流程）
//   顺序: A → C → E → D → B
//   每块: 前进到目标位置 → 按方案找该位置应放颜色的物块仓 → 转到车头 → 后退放料
void T1_PlaceAllBlocks() {
    // ========== 初始定位 ==========
    // 右转90度
    Cart.rotateAngle(-90.0f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // 定距直行（到内圈）
    Cart.moveMm(400.0f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // 左转90度
    Cart.rotateAngle(90.0f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // 定距循迹前进
    Cart.trackForwardMm(273.3f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // 定距直线倒车
    Cart.moveMm(-56.4f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // ========== 放 A 块（位置A；方案中A位=T1_PLAN[plan][0]） ==========
    // 右转90度
    Cart.rotateAngle(-90.0f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // 定距直行（到A）
    Cart.moveMm(63.7f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // 把"位置A应放颜色"的物块所在仓转到车头开口（放料位）
    _alignColorCell(T1_PLAN[_t1Plan][0]);
    
    // 定距后退 = 放料（车头"罩"滑脱，物块留在A）
    Cart.moveMm(-63.7f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // 左转90度
    Cart.rotateAngle(90.0f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // 定距循迹前进
    Cart.trackForwardMm(852.4f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // ========== 放 C 块（位置C；方案中C位=T1_PLAN[plan][2]） ==========
    // 右转90度
    Cart.rotateAngle(-90.0f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // 定距直行（到C）
    Cart.moveMm(66.0f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // 把"位置C应放颜色"的物块所在仓转到车头开口（放料位）
    _alignColorCell(T1_PLAN[_t1Plan][2]);
    
    // 定距后退 = 放料（物块留在C）
    Cart.moveMm(-66.0f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // 左转90度
    Cart.rotateAngle(90.0f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // 定距循迹前进
    Cart.trackForwardMm(1980.8f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // 定距直线倒车
    Cart.moveMm(-223.0f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // ========== 放 E 块（位置E；方案中E位=T1_PLAN[plan][4]） ==========
    // 右转90度
    Cart.rotateAngle(-90.0f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // 定距直行（到E）
    Cart.moveMm(167.0f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // 把"位置E应放颜色"的物块所在仓转到车头开口（放料位）
    _alignColorCell(T1_PLAN[_t1Plan][4]);
    
    // 定距后退 = 放料（物块留在E）
    Cart.moveMm(-176.0f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // 左转90度
    Cart.rotateAngle(90.0f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // 定距循迹前进
    Cart.trackForwardMm(503.3f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // ========== 放 D 块（位置D；方案中D位=T1_PLAN[plan][3]） ==========
    // 右转90度
    Cart.rotateAngle(-90.0f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // 定距直行（到D）
    Cart.moveMm(107.0f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // 把"位置D应放颜色"的物块所在仓转到车头开口（放料位）
    _alignColorCell(T1_PLAN[_t1Plan][3]);
    
    // 定距后退 = 放料（物块留在D）
    Cart.moveMm(-107.0f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // 左转90度
    Cart.rotateAngle(90.0f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // 定距循迹前进
    Cart.trackForwardMm(760.5f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // ========== 放 B 块（位置B；方案中B位=T1_PLAN[plan][1]） ==========
    // 右转90度
    Cart.rotateAngle(-90.0f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // 定距直行（到B）
    Cart.moveMm(106.0f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // 把"位置B应放颜色"的物块所在仓转到车头开口（放料位）
    _alignColorCell(T1_PLAN[_t1Plan][1]);
    
    // 定距后退 = 放料（物块留在B）
    Cart.moveMm(-106.0f, 800);
    while (!Cart.isDone()) { Cart.update(); }
}

// 【任务1-3】返回 HOME：全部轮子进入 HOME 区并停稳
void T1_ReturnHome() {
    // 右转90度
    Cart.rotateAngle(-90.0f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // 定距循迹前进
    Cart.trackForwardMm(425.9f, 800);  // 占位值，需实测
    while (!Cart.isDone()) { Cart.update(); }
    
    // 右转90度
    Cart.rotateAngle(-90.0f, 800);
    while (!Cart.isDone()) { Cart.update(); }
    
    // 定距直行
    Cart.moveMm(487.8f, 800);  // 占位值，需实测
    while (!Cart.isDone()) { Cart.update(); }
}

// ============================================================================
// 七、工具函数
// ============================================================================

// 颜色码 → 名称（仅用于串口日志，方便人看）
static const char *colorName(uint8_t col) {
    switch (col) {
    case COL_GREEN: return "GREEN";
    case COL_WHITE: return "WHITE";
    case COL_RED:   return "RED";
    case COL_BLACK: return "BLACK";
    case COL_BLUE:  return "BLUE";
    default:        return "NONE";
    }
}

// ============================================================================
// 八、对外接口
// ============================================================================

// 上电初始化: 流程置为"运行中"，从任务2第一步开始（先2后1）
void TaskFlow_Init() {
    _state = TF_RUNNING;              // 标记流程开始运行
    _step = STEP_T2_LEAVE_HOME;       // 从任务2的"出HOME"开始
    _t1Plan = 0;                      // 清空任务1方案
    _plan = 0;                        // 清空任务2方案
    _block = BLK_A;                   // 摆放从 A 开始
    _pickIdx = 0;                     // 拾取计数清零（从点3开始收）
    _collectIdx = 0;                  // 清空任务1收料计数
    _detectIdx = 0;                   // 清空识别仓号
    _busy = false;                    // 无进行中的动作
    _finalHalf = false;               // 收料结束36°旋转标记清零
    _debugStopAfter = STEP_DONE;      // 正常模式: 无阶段停止点
}

// 状态机主循环: 正常模式下 loop() 只调本函数，所以先泵动各硬件模块
void TaskFlow_Update() {
    // 泵动各模块（不调 update 的话，底盘/转盘/扫码/颜色都不会动）
    Cart.update();                    // 底盘状态机推进
    Scanner.update();                 // 二维码收包解析
    TurntableMotor.update();          // 转盘旋转推进（完成后自动更新仓位）
    ColorSensors.update();            // 颜色测量状态机推进

    if (_state != TF_RUNNING) return; // 流程未运行(空闲/已完成)则什么都不做

    // 调试阶段运行: 到达指定步骤即停止（不执行该步骤，_state 回 IDLE）
    if (_debugStopAfter != STEP_DONE && _step == _debugStopAfter) {
        _state = TF_IDLE;
        return;
    }

    switch (_step) {                  // 看当前到剧本的哪一行

    // ===================== 任务2（先执行） =====================

    // ---- 出HOME → 去右二维码区 ----
    case STEP_T2_LEAVE_HOME:
        DEBUG_SERIAL.println(F("[T2] leave HOME")); // 日志
        T2_LeaveHome();               // 【用户动作】车驶出HOME
        _step = STEP_T2_TO_QR;        // 下一步: 去二维码区
        break;

    case STEP_T2_TO_QR:
        T2_MoveToQRZone();            // 【用户动作】移到右二维码区
        _step = STEP_T2_SCAN;         // 下一步: 扫码
        _scanStartMs = millis();      // 记下开始等待的时刻
        break;

    // ---- 扫码: 等结果(1~6)，非法/超时10s → 回定位重扫 ----
    case STEP_T2_SCAN: {
        if (Scanner.available()) {     // 收到一帧扫码结果?
            int n = Scanner.result();  // 取出二维码数字
            if (n >= 1 && n <= 6) {    // 是合法方案号?
                _plan = (uint8_t)n - 1;  // 转成数组下标(0~5)存起来
                _pickIdx = 0;          // 拾取计数清零（从点3开始收）
                DEBUG_SERIAL.print(F("[T2] plan #")); // 日志: 方案号
                DEBUG_SERIAL.println(n);
                _step = STEP_T2_TRACK_COLLECT; // 下一步: 开始收料
            } else {                   // 扫到非法数字
                DEBUG_SERIAL.print(F("[T2] invalid scan=")); // 日志
                DEBUG_SERIAL.println(n);
                _step = STEP_T2_TO_QR; // 回去重新定位再扫
            }
        } else if (millis() - _scanStartMs > 10000UL) { // 超过10秒没扫到
            DEBUG_SERIAL.println(F("[T2] scan timeout, re-position")); // 日志
            _step = STEP_T2_TO_QR;     // 回去重新定位再扫
        }
        break;
    }

    // ---- 收料: 持续循迹不停车，到达拾取点时记录并收格 ----
    //   收格旋转只需前 ROTATE_AT 个拾取点触发（最后一块进仓后不转72°）；
    //   循迹走完总里程后，再转 36°（半格），使物块不暴露在车头开口处。
    case STEP_T2_TRACK_COLLECT: {
        // 3个拾取点位置，单位 mm（占位值，需实测）
        static const float milestoneMM[3] = { 979.3f, 1390.0f, 1816.5f };
        // 总循迹距离，单位 mm（覆盖3个取料点，占位值）
        static const float totalMM = 1816.5f;
        // 需要转72°收格的拾取点个数（只前2个；第3块进仓后不转）
        static const uint8_t ROTATE_AT = 2;

        if (!_busy) {
            // 首次进入: 启动持续循迹（不停车）
            DEBUG_SERIAL.println(F("[T2] continuous track collect start"));
            Cart.trackForwardMm(totalMM, 800);
            _pickIdx = 0;
            _finalHalf = false;
            _busy = true;
        } else {
            // 到达当前拾取点（左右轮【都】达到才触发），物块已进入车头仓
            if (_pickIdx < 3 && TurntableMotor.isDone() &&
                (MotorL.currentStep() >= (long)(milestoneMM[_pickIdx] * WHEEL_STEPS_PER_MM) &&
                 MotorR.currentStep() >= (long)(milestoneMM[_pickIdx] * WHEEL_STEPS_PER_MM))) {
                uint8_t point = 2 - _pickIdx;             // 当前摆放点(0=点1,1=点2,2=点3)
                uint8_t blk = T2_PLAN[_plan][point];      // 查表: 该点摆的是哪个物块
                TurntableMotor.occupyFront();             // 标记车头仓已占用
                _cellOfBlock[blk] = TurntableMotor.frontCell(); // 记录仓号
                DEBUG_SERIAL.print(F("[T2] block collected at point "));
                DEBUG_SERIAL.println(point);
                // 前 ROTATE_AT 个拾取点收完后转72°腾出下一空仓；最后一个不转
                if (_pickIdx < ROTATE_AT) {
                    TurntableMotor.rotateMultiples(1, 200);
                }
                _pickIdx++;
            }
            // 循迹完成且3块都收完 → 先转36°遮蔽开口，再进入摆放
            if (Cart.isDone() && _pickIdx >= 3) {
                if (!_finalHalf) {
                    // 转 36°（半格）: 让物块不直接暴露在车头开口处
                    DEBUG_SERIAL.println(F("[T2] final rotate 36deg"));
                    TurntableMotor.rotateDegrees(36.0f, 200);
                    _finalHalf = true;
                } else if (TurntableMotor.isDone()) {
                    // 36° 转完 → 进入摆放阶段
                    DEBUG_SERIAL.println(F("[T2] 3 blocks collected, start placing"));
                    _busy = false;
                    _block = BLK_A;
                    _step = STEP_T2_TO_PODIUM;
                }
            }
        }
        break;
    }

    // ---- 摆放: 一次性摆放全部 3 个物块到颁奖台 ----
    case STEP_T2_TO_PODIUM:
        DEBUG_SERIAL.println(F("[T2] placing all 3 blocks"));
        T2_MoveToPodium(0);             // 【用户动作】完整摆放流程（3块全部）
        // 任务2完成，直接进入任务1（无需返回HOME）
        _t1Plan = 0;                  // 清空任务1方案
        _collectIdx = 0;              // 清空收料计数
        _detectIdx = 0;               // 清空识别仓号
        _step = STEP_T1_TO_QR;        // 直接进入任务1: 去二维码区
        break;

    // ===================== 任务1（后执行） =====================

    // ---- 去左二维码区（任务2完成后直接进入，无需出HOME） ----
    case STEP_T1_TO_QR:
        T1_MoveToQRZone();            // 【用户动作】移到左二维码区
        _step = STEP_T1_SCAN;         // 下一步: 扫码
        _scanStartMs = millis();      // 记下开始等待的时刻（超时用）
        break;

    // ---- 扫码: 等结果(1~16)，非法/超时10s → 回定位重扫 ----
    case STEP_T1_SCAN: {
        if (Scanner.available()) {     // 收到一帧扫码结果?
            int n = Scanner.result();  // 取出二维码数字
            if (n >= 1 && n <= 16) {   // 是合法方案号?
                _t1Plan = (uint8_t)n - 1; // 转成数组下标(0~15)存起来
                _collectIdx = 0;       // 收料计数清零，准备收第1块
                DEBUG_SERIAL.print(F("[T1] plan #")); // 日志: 方案号
                DEBUG_SERIAL.println(n);
                _step = STEP_T1_TRACK_COLLECT; // 下一步: 开始收料
            } else {                   // 扫到非法数字
                DEBUG_SERIAL.print(F("[T1] invalid scan=")); // 日志
                DEBUG_SERIAL.println(n);
                _step = STEP_T1_TO_QR; // 回去重新定位再扫
            }
        } else if (millis() - _scanStartMs > 10000UL) { // 超过10秒没扫到
            DEBUG_SERIAL.println(F("[T1] scan timeout, re-position")); // 日志
            _step = STEP_T1_TO_QR;     // 回去重新定位再扫
        }
        break;
    }

    // ---- 收料: 持续循迹不停车，到达拾取点时记录并收格 ----
    //   收格旋转只需前 ROTATE_AT 个拾取点触发（最后一块进仓后不转72°）；
    //   36°遮蔽在【识别完成之后】的 STEP_T1_SHIELD 执行，不在此阶段。
    case STEP_T1_TRACK_COLLECT: {
        // 5个拾取点位置，单位 mm（占位值，需实测）
        static const float milestoneMM[5] = { 396.6f, 872.6f, 1348.5f, 1824.5f, 2300.4f };
        // 总循迹距离，单位 mm（覆盖5个取料点，占位值）
        static const float totalMM = 2300.4f;
        // 需要转72°收格的拾取点个数（只前4个；第5块进仓后不转）
        static const uint8_t ROTATE_AT = 4;

        if (!_busy) {
            // 首次进入: 启动持续循迹（不停车）
            DEBUG_SERIAL.println(F("[T1] continuous track collect start"));
            Cart.trackForwardMm(totalMM, 800);
            _collectIdx = 0;
            _finalHalf = false;
            _busy = true;
        } else {
            // 到达当前拾取点（左右轮【都】达到才触发），物块已进入车头仓
            if (_collectIdx < 5 && TurntableMotor.isDone() &&
                (MotorL.currentStep() >= (long)(milestoneMM[_collectIdx] * WHEEL_STEPS_PER_MM) &&
                 MotorR.currentStep() >= (long)(milestoneMM[_collectIdx] * WHEEL_STEPS_PER_MM))) {
                TurntableMotor.occupyFront();             // 标记车头仓已占用
                DEBUG_SERIAL.print(F("[T1] block #"));
                DEBUG_SERIAL.println(_collectIdx);
                // 前 ROTATE_AT 个拾取点收完后转72°腾出下一空仓；最后一个不转
                if (_collectIdx < ROTATE_AT) {
                    TurntableMotor.rotateMultiples(1, 200);
                }
                _collectIdx++;
            }
            // 循迹完成且5块都收完 → 进入颜色识别（36°遮蔽放到识别后 STEP_T1_SHIELD）
            if (Cart.isDone() && _collectIdx >= 5 && TurntableMotor.isDone()) {
                DEBUG_SERIAL.println(F("[T1] 5 blocks collected"));
                _busy = false;
                _detectIdx = 0;
                _step = STEP_T1_DETECT_CELL;
            }
        }
        break;
    }

    // ---- 颜色识别: 该仓转180°到识别模块 → 开始一轮测量 ----
    case STEP_T1_DETECT_CELL:
        if (!_busy) {                 // 本步还没开始?
            DEBUG_SERIAL.print(F("[T1] detect cell ")); // 日志: 识别几号仓
            DEBUG_SERIAL.println(_detectIdx);
            TurntableMotor.rotateToSensor(_detectIdx, 200); // 该仓转到识别模块(180°)
            _busy = true;             // 标记: 正在等转盘到位
        } else if (TurntableMotor.isDone()) { // 转盘到位了?
            _busy = false;            // 清除标记
            ColorSensors.begin();     // 启动一轮颜色测量(约0.4s)
            _detectT0 = millis();     // 记下测量开始时刻(3s超时用)
            _step = STEP_T1_DETECT_WAIT; // 下一步: 等测量结果
        }
        break;

    // ---- 等待颜色测量完成，把颜色记到该仓 ----
    case STEP_T1_DETECT_WAIT: {
        if (ColorSensors.available()) { // 测量完成，有结果了?
            uint8_t col = (uint8_t)ColorSensors.color(); // 取颜色码
            TurntableMotor.setCellColor(_detectIdx, col); // 记录: 该仓=该颜色
            DEBUG_SERIAL.print(F("[T1] cell "));  // 日志: 仓号
            DEBUG_SERIAL.print(_detectIdx);
            DEBUG_SERIAL.print(F(" = "));
            DEBUG_SERIAL.println(colorName(col)); // 日志: 颜色名
            _detectIdx++;             // 下一个仓
            if (_detectIdx < 5) {     // 还没识别完5个?
                _step = STEP_T1_DETECT_CELL; // 继续识别下一仓
            } else {                  // 5个都识别完了
                _step = STEP_T1_SHIELD; // 下一步: 转36°遮蔽开口，再摆放
            }
        } else if (millis() - _detectT0 > 3000UL) { // 超过3秒没测到
            DEBUG_SERIAL.print(F("[T1] cell "));  // 日志: 该仓超时
            DEBUG_SERIAL.print(_detectIdx);
            DEBUG_SERIAL.println(F(" detect timeout"));
            _detectIdx++;             // 跳过该仓，继续下一个
            if (_detectIdx < 5) {
                _step = STEP_T1_DETECT_CELL;
            } else {
                _step = STEP_T1_SHIELD;
            }
        }
        break;
    }

    // ---- 识别完成后: 转36°遮蔽开口（物块不暴露在车头），再进入摆放 ----
    case STEP_T1_SHIELD:
        if (!_finalHalf) {                        // 还没转过36°?
            DEBUG_SERIAL.println(F("[T1] shield rotate 36deg"));
            TurntableMotor.rotateDegrees(36.0f, 200); // 转半格遮蔽车头开口
            _finalHalf = true;                    // 标记已转
        } else if (TurntableMotor.isDone()) {     // 转完了?
            _step = STEP_T1_TO_TARGET;            // 下一步: 摆放全部5块
        }
        break;

    // ---- 放料: 一次性摆放全部 5 个物块 ----
    case STEP_T1_TO_TARGET:
        DEBUG_SERIAL.println(F("[T1] placing all 5 blocks"));
        T1_PlaceAllBlocks();          // 【用户动作】完整摆放流程（5块全部）
        _step = STEP_T1_RETURN_HOME;  // 下一步: 返回HOME
        break;

    // ---- 任务1结束: 返回HOME，全部任务完成 ----
    case STEP_T1_RETURN_HOME:
        DEBUG_SERIAL.println(F("[T1] return HOME")); // 日志
        T1_ReturnHome();              // 【用户动作】返回HOME
        _state = TF_DONE;             // 流程状态 → 完成
        DEBUG_SERIAL.println(F("[TF] ALL TASKS DONE")); // 日志: 全部完成
        break;

    default:
        break;                        // 其他值不做任何事
    }
}

// 获取当前流程状态（外部可查询: 空闲/运行/完成）
TaskFlowState TaskFlow_State() {
    return _state;
}

// ============================================================================
// 调试/测试支持实现（与正式流程共用同一套状态机/运动函数代码）
// ============================================================================

// 从指定步骤开始运行流程，到达 stopAfter 步骤时自动停止（不执行该步骤）
static void _debugRun(TaskStep start, TaskStep stopAfter) {
    _state = TF_RUNNING;
    _step = start;
    _debugStopAfter = stopAfter;
}

// 任务2连续收料阶段: 设方案，从收料开始，收满3块+36°遮蔽后停止
void TaskFlow_DebugRunCollectT2(uint8_t plan) {
    _plan = (plan >= 1 && plan <= 6) ? (uint8_t)(plan - 1) : 0;
    _pickIdx = 0;
    _busy = false;
    _finalHalf = false;
    _debugRun(STEP_T2_TRACK_COLLECT, STEP_T2_TO_PODIUM);
}

// 任务1连续收料阶段: 设方案，收满5块后停止（36°遮蔽在识别后的 SHIELD 阶段）
void TaskFlow_DebugRunCollectT1(uint8_t plan) {
    _t1Plan = (plan >= 1 && plan <= 16) ? (uint8_t)(plan - 1) : 0;
    _collectIdx = 0;
    _busy = false;
    _finalHalf = false;
    _debugRun(STEP_T1_TRACK_COLLECT, STEP_T1_DETECT_CELL);
}

// 任务1逐仓颜色识别阶段: 5仓识别完停止
void TaskFlow_DebugRunDetectT1() {
    _detectIdx = 0;
    _busy = false;
    _debugRun(STEP_T1_DETECT_CELL, STEP_T1_SHIELD);
}

// 任务1 36°遮蔽阶段: 转完停止
void TaskFlow_DebugRunShieldT1() {
    _finalHalf = false;
    _busy = false;
    _debugRun(STEP_T1_SHIELD, STEP_T1_TO_TARGET);
}

// 任务2摆放测试数据: 方案(1~6) + 各物块(A/B/C)所在仓号
void TaskFlow_DebugSetT2Place(uint8_t plan, uint8_t cellA, uint8_t cellB, uint8_t cellC) {
    _plan = (plan >= 1 && plan <= 6) ? (uint8_t)(plan - 1) : 0;
    _cellOfBlock[BLK_A] = cellA;
    _cellOfBlock[BLK_B] = cellB;
    _cellOfBlock[BLK_C] = cellC;
}

// 任务1摆放测试数据: 方案(1~16) + 各仓颜色（颜色码与 ColorSensor::Color 一致）
void TaskFlow_DebugSetT1Place(uint8_t plan, const uint8_t cellColors[5]) {
    _t1Plan = (plan >= 1 && plan <= 16) ? (uint8_t)(plan - 1) : 0;
    for (uint8_t i = 0; i < 5; i++) {
        TurntableMotor.setCellColor(i, cellColors[i]);
    }
}

// 任务流程模块
// 编排正常模式下的完整工作流程
// 内部基于状态机实现，每个步骤调用其他模块的功能函数

#include "TaskFlow.h"
// #include "Chassis.h"
// #include "QRScanner.h"
// #include "Turntable.h"
// #include "PushRod.h"

// ===================== 流程步骤枚举 =====================
// 后续按实际任务顺序定义步骤
typedef enum {
    STEP_IDLE,
    // STEP_START,       // 启动
    // STEP_TRACK,       // 循迹
    // STEP_SCAN_QR,     // 扫码
    // STEP_TURNTABLE,   // 转盘
    // STEP_PUSH_ROD,    // 推杆
    STEP_DONE
} TaskStep;

static TaskFlowState _state = TF_IDLE;
static TaskStep _step = STEP_IDLE;

// 初始化任务流程
void TaskFlow_Init() {
    _state = TF_IDLE;
    _step = STEP_IDLE;
}

// 任务流程状态机更新
void TaskFlow_Update() {
    if (_state != TF_RUNNING) return;

    switch (_step) {
    case STEP_IDLE:
    case STEP_DONE:
        _state = TF_DONE;
        break;

    default:
        break;
    }
}

// 获取当前流程状态
TaskFlowState TaskFlow_State() {
    return _state;
}

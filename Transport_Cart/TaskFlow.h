// 任务流程模块
// 正常模式下按步骤顺序执行：任务2(颁奖) → 任务1(搬运物料)
// 所有功能函数已在其他模块实现，此处只编排流程
//
// 【调试/测试支持】
//   为了让调试阶段的测试与正式流程使用同一套代码（测试通过即证明
//   正式流程对应部分可用），本头文件暴露:
//     - 6 个运动函数（与正式流程调用的完全相同）
//     - 4 个阶段运行接口（运行真实状态机的 收料/识别/遮蔽 阶段，到阶段结束即停）
//     - 2 个模拟数据接口（摆放测试前注入 方案/仓位/颜色）

#ifndef TASKFLOW_H
#define TASKFLOW_H

#include <Arduino.h>

// ===================== 对外接口（正常模式） =====================

// 任务流程状态枚举
typedef enum {
    TF_IDLE,       // 空闲
    TF_RUNNING,    // 运行中
    TF_DONE        // 完成
} TaskFlowState;

// 任务流程初始化（上电从任务2开始）
void TaskFlow_Init();

// 任务流程更新函数，在 loop() 中反复调用
void TaskFlow_Update();

// 获取当前流程状态
TaskFlowState TaskFlow_State();

// ===================== 运动函数（正式流程与测试共用） =====================
// 与正式流程 TaskFlow.cpp 内部调用的是同一份实现

// 任务2: 出HOME
void T2_LeaveHome();
// 任务2: 去右二维码区
void T2_MoveToQRZone();
// 任务2: 摆放全部3块到颁奖台（podium 参数保留未用）
void T2_MoveToPodium(uint8_t podium);

// 任务1: 去左二维码区
void T1_MoveToQRZone();
// 任务1: 摆放全部5块到目标位置
void T1_PlaceAllBlocks();
// 任务1: 返回HOME
void T1_ReturnHome();

// ===================== 阶段运行接口（测试用，同一套状态机代码） =====================
// 调用后需在 loop 中反复调用 TaskFlow_Update()，阶段完成自动停（_state 回到 TF_IDLE）

// 运行"任务2连续收料"阶段（plan: 1~6），收满3块+36°遮蔽后停止
void TaskFlow_DebugRunCollectT2(uint8_t plan);

// 运行"任务1连续收料"阶段（plan: 1~16），收满5块后停止
void TaskFlow_DebugRunCollectT1(uint8_t plan);

// 运行"任务1逐仓颜色识别"阶段，5仓识别完停止
void TaskFlow_DebugRunDetectT1();

// 运行"任务1 36°遮蔽"阶段，转完停止
void TaskFlow_DebugRunShieldT1();

// ===================== 模拟数据接口（测试用） =====================
// 摆放测试前注入状态，使 T2_MoveToPodium / T1_PlaceAllBlocks 可脱离完整流程运行

// 任务2摆放测试数据: 方案(1~6) + 各物块(A/B/C)所在仓号
void TaskFlow_DebugSetT2Place(uint8_t plan, uint8_t cellA, uint8_t cellB, uint8_t cellC);

// 任务1摆放测试数据: 方案(1~16) + 各仓颜色（颜色码与 ColorSensor::Color 一致）
void TaskFlow_DebugSetT1Place(uint8_t plan, const uint8_t cellColors[5]);

#endif

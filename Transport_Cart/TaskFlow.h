// 任务流程模块
// 正常模式下按步骤顺序执行：启动→循迹→扫码→转盘→推杆等
// 所有功能函数已在其他模块实现，此处只编排流程

#ifndef TASKFLOW_H
#define TASKFLOW_H

#include <Arduino.h>

// 任务流程状态枚举
typedef enum {
    TF_IDLE,       // 空闲
    TF_RUNNING,    // 运行中
    TF_DONE        // 完成
} TaskFlowState;

// 任务流程初始化
void TaskFlow_Init();

// 任务流程更新函数，在 loop() 中反复调用
void TaskFlow_Update();

// 获取当前流程状态
TaskFlowState TaskFlow_State();

#endif

#pragma once

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>

extern "C" {
#include "motor_control.h"
#include "transmit.h"
}

// 从 Neck 的直接 PDO 接口和反馈缓存提取，不依赖脖子运动学、补偿或日志。
extern std::atomic<bool> motor_bus_ready;
extern std::atomic<bool> inOP;
extern std::atomic<int> wkc;

struct MotorFeedback {
    double position_deg = 0;
    double velocity_deg_s = 0;
    double current_A = 0; // 沿用 Neck 对类型1第三量的命名，不换算为力矩。
    std::array<std::chrono::steady_clock::time_point, 3> observed{};
    double temperature = 0;
    int error = 0;
    int status = 0;
};

// phase随整帧一起交接，字符串需保持有效（调用方使用字符串字面量）。
std::uint64_t writeMotorDirect(const EtherCAT_Msg& frame, const char* phase = "control");
bool motorDirectPending();
void finishMotorDirect();
// 提交完整目标并等后台线程完成该帧的PDO交换；调用方不直接访问SOEM。
void exchangeMotorFrame(const EtherCAT_Msg& frame, const char* phase = "control");
// 可选周期观察接口：只在startBus之前注册、stopBus之后注销。
// 回调在收发线程完成反馈更新后执行，不能调用阻塞的总线接口。
using MotorCycleObserver = void (*)(const EtherCAT_Msg& sent, const char* phase, void* context);
void setMotorCycleObserver(MotorCycleObserver observer, void* context);
void recordMotorFeedback(int slave, const EtherCAT_Msg& raw, const OD_Motor_Msg decoded[6]);
MotorFeedback motorFeedback(int slave, std::uint16_t motor_id);
void clearMotorFeedback();
bool queryMotorPosition(int slave, int passage, std::uint16_t motor_id, double& position_deg);

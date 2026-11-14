#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include "motor_config.h"
#include "motor_bus.h"

namespace {
using Clock = std::chrono::steady_clock;
constexpr std::array<float, MOTOR_JOINT_COUNT> target_deg{17.61f, 12.53f, -2.905567f};
constexpr double knee_speed_deg_s = 115.0;
volatile std::sig_atomic_t quit = 0;

void request_exit(int) { quit = 1; }
}

int main(int argc, char** argv)
{
    if (argc > 1) {
        std::printf("用法: %s；网卡固定enp4s0，站立后保持，Ctrl+C退出。\n", argv[0]);
        return argc == 2 && (!std::strcmp(argv[1], "--help") || !std::strcmp(argv[1], "-h")) ? 0 : 1;
    }
    std::signal(SIGINT, request_exit);
    std::signal(SIGTERM, request_exit);
    if (!startBus()) {
        stopBus();
        return 1;
    }

    EtherCAT_Msg command{};
    auto tick = [&] { exchangeMotorFrame(command); };

    // 纯查询PDO读取初始位置，不与控制帧混装。
    std::array<double, MOTOR_JOINT_COUNT> start_deg{};
    std::puts("读取ID1/ID2/ID3当前角度，Ctrl+C退出。");
    for (int i = 0; i < MOTOR_JOINT_COUNT && !quit; ++i) {
        const auto& motor = MOTOR_JOINT_CONFIGS[i];
        while (!quit && !queryMotorPosition(0, motor.passage, motor.motor_id, start_deg[i])) {}
    }
    if (!quit) {
        std::array<double, MOTOR_JOINT_COUNT> delta_deg{};
        for (int i = 0; i < MOTOR_JOINT_COUNT; ++i)
            delta_deg[i] = static_cast<double>(target_deg[i]) - start_deg[i];
        double reference_deg = std::abs(delta_deg[0]);
        // 膝已在目标时，以另外两轴的最大位移作为时间基准。
        if (reference_deg == 0)
            reference_deg = std::max(std::abs(delta_deg[1]), std::abs(delta_deg[2]));
        const double duration_s = reference_deg / knee_speed_deg_s;
        std::array<uint16_t, MOTOR_JOINT_COUNT> raw_speed{};
        for (int i = 0; i < MOTOR_JOINT_COUNT; ++i) {
            const double speed_deg_s = duration_s == 0 ? 0 : std::abs(delta_deg[i]) / duration_s;
            // 位置模式速度单位0.1 rpm = 0.6 deg/s，向上取整使限速不拖慢插值。
            // 零位移轴给非零速度字段，目标仍固定在原位。
            raw_speed[i] = static_cast<uint16_t>(std::max(1.0, std::ceil(speed_deg_s / 0.6)));
            std::printf("ID%d: %.6f -> %.6f deg，规划速度 %.6f deg/s\n",
                        MOTOR_JOINT_CONFIGS[i].motor_id, start_deg[i], target_deg[i], speed_deg_s);
        }
        std::printf("同步移动时间 %.6f s；到位后保持，Ctrl+C退出。\n", duration_s);
        const auto begin = Clock::now();
        bool holding = false;
        while (!quit) {
            const double elapsed_s = std::chrono::duration<double>(Clock::now() - begin).count();
            const double progress = duration_s == 0 ? 1.0 : std::min(1.0, elapsed_s / duration_s);
            for (int i = 0; i < MOTOR_JOINT_COUNT; ++i) {
                const auto& motor = MOTOR_JOINT_CONFIGS[i];
                const float position_deg = progress == 1.0 ? target_deg[i] :
                    static_cast<float>(start_deg[i] + progress * delta_deg[i]);
                // 原生伺服位置模式；电流字段沿用现有程序的500（0.1 A）。
                set_motor_position(&command, motor.passage, motor.motor_id,
                                   position_deg, raw_speed[i], 500, 2);
            }
            tick(); // 三轴在同一PDO更新，规划终点也在同一周期下发。
            if (progress == 1.0 && !holding) {
                std::puts("插值完成，持续下发站立目标保持，Ctrl+C退出。");
                holding = true;
            }
        }
    }
    stopBus();
    return 0;
}

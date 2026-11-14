#include <array>
#include <csignal>
#include <cstdio>
#include <cstring>
#include "motor_config.h"
#include "motor_bus.h"

namespace {
volatile std::sig_atomic_t quit = 0;

void request_exit(int) { quit = 1; }
}

int main(int argc, char** argv)
{
    if (argc > 1) {
        std::printf("用法: %s；网卡固定enp4s0，读取ID1/ID2/ID3角度（度），打印后退出。\n", argv[0]);
        return argc == 2 && (!std::strcmp(argv[1], "--help") || !std::strcmp(argv[1], "-h")) ? 0 : 1;
    }
    std::signal(SIGINT, request_exit);
    std::signal(SIGTERM, request_exit);
    if (!startBus()) {
        stopBus();
        return 1;
    }

    // 只通过后台收发线程发送纯位置查询，不发送任何运动、使能、制动或调零指令。
    std::array<double, MOTOR_JOINT_COUNT> angles{};
    for (int i = 0; i < MOTOR_JOINT_COUNT && !quit; ++i) {
        const auto& motor = MOTOR_JOINT_CONFIGS[i];
        while (!quit && !queryMotorPosition(0, motor.passage, motor.motor_id, angles[i])) {}
    }
    if (!quit) {
        for (int i = 0; i < MOTOR_JOINT_COUNT; ++i)
            std::printf("ID%u: %.6f deg\n", static_cast<unsigned>(MOTOR_JOINT_CONFIGS[i].motor_id), angles[i]);
    }
    stopBus();
    return quit ? 1 : 0;
}

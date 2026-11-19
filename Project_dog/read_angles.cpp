#include <array>
#include <csignal>
#include <cstdio>
#include <cstring>
#include "motor_config.h"  // 电机ID与CAN通道配置
#include "motor_bus.h"     // 电机通信接口

namespace {
volatile std::sig_atomic_t quit = 0;
void request_exit(int) { quit = 1; }  // 退出标志
}

int main(int argc, char** argv)
{
    // 命令行参数检查
    if (argc > 1) {
        std::printf("用法: %s；网卡固定enp4s0，读取ID1/ID2/ID3角度（度），打印后退出。\n", argv[0]);
        return argc == 2 && (!std::strcmp(argv[1], "--help") || !std::strcmp(argv[1], "-h")) ? 0 : 1;
    }

    // 注册退出信号
    std::signal(SIGINT, request_exit);
    std::signal(SIGTERM, request_exit);

    // 初始化EtherCAT通信
    if (!startBus()) {
        stopBus();
        return 1;
    }

    std::array<double, MOTOR_JOINT_COUNT> angles{};  // 存储三个电机角度

    // 逐个查询电机位置
    for (int i = 0; i < MOTOR_JOINT_COUNT && !quit; ++i) {
        const auto& motor = MOTOR_JOINT_CONFIGS[i];

        // 从站索引、CAN通道、电机ID、角度输出
        // 查询失败则重试
        while (!quit && !queryMotorPosition(0, motor.passage, motor.motor_id, angles[i])) {}
    }

    // 打印查询结果
    if (!quit) {
        for (int i = 0; i < MOTOR_JOINT_COUNT; ++i)
            std::printf("ID%u: %.6f deg\n", static_cast<unsigned>(MOTOR_JOINT_CONFIGS[i].motor_id), angles[i]);
    }

    stopBus();  // 关闭通信
    return quit ? 1 : 0;
}
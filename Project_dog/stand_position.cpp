/*
 * 功能：读取三个电机的初始角度，规划三轴同步运动到站立姿态，
 *       到位后持续发送目标位置，保持站立。
 *
 * 流程：初始化通信 → 读取初始角度 → 计算运动时间和速度
 *       → 位置插值 → 持续下发控制指令 → 退出时关闭通信。
 */

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
using Clock = std::chrono::steady_clock;  // 单调时钟，用于运动计时
constexpr std::array<float, MOTOR_JOINT_COUNT> target_deg{17.61f, 12.53f, -2.905567f}; // 三轴站立目标角度
constexpr double knee_speed_deg_s = 115.0;  // ID1膝关节参考速度（度/秒）
volatile std::sig_atomic_t quit = 0;

void request_exit(int) { quit = 1; }
}

int main(int argc, char** argv)
{
    // 检查命令行参数
    if (argc > 1) {
        std::printf("用法: %s；网卡固定enp4s0，站立后保持，Ctrl+C退出。\n", argv[0]);
        return argc == 2 && (!std::strcmp(argv[1], "--help") || !std::strcmp(argv[1], "-h")) ? 0 : 1;
    }

    // 注册退出信号
    std::signal(SIGINT, request_exit);
    std::signal(SIGTERM, request_exit);

    // 启动 EtherCAT 通信
    if (!startBus()) {
        stopBus();
        return 1;
    }

    EtherCAT_Msg command{};  // 存储电机控制报文
    auto tick = [&] { exchangeMotorFrame(command); };  // 提交完整控制帧并等待一次PDO交换

    // 查询三个电机的初始角度
    std::array<double, MOTOR_JOINT_COUNT> start_deg{};
    std::puts("读取ID1/ID2/ID3当前角度，Ctrl+C退出。");
    for (int i = 0; i < MOTOR_JOINT_COUNT && !quit; ++i) {
        const auto& motor = MOTOR_JOINT_CONFIGS[i];
        while (!quit && !queryMotorPosition(0, motor.passage, motor.motor_id, start_deg[i])) {}
    }

    if (!quit) {
        // 计算各电机需要转动的角度
        std::array<double, MOTOR_JOINT_COUNT> delta_deg{};
        for (int i = 0; i < MOTOR_JOINT_COUNT; ++i)
            delta_deg[i] = static_cast<double>(target_deg[i]) - start_deg[i];

        // 以膝关节位移确定运动时间，膝不动时取其他轴的最大位移
        double reference_deg = std::abs(delta_deg[0]);
        if (reference_deg == 0)
            reference_deg = std::max(std::abs(delta_deg[1]), std::abs(delta_deg[2]));

        const double duration_s = reference_deg / knee_speed_deg_s;  // 总运动时间

        // 计算各电机速度，使三轴同步到达目标
        std::array<uint16_t, MOTOR_JOINT_COUNT> raw_speed{};
        for (int i = 0; i < MOTOR_JOINT_COUNT; ++i) {
            const double speed_deg_s = duration_s == 0 ? 0 : std::abs(delta_deg[i]) / duration_s;

            // 速度单位转换：0.1 rpm = 0.6 deg/s
            raw_speed[i] = static_cast<uint16_t>(std::max(1.0, std::ceil(speed_deg_s / 0.6)));

            std::printf("ID%d: %.6f -> %.6f deg，规划速度 %.6f deg/s\n",
                        MOTOR_JOINT_CONFIGS[i].motor_id, start_deg[i], target_deg[i], speed_deg_s);
        }

        std::printf("同步移动时间 %.6f s；到位后保持，Ctrl+C退出。\n", duration_s);

        const auto begin = Clock::now();  // 记录运动开始时间
        bool holding = false;

        // 持续计算并下发三个电机的位置指令
        while (!quit) {
            const double elapsed_s = std::chrono::duration<double>(Clock::now() - begin).count();

            // 运动进度：0为起点，1为终点
            const double progress = duration_s == 0 ? 1.0 : std::min(1.0, elapsed_s / duration_s);

            for (int i = 0; i < MOTOR_JOINT_COUNT; ++i) {
                const auto& motor = MOTOR_JOINT_CONFIGS[i];

                // 线性插值，计算当前目标角度
                const float position_deg = progress == 1.0 ? target_deg[i] :
                    static_cast<float>(start_deg[i] + progress * delta_deg[i]);

                // 构造位置模式报文：角度、速度限制、电流限制、反馈类型
                set_motor_position(&command, motor.passage, motor.motor_id,
                                   position_deg, raw_speed[i], 500, 2);
            }

            tick();  // 三轴指令通过同一次PDO交换下发

            // 到位后持续保持站立
            if (progress == 1.0 && !holding) {
                std::puts("插值完成，持续下发站立目标保持，Ctrl+C退出。");
                holding = true;
            }
        }
    }

    stopBus();  // 关闭 EtherCAT 通信
    return 0;
}
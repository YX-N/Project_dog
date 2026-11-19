
/*
 * 功能：通过位置控制模式完成单腿一次跳跃，最后保持站立。
 * 流程：读取初始角度 → 站立 → 下蹲 → 蹬伸 → 收腿 → 落地
 *       → 缓冲下蹲 → 恢复站立 → 持续保持。
 * 控制方式：三轴同步发送位置指令，根据电机反馈判断是否到位。
 */

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <exception>
#include <string>
#include "motor_config.h"
#include "motor_bus.h"

namespace {
using Clock = std::chrono::steady_clock;
using Pose = std::array<float, MOTOR_JOINT_COUNT>;

// 各运动阶段的目标角度，顺序：[ID1膝、ID2大腿、ID3]
constexpr float hip_deg = -2.905567f;
constexpr Pose stand {17.61f, 12.53f, hip_deg};  // 站立
constexpr Pose crouch{57.57f, 26.10f, hip_deg};  // 下蹲
constexpr Pose push  { 6.10f,  8.59f, hip_deg};  // 蹬伸
constexpr Pose tuck  {28.37f, 16.19f, hip_deg};  // 收腿
constexpr Pose land  {11.96f, 10.60f, hip_deg};  // 落地

constexpr double slow_deg_s = 115.0;  // 慢速运动速度
constexpr double fast_deg_s = 460.0;  // 快速运动速度
constexpr float arrival_deg = 0.5f;   // 到位允许误差（度）

volatile std::sig_atomic_t quit = 0;
void request_exit(int) { quit = 1; }

// 将角速度 deg/s 转换为驱动器速度字段（0.1 rpm）
uint16_t speed_raw(double deg_s)
{
    return static_cast<uint16_t>(std::max(1L, std::lround(deg_s / 0.6)));
}

// 封装单腿的控制指令、反馈和运动操作
struct Leg {
    EtherCAT_Msg command{};      // 电机控制PDO数据
    Pose actual{};               // 三个电机的实际角度
    Clock::time_point last_move{}; // 上一次动作开始时间

    // 发送一帧控制指令，并更新实际角度
    void tick() {
        exchangeMotorFrame(command);
        for (int i = 0; i < MOTOR_JOINT_COUNT; ++i) {
            const auto value = motorFeedback(0, MOTOR_JOINT_CONFIGS[i].motor_id);
            if (value.observed[0] != Clock::time_point{})
                actual[i] = value.position_deg;
        }
    }

    // 主动查询三个电机的初始角度
    void sample() {
        for (int i = 0; i < MOTOR_JOINT_COUNT && !quit; ++i) {
            const auto& motor = MOTOR_JOINT_CONFIGS[i];
            double angle = 0;
            while (!quit && !queryMotorPosition(0, motor.passage, motor.motor_id, angle)) {}
            actual[i] = angle;
        }
    }

    // 切换到指定姿态：计算各轴速度并一次下发完整目标
    void move(const char* name, const Pose& target, double knee_deg_s) {
        if (quit) return;

        // 根据实测位置计算膝、大腿到目标的剩余位移
        const double knee_distance = std::abs(static_cast<double>(target[0]) - actual[0]);
        const double thigh_distance = std::abs(static_cast<double>(target[1]) - actual[1]);

        // 按位移比例分配速度，使膝与大腿同步到位
        const double thigh_deg_s = knee_distance == 0 ? knee_deg_s :
            thigh_distance / knee_distance * knee_deg_s;
        const double speeds[] = {knee_deg_s, thigh_deg_s, slow_deg_s};

        for (int i = 0; i < MOTOR_JOINT_COUNT; ++i) {
            const auto& motor = MOTOR_JOINT_CONFIGS[i];

            // 构造位置控制报文：目标角度、速度、电流限制、反馈类型
            set_motor_position(&command, motor.passage, motor.motor_id,
                               target[i], speed_raw(speeds[i]), 500, 2);
        }

        std::printf("%s: ID1 %.6f deg @ %.3f deg/s，ID2 %.6f deg @ %.3f deg/s\n",
                    name, target[0], knee_deg_s, target[1], thigh_deg_s);

        last_move = Clock::now();
        tick();  // 下发新目标；后续周期保持同一目标
    }

    // 落地缓冲：从当前姿态减速运动到下蹲姿态
    void land_to_crouch(double v_land0) {
        if (quit) return;

        const Pose start = actual;
        const double knee_distance = std::abs(static_cast<double>(crouch[0]) - start[0]);
        const double thigh_distance = std::abs(static_cast<double>(crouch[1]) - start[1]);

        // 按匀减速运动计算缓冲时间：T = 2S / V0
        const double distance = knee_distance > 0 ? knee_distance : thigh_distance;
        const double duration = 2.0 * distance / v_land0;

        const double thigh_speed = knee_distance > 0 ?
            v_land0 * thigh_distance / knee_distance : v_land0;
        const double speeds[] = {v_land0, thigh_speed};

        const auto begin = Clock::now();

        while (!quit) {
            const double elapsed = std::chrono::duration<double>(Clock::now() - begin).count();
            const double u = duration > 0 ? std::min(elapsed / duration, 1.0) : 1.0;

            // 匀减速位置插值，末端速度趋于0
            const double progress = u * (2.0 - u);

            // 仅更新膝和大腿，ID3保持原指令
            for (int i = 0; i < 2; ++i) {
                const auto& motor = MOTOR_JOINT_CONFIGS[i];
                const float position = u >= 1.0 ? crouch[i] :
                    static_cast<float>(start[i] + (static_cast<double>(crouch[i]) - start[i]) * progress);

                set_motor_position(&command, motor.passage, motor.motor_id,
                                   position, speed_raw(speeds[i]), 500, 2);
            }

            tick();
            if (u >= 1.0) break;
        }
    }

    // 在指定时间之前持续发送当前控制指令
    void hold_until(Clock::time_point end) {
        while (!quit && Clock::now() < end) tick();
    }

    // 根据实际角度判断三轴是否全部到位
    void wait_arrival(const Pose& target) {
        while (!quit) {
            bool arrived = true;
            for (int i = 0; i < MOTOR_JOINT_COUNT; ++i)
                arrived &= std::abs(actual[i] - target[i]) < arrival_deg;

            if (arrived) return;
            tick();
        }
    }

    // 慢速运动到目标，并等待实际到位
    void slow(const char* name, const Pose& target) {
        move(name, target, slow_deg_s);
        wait_arrival(target);
    }
};
}

int main(int argc, char** argv)
{
    // 检查命令行参数并输出使用说明
    if (argc > 3 || (argc > 1 && (!std::strcmp(argv[1], "--help") || !std::strcmp(argv[1], "-h")))) {
        std::printf("用法: %s [V_PUSH=340(deg/s)] [V_LAND0=400(deg/s)]\n"
                    "网卡固定enp4s0，执行一次跳跃，最后保持stand，Ctrl+C退出。\n", argv[0]);
        return argc > 3 ? 1 : 0;
    }

    // 蹬伸速度，默认340 deg/s，可通过命令行修改
    double v_push = 340.0;
    try {
        if (argc > 1) v_push = std::stod(argv[1]);
    } catch (const std::exception& error) {
        std::fprintf(stderr, "V_PUSH参数错误: %s\n", error.what());
        return 1;
    }

    // 落地缓冲初速度，默认400 deg/s
    double v_land0 = 400.0;
    try {
        if (argc > 2) v_land0 = std::stod(argv[2]);
        if (!std::isfinite(v_land0) || v_land0 <= 0) {
            std::fprintf(stderr, "V_LAND0必须为有限正数\n");
            return 1;
        }
    } catch (const std::exception& error) {
        std::fprintf(stderr, "V_LAND0参数错误: %s\n", error.what());
        return 1;
    }

    // 注册退出信号
    std::signal(SIGINT, request_exit);
    std::signal(SIGTERM, request_exit);

    // 初始化EtherCAT通信
    if (!startBus()) {
        stopBus();
        return 1;
    }

    Leg leg;
    std::puts("读取当前角度；慢速段到位容差0.5度；Ctrl+C退出。");

    // 第一阶段：读取初始位置，慢速站立、下蹲
    leg.sample();
    leg.slow("stand", stand);
    leg.slow("crouch", crouch);
    leg.hold_until(Clock::now() + std::chrono::milliseconds(300));

    // 第二阶段：快速蹬伸，膝关节接近目标3度时切换收腿
    leg.move("push", push, v_push);
    while (!quit && std::abs(leg.actual[0] - push[0]) >= 3.0f) leg.tick();

    // 第三阶段：收腿，80ms后切换落地姿态
    leg.move("tuck", tuck, fast_deg_s);
    leg.hold_until(leg.last_move + std::chrono::milliseconds(80));
    leg.move("land", land, fast_deg_s);
    leg.wait_arrival(land);

    // 第四阶段：落地缓冲，减速下蹲
    leg.land_to_crouch(v_land0);
    leg.wait_arrival(crouch);
    leg.hold_until(Clock::now() + std::chrono::milliseconds(300));

    // 第五阶段：恢复站立并持续保持
    leg.slow("stand", stand);
    if (!quit) std::puts("跳跃完成，持续保持stand，Ctrl+C退出。");
    while (!quit) leg.tick();

    stopBus();
    return 0;
}

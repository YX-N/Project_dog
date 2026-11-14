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
constexpr float hip_deg = -2.905567f;
// 数组顺序为 [ID1膝, ID2大腿, ID3]；所有位置均为驱动器原始角度（度）。
constexpr Pose stand {17.61f, 12.53f, hip_deg};
constexpr Pose crouch{57.57f, 26.10f, hip_deg};
constexpr Pose push  { 6.10f,  8.59f, hip_deg};
constexpr Pose tuck  {28.37f, 16.19f, hip_deg};
constexpr Pose land  {11.96f, 10.60f, hip_deg};
constexpr double slow_deg_s = 115.0;
constexpr double fast_deg_s = 460.0;
constexpr float arrival_deg = 0.5f;
volatile std::sig_atomic_t quit = 0;

void request_exit(int) { quit = 1; }

uint16_t speed_raw(double deg_s)
{
    // 位置模式速度字段：0.1 rpm = 0.6 deg/s，取最近的协议值。
    // 零位移时仍给非零速度字段，位置目标保持不变。
    return static_cast<uint16_t>(std::max(1L, std::lround(deg_s / 0.6)));
}

struct Leg {
    EtherCAT_Msg command{};
    Pose actual{};
    Clock::time_point last_move{};

    void tick() {
        exchangeMotorFrame(command);
        for (int i = 0; i < MOTOR_JOINT_COUNT; ++i) {
            const auto value = motorFeedback(0, MOTOR_JOINT_CONFIGS[i].motor_id);
            if (value.observed[0] != Clock::time_point{}) actual[i] = value.position_deg;
        }
    }

    void sample() {
        for (int i = 0; i < MOTOR_JOINT_COUNT && !quit; ++i) {
            const auto& motor = MOTOR_JOINT_CONFIGS[i];
            double angle = 0;
            while (!quit && !queryMotorPosition(0, motor.passage, motor.motor_id, angle)) {}
            actual[i] = angle;
        }
    }

    void move(const char* name, const Pose& target, double knee_deg_s) {
        if (quit) return;
        // 使用切换时的实测位置；push提前切到tuck、定时切land时也按剩余位移同步。
        const double knee_distance = std::abs(static_cast<double>(target[0]) - actual[0]);
        const double thigh_distance = std::abs(static_cast<double>(target[1]) - actual[1]);
        // 膝无位移时，大腿单独使用给定速度，避免除零。
        const double thigh_deg_s = knee_distance == 0 ? knee_deg_s :
            thigh_distance / knee_distance * knee_deg_s;
        const double speeds[] = {knee_deg_s, thigh_deg_s, slow_deg_s};
        for (int i = 0; i < MOTOR_JOINT_COUNT; ++i) {
            const auto& motor = MOTOR_JOINT_CONFIGS[i];
            // 电流字段沿用现有程序的500（单位0.1 A），类型2反馈位置。
            set_motor_position(&command, motor.passage, motor.motor_id,
                               target[i], speed_raw(speeds[i]), 500, 2);
        }
        std::printf("%s: ID1 %.6f deg @ %.3f deg/s，ID2 %.6f deg @ %.3f deg/s\n",
                    name, target[0], knee_deg_s, target[1], thigh_deg_s);
        last_move = Clock::now();
        tick(); // 同一PDO一次切换完整目标；后续周期只重发相同目标，不插值。
    }

    void land_to_crouch(double v_land0) {
        if (quit) return;
        const Pose start = actual;
        const double knee_distance = std::abs(static_cast<double>(crouch[0]) - start[0]);
        const double thigh_distance = std::abs(static_cast<double>(crouch[1]) - start[1]);
        // v(t) = V_LAND0 * (1 - t/T)，T = 2 * 距离 / V_LAND0。
        // 膝无位移时按大腿位移计时，避免除零。
        const double distance = knee_distance > 0 ? knee_distance : thigh_distance;
        const double duration = 2.0 * distance / v_land0;
        const double thigh_speed = knee_distance > 0 ?
            v_land0 * thigh_distance / knee_distance : v_land0;
        const double speeds[] = {v_land0, thigh_speed};
        const auto begin = Clock::now();
        while (!quit) {
            const double elapsed = std::chrono::duration<double>(Clock::now() - begin).count();
            const double u = duration > 0 ? std::min(elapsed / duration, 1.0) : 1.0;
            const double progress = u * (2.0 - u);
            for (int i = 0; i < 2; ++i) {
                const auto& motor = MOTOR_JOINT_CONFIGS[i];
                const float position = u >= 1.0 ? crouch[i] :
                    static_cast<float>(start[i] + (static_cast<double>(crouch[i]) - start[i]) * progress);
                // 速度字段保留初速度上限；减速由逐周期位置插值实现，
                // 避免末端零限速阻止到位。ID3保持原指令不变。
                set_motor_position(&command, motor.passage, motor.motor_id,
                                   position, speed_raw(speeds[i]), 500, 2);
            }
            tick();
            if (u >= 1.0) break; // 精确下发crouch，轨迹末速度为零。
        }
    }

    void hold_until(Clock::time_point end) {
        while (!quit && Clock::now() < end) tick();
    }

    void wait_arrival(const Pose& target) {
        while (!quit) {
            bool arrived = true;
            for (int i = 0; i < MOTOR_JOINT_COUNT; ++i)
                arrived &= std::abs(actual[i] - target[i]) < arrival_deg;
            if (arrived) return;
            tick();
        }
    }

    void slow(const char* name, const Pose& target) {
        move(name, target, slow_deg_s);
        wait_arrival(target);
    }
};
}

int main(int argc, char** argv)
{
    if (argc > 3 || (argc > 1 && (!std::strcmp(argv[1], "--help") || !std::strcmp(argv[1], "-h")))) {
        std::printf("用法: %s [V_PUSH=340(deg/s)] [V_LAND0=400(deg/s)]\n"
                    "网卡固定enp4s0，执行一次跳跃，最后保持stand，Ctrl+C退出。\n", argv[0]);
        return argc > 3 ? 1 : 0;
    }
    double v_push = 340.0;
    try {
        if (argc > 1) v_push = std::stod(argv[1]);
    } catch (const std::exception& error) {
        std::fprintf(stderr, "V_PUSH参数错误: %s\n", error.what());
        return 1;
    }
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
    std::signal(SIGINT, request_exit);
    std::signal(SIGTERM, request_exit);
    if (!startBus()) {
        stopBus();
        return 1;
    }

    Leg leg;
    std::puts("读取当前角度；慢速段到位容差0.5度；Ctrl+C退出。");
    leg.sample();
    leg.slow("stand", stand);
    leg.slow("crouch", crouch);
    leg.hold_until(Clock::now() + std::chrono::milliseconds(300));
    leg.move("push", push, v_push);
    while (!quit && std::abs(leg.actual[0] - push[0]) >= 3.0f) leg.tick();
    leg.move("tuck", tuck, fast_deg_s);
    leg.hold_until(leg.last_move + std::chrono::milliseconds(80));
    leg.move("land", land, fast_deg_s);
    leg.wait_arrival(land);
    leg.land_to_crouch(v_land0);
    leg.wait_arrival(crouch);
    leg.hold_until(Clock::now() + std::chrono::milliseconds(300));
    leg.slow("stand", stand);
    if (!quit) std::puts("跳跃完成，持续保持stand，Ctrl+C退出。");
    while (!quit) leg.tick();

    stopBus();
    return 0;
}

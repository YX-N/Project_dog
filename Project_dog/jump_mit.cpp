#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <exception>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <locale>
#include <sstream>
#include <string>
#include <stdexcept>
#include "motor_config.h"
#include "motor_bus.h"

namespace {
using Clock = std::chrono::steady_clock;
using Pose = std::array<double, MOTOR_JOINT_COUNT>;
constexpr double rad_per_deg = 3.14159265358979323846 / 180.0;
constexpr double hip_deg = -2.905567;
// 数组顺序：[ID1膝, ID2大腿, ID3]，内部位置和速度使用度、度/秒。
constexpr Pose stand {17.61, 12.53, hip_deg};
constexpr Pose crouch{57.57, 26.10, hip_deg};
constexpr Pose push  {-13.20, 1.99, hip_deg};
constexpr Pose land  {11.96, 10.60, hip_deg};
constexpr Pose land_deep{71.00, 30.61, hip_deg};
constexpr double slow_deg_s = 115.0;
constexpr double gain_transition_s = 0.030;
constexpr double push_ramp_s = 0.040;
constexpr double flight_duration_s = 0.090;
enum class Profile { Linear, Quintic };
struct Gains { double kp, kd; };
constexpr Gains normal{80.0, 2.0};
constexpr Gains pushing{250.0, 3.0};
volatile std::sig_atomic_t quit = 0;

void request_exit(int) { quit = 1; }

struct CycleLog {
    struct Row {
        double t;
        const char* phase;
        // 每轴：pos_cmd, vel_cmd, kp, kd, pos_act, vel_act, cur。
        std::array<std::array<double, 7>, 2> motors;
    };
    Clock::time_point start;
    std::chrono::system_clock::time_point wall_start;
    std::deque<Row> rows; // 仅收发线程追加，避免扩容时复制全部历史数据。

    CycleLog(Clock::time_point start, std::chrono::system_clock::time_point wall_start)
        : start(start), wall_start(wall_start) {}

    static void observe(const EtherCAT_Msg& sent, const char* phase, void* context) {
        auto& log = *static_cast<CycleLog*>(context);
        Row row{};
        row.t = std::chrono::duration<double>(Clock::now() - log.start).count();
        row.phase = phase;
        for (int i = 0; i < 2; ++i) {
            auto& out = row.motors[i];
            out.fill(std::numeric_limits<double>::quiet_NaN());
            const auto id = MOTOR_JOINT_CONFIGS[i].motor_id;
            // 从该周期实际提交的MIT报文反解命令，包含协议限幅和量化。
            if (sent.can_ide == 1) {
                for (const auto& motor : sent.motor) {
                    if (motor.id != id || motor.dlc != 8 || (motor.data[0] >> 5) != 0) continue;
                    const auto* d = motor.data;
                    out[0] = uint_to_float((d[3] << 8) | d[4], -12.5f, 12.5f, 16) / rad_per_deg;
                    out[1] = uint_to_float((d[5] << 4) | (d[6] >> 4), -18, 18, 12) / rad_per_deg;
                    out[2] = uint_to_float(((d[0] & 0x1f) << 7) | (d[1] >> 1), 0, 500, 12);
                    out[3] = uint_to_float(((d[1] & 1) << 8) | d[2], 0, 5, 9);
                    break;
                }
            }
            const auto feedback = motorFeedback(0, id);
            if (feedback.observed[0] != Clock::time_point{}) out[4] = feedback.position_deg;
            if (feedback.observed[1] != Clock::time_point{}) out[5] = feedback.velocity_deg_s;
            if (feedback.observed[2] != Clock::time_point{}) out[6] = feedback.current_A;
        }
        log.rows.push_back(row);
    }

    bool save(double v_push) const {
        try {
            const auto stamp = std::chrono::system_clock::to_time_t(wall_start);
            std::tm local{};
            localtime_r(&stamp, &local);
            const auto micros = std::chrono::duration_cast<std::chrono::microseconds>(
                wall_start.time_since_epoch()).count() % 1000000;
            std::ostringstream name;
            name.imbue(std::locale::classic());
            name << "logs/jump_" << std::put_time(&local, "%Y%m%d_%H%M%S") << '_'
                 << std::setfill('0') << std::setw(6) << micros << "_V_PUSH"
                 << std::setprecision(15) << v_push << ".csv";
            std::filesystem::create_directories("logs");
            std::ofstream csv;
            csv.exceptions(std::ios::failbit | std::ios::badbit);
            csv.open(name.str());
            csv.imbue(std::locale::classic());
            csv << "t,phase";
            for (int id : {1, 2}) {
                for (const char* field : {"pos_cmd", "vel_cmd", "kp", "kd", "pos_act", "vel_act", "cur"})
                    csv << ",ID" << id << '_' << field;
            }
            csv << '\n' << std::fixed << std::setprecision(9);
            for (const auto& row : rows) {
                csv << row.t << ',' << row.phase;
                for (const auto& motor : row.motors)
                    for (double value : motor) csv << ',' << value;
                csv << '\n';
            }
            csv.close();
            std::printf("CSV已保存：%s（%zu行）\n", name.str().c_str(), rows.size());
            return true;
        } catch (const std::exception& error) {
            std::fprintf(stderr, "CSV保存失败: %s\n", error.what());
            return false;
        }
    }
};

struct Leg {
    EtherCAT_Msg command{};
    Pose commanded{0.0, 0.0, hip_deg};
    Pose commanded_velocity{};
    const char* phase = "sample";
    double tor_ff_stand, tor_ff_crouch, tor_fric;
    bool in_flight = false;
    Gains gain_from{0.0, 0.0}, gain_target{0.0, 0.0};
    Clock::time_point gain_begin = Clock::now();
    double gain_duration = gain_transition_s;
    bool ff_ramping = false;
    Clock::time_point ff_begin{};

    Leg(double tor_ff_stand, double tor_ff_crouch, double tor_fric)
        : tor_ff_stand(tor_ff_stand), tor_ff_crouch(tor_ff_crouch), tor_fric(tor_fric) {}

    Gains gains_at(Clock::time_point now) const {
        const double elapsed = std::chrono::duration<double>(now - gain_begin).count();
        const double blend = std::clamp(elapsed / gain_duration, 0.0, 1.0);
        return {gain_from.kp + (gain_target.kp - gain_from.kp) * blend,
                gain_from.kd + (gain_target.kd - gain_from.kd) * blend};
    }

    void tick() {
        // 即使只保持目标，也每周期更新增益渐变；ID3仍全程使用固定80/2。
        const auto now = Clock::now();
        const Gains gains = gains_at(now);
        const double k = std::clamp((commanded[0] - stand[0]) / (crouch[0] - stand[0]), 0.0, 1.0);
        const bool disable_ff = in_flight || !std::strcmp(phase, "land_arm") ||
                                !std::strcmp(phase, "land_wait");
        // 触地后50ms内按线性系数恢复随姿态变化的重力前馈。
        const double ff_blend = ff_ramping ? std::clamp(
            std::chrono::duration<double>(now - ff_begin).count() / 0.050, 0.0, 1.0) : 1.0;
        const double knee_tor = disable_ff ? 0.0 : ff_blend *
            (tor_ff_stand + (tor_ff_crouch - tor_ff_stand) * k);
        for (int i = 0; i < MOTOR_JOINT_COUNT; ++i) {
            const auto& motor = MOTOR_JOINT_CONFIGS[i];
            const Gains g = i == 2 ? normal : gains;
            const double pos = i == 2 ? hip_deg : commanded[i];
            const double spd = i == 2 ? 0.0 : commanded_velocity[i];
            double tor = i == 0 ? knee_tor : 0.0;
            // 摩擦补偿独立于重力前馈，空中运动也生效；ID3不补偿。
            if (i < 2 && std::abs(spd) >= 5.0) tor += tor_fric * (spd > 0.0 ? 1.0 : -1.0);
            send_motor_ctrl_cmd(&command, motor.passage, motor.motor_id,
                                g.kp, g.kd, pos * rad_per_deg, spd * rad_per_deg, tor);
        }
        exchangeMotorFrame(command, phase);
    }

    void sample() {
        // 先独立查询初始位置，再开始三轴纯MIT控制，不能与ID3指令混装。
        for (int i = 0; i < 2 && !quit; ++i) {
            const auto& motor = MOTOR_JOINT_CONFIGS[i];
            while (!quit && !queryMotorPosition(0, motor.passage, motor.motor_id, commanded[i])) {}
        }
    }

    void set(const Pose& position, const Pose& velocity, Gains gains,
             double transition_s = gain_transition_s) {
        if (gains.kp != gain_target.kp || gains.kd != gain_target.kd) {
            const auto now = Clock::now();
            gain_from = gains_at(now);
            gain_target = gains;
            gain_begin = now;
            gain_duration = transition_s;
        }
        commanded = position;
        commanded[2] = hip_deg;
        commanded_velocity = velocity;
        commanded_velocity[2] = 0.0;
    }

    void move(const char* name, const Pose& target, double knee_deg_s, Gains gains,
              Profile profile = Profile::Linear, double fixed_duration = -1.0,
              double transition_s = gain_transition_s) {
        if (quit) return;
        phase = name;
        in_flight = false;
        // 保留原段时长；五次曲线改变速度分布，不再将115deg/s视为峰值。
        const Pose start = commanded;
        const double knee_distance = std::abs(target[0] - start[0]);
        const double thigh_distance = std::abs(target[1] - start[1]);
        const double duration = fixed_duration >= 0.0 ? fixed_duration :
            (knee_distance > 0.0 ? knee_distance : thigh_distance) / knee_deg_s;
        std::printf("%s: ID1 %.2f deg，ID2 %.2f deg，时长 %.6f s，Kp %.1f Kd %.1f\n",
                    name, target[0], target[1], duration, gains.kp, gains.kd);
        set(start, Pose{}, gains, transition_s);
        const auto begin = Clock::now();
        while (!quit) {
            const double elapsed = std::chrono::duration<double>(Clock::now() - begin).count();
            const double u = duration > 0.0 ? std::min(elapsed / duration, 1.0) : 1.0;
            double progress = u, derivative = 1.0;
            if (profile == Profile::Quintic) {
                // s(u)=10u^3-15u^4+6u^5；两端速度、加速度均为0。
                progress = u * u * u * (10.0 + u * (-15.0 + 6.0 * u));
                derivative = 30.0 * u * u * (1.0 - u) * (1.0 - u);
            }
            Pose position = target;
            Pose velocity{};
            for (int i = 0; i < 2; ++i) {
                if (u < 1.0) {
                    const double delta = target[i] - start[i];
                    position[i] = start[i] + delta * progress;
                    velocity[i] = delta / duration * derivative;
                }
            }
            set(position, velocity, gains, transition_s);
            tick();
            if (u >= 1.0) return;
        }
    }

    void push_off(double v_push) {
        if (quit) return;
        phase = "push";
        in_flight = false;
        const Pose start = commanded;
        const double knee_distance = std::abs(push[0] - start[0]);
        const double thigh_distance = std::abs(push[1] - start[1]);
        const double distance = knee_distance > 0.0 ? knee_distance : thigh_distance;
        const double ramp_distance = 0.5 * v_push * push_ramp_s;
        // 若目标落在加速段内，则在目标处截断；正常行程为加速40ms后匀速。
        const double duration = distance <= ramp_distance ?
            std::sqrt(2.0 * distance * push_ramp_s / v_push) :
            push_ramp_s + (distance - ramp_distance) / v_push;
        std::printf("push: ID1 %.2f deg，ID2 %.2f deg，V_PUSH %.1f deg/s，时长 %.6f s\n",
                    push[0], push[1], v_push, duration);
        const auto begin = Clock::now();
        while (!quit) {
            const double elapsed = std::chrono::duration<double>(Clock::now() - begin).count();
            const bool finished = distance == 0.0 || elapsed >= duration;
            Pose position = push;
            Pose velocity{};
            if (!finished) {
                const bool accelerating = elapsed < push_ramp_s;
                const double speed = accelerating ? v_push * elapsed / push_ramp_s : v_push;
                const double travel = accelerating ? 0.5 * speed * elapsed :
                    ramp_distance + v_push * (elapsed - push_ramp_s);
                for (int i = 0; i < 2; ++i) {
                    const double ratio = (push[i] - start[i]) / distance;
                    position[i] = start[i] + ratio * travel;
                    velocity[i] = ratio * speed;
                }
            }
            set(position, velocity, pushing);
            tick();
            if (finished) return;
        }
    }

    void move_from_feedback(const char* name, const Pose& target, double duration,
                            Gains gains, bool flying) {
        if (quit) return;
        phase = name;
        in_flight = flying;
        Pose start{}, initial_velocity{};
        for (int i = 0; i < 2; ++i) {
            const auto feedback = motorFeedback(0, MOTOR_JOINT_CONFIGS[i].motor_id);
            start[i] = feedback.position_deg;
            initial_velocity[i] = feedback.velocity_deg_s;
        }
        const auto begin = Clock::now();
        while (!quit) {
            const double elapsed = std::chrono::duration<double>(Clock::now() - begin).count();
            const double u = duration > 0.0 ? std::min(elapsed / duration, 1.0) : 1.0;
            Pose position = target;
            Pose velocity{};
            for (int i = 0; i < 2; ++i) {
                if (u < 1.0) {
                    // 五次Hermite：起点为实测q/v，终点速度为0，两端加速度取0。
                    const double delta = target[i] - start[i];
                    const double b1 = initial_velocity[i] * duration;
                    const double b3 = 10.0 * delta - 6.0 * b1;
                    const double b4 = -15.0 * delta + 8.0 * b1;
                    const double b5 = 6.0 * delta - 3.0 * b1;
                    position[i] = start[i] + b1 * u + u * u * u * (b3 + u * (b4 + u * b5));
                    velocity[i] = (b1 + u * u * (3.0 * b3 + u * (4.0 * b4 + u * 5.0 * b5))) /
                                  duration;
                }
            }
            set(position, velocity, gains);
            tick();
            if (u >= 1.0) return;
        }
    }

    void fly_to_land(Gains landing) {
        move_from_feedback("land", land, flight_duration_s, landing, true);
    }

    void land_arm(Gains landing, double arm_deg) {
        if (quit) return;
        phase = "land_arm";
        in_flight = false;
        set(land, Pose{}, landing);
        bool within_tolerance = false;
        Clock::time_point stable_since{};
        while (!quit) {
            tick();
            const auto feedback = motorFeedback(0, MOTOR_JOINT_CONFIGS[0].motor_id);
            const auto now = Clock::now();
            if (std::abs(feedback.position_deg - commanded[0]) < arm_deg &&
                std::abs(feedback.velocity_deg_s) < 100.0) {
                if (!within_tolerance) {
                    stable_since = now;
                    within_tolerance = true;
                }
                if (now - stable_since >= std::chrono::milliseconds(20)) return;
            } else {
                within_tolerance = false; // 位置或速度条件不满足时重新计时。
            }
        }
    }

    void land_wait(Gains landing, double td_deg, double td_vel, bool timeout_enabled) {
        if (quit) return;
        phase = "land_wait";
        in_flight = false;
        set(land, Pose{}, landing);
        std::puts("已就绪");
        std::fflush(stdout);
        const auto begin = Clock::now();
        while (!quit) {
            tick();
            const auto feedback = motorFeedback(0, MOTOR_JOINT_CONFIGS[0].motor_id);
            // 正向位置偏差或正向弯曲速度任一超阈值即触地，不取绝对值。
            // 仅完整跳跃允许0.3秒超时，drop一直等触地。
            if (feedback.position_deg - commanded[0] > td_deg ||
                feedback.velocity_deg_s > td_vel ||
                (timeout_enabled && Clock::now() - begin > std::chrono::milliseconds(300))) return;
        }
    }

    void absorb(double v_abs0, Gains gains) {
        if (quit) return;
        phase = "absorb";
        in_flight = false;
        Pose start{};
        for (int i = 0; i < 2; ++i)
            start[i] = motorFeedback(0, MOTOR_JOINT_CONFIGS[i].motor_id).position_deg;
        // 时间由ID1到最深点的距离确定；两轴共用T，不使用实测初速度。
        const double duration = 3.0 * std::abs(land_deep[0] - start[0]) / v_abs0;
        const auto begin = Clock::now();
        ff_ramping = true;
        ff_begin = begin;
        while (!quit) {
            const double elapsed = std::chrono::duration<double>(Clock::now() - begin).count();
            const double u = duration > 0.0 ? std::min(elapsed / duration, 1.0) : 1.0;
            Pose position = land_deep;
            Pose velocity{};
            for (int i = 0; i < 2; ++i) {
                if (u < 1.0) {
                    const double delta = land_deep[i] - start[i];
                    const double remaining = 1.0 - u;
                    position[i] = start[i] + delta * (1.0 - remaining * remaining * remaining);
                    velocity[i] = (3.0 * delta / duration) * remaining * remaining;
                }
            }
            set(position, velocity, gains);
            tick();
            if (u >= 1.0) return;
        }
    }

    void return_to_stand() {
        move("stand_return", stand, slow_deg_s, normal, Profile::Quintic, 0.5, 0.5);
    }

    void hold(const Pose& target, Gains gains, double seconds, const char* name) {
        if (quit) return;
        phase = name;
        in_flight = false;
        set(target, Pose{}, gains);
        const auto begin = Clock::now();
        while (!quit && std::chrono::duration<double>(Clock::now() - begin).count() < seconds) tick();
    }

    void hold_forever(const Pose& target, Gains gains, const char* name) {
        if (quit) return;
        phase = name;
        in_flight = false;
        set(target, Pose{}, gains);
        while (!quit) tick();
    }
};

void usage(const char* program) {
    std::printf("用法: %s [V_PUSH=340] [Kp_LAND=25] [Kd_LAND=3] [TOR_FF_STAND=-3] [TOR_FF_CROUCH=-4.5] [TD_DEG=3] [V_ABS0=450] [TD_VEL=150] [ARM_DEG=4] [Kp_ABS=40] [Kd_ABS=3.5] [TOR_FRIC=1.5]\n"
                "      %s drop [Kp_LAND=25] [Kd_LAND=3] [TOR_FF_STAND=-3] [TOR_FF_CROUCH=-4.5] [TD_DEG=3] [V_ABS0=450] [TD_VEL=150] [ARM_DEG=4] [Kp_ABS=40] [Kd_ABS=3.5] [TOR_FRIC=1.5]\n"
                "速度单位deg/s，力矩单位N·m，角度单位度；V_ABS0替代原T_ABS参数；网卡固定enp4s0；Ctrl+C退出。\n", program, program);
}
}

int main(int argc, char** argv)
{
    const auto program_start = Clock::now();
    const auto wall_start = std::chrono::system_clock::now();
    const bool drop = argc > 1 && !std::strcmp(argv[1], "drop");
    if (argc > 13 || (argc > 1 && (!std::strcmp(argv[1], "--help") || !std::strcmp(argv[1], "-h")))) {
        usage(argv[0]);
        return argc > 13 ? 1 : 0;
    }
    double v_push = 340.0;
    double tor_ff_stand = -3.0;
    double tor_ff_crouch = -4.5;
    double td_deg = 3.0;
    double v_abs0 = 450.0;
    double td_vel = 150.0;
    double arm_deg = 4.0;
    double tor_fric = 1.5;
    Gains landing{25.0, 3.0};
    Gains absorption{40.0, 3.5};
    try {
        if (!drop && argc > 1) v_push = std::stod(argv[1]);
        if (argc > 2) landing.kp = std::stod(argv[2]);
        if (argc > 3) landing.kd = std::stod(argv[3]);
        if (argc > 4) tor_ff_stand = std::stod(argv[4]);
        if (argc > 5) tor_ff_crouch = std::stod(argv[5]);
        if (argc > 6) td_deg = std::stod(argv[6]);
        if (argc > 7) v_abs0 = std::stod(argv[7]);
        if (argc > 8) td_vel = std::stod(argv[8]);
        if (argc > 9) arm_deg = std::stod(argv[9]);
        if (argc > 10) absorption.kp = std::stod(argv[10]);
        if (argc > 11) absorption.kd = std::stod(argv[11]);
        if (argc > 12) tor_fric = std::stod(argv[12]);
        if (!std::isfinite(v_abs0) || v_abs0 <= 0)
            throw std::invalid_argument("V_ABS0必须为有限正数");
    } catch (const std::exception& error) {
        std::fprintf(stderr, "参数错误: %s\n", error.what());
        usage(argv[0]);
        return 1;
    }

    std::signal(SIGINT, request_exit);
    std::signal(SIGTERM, request_exit);
    CycleLog log(program_start, wall_start);
    setMotorCycleObserver(CycleLog::observe, &log);
    if (!startBus()) {
        stopBus();
        setMotorCycleObserver(nullptr, nullptr);
        log.save(v_push);
        return 1;
    }

    Leg leg(tor_ff_stand, tor_ff_crouch, tor_fric);
    leg.sample();
    if (drop) {
        leg.move("drop_land", land, slow_deg_s, normal);
        leg.land_arm(landing, arm_deg);
        leg.land_wait(landing, td_deg, td_vel, false);
        leg.absorb(v_abs0, absorption);
        leg.hold(land_deep, absorption, 0.15, "land_deep_hold");
        leg.return_to_stand();
        if (!quit) std::puts("drop: 缓冲及回站完成，持续保持stand，Ctrl+C退出。");
        leg.hold_forever(stand, normal, "stand_hold");
    } else {
        leg.move("stand", stand, slow_deg_s, normal, Profile::Quintic);
        leg.move("crouch", crouch, slow_deg_s, normal, Profile::Quintic);
        leg.hold(crouch, normal, 0.3, "crouch_hold");
        leg.push_off(v_push);
        // push插值后继续蹬伸保持，实测偏差小于3度或等待0.2秒后进入空中段。
        const auto push_deadline = Clock::now() + std::chrono::milliseconds(200);
        leg.phase = "push_wait";
        while (!quit && Clock::now() < push_deadline) {
            const auto feedback = motorFeedback(0, MOTOR_JOINT_CONFIGS[0].motor_id);
            if (std::abs(feedback.position_deg - push[0]) < 3.0) break;
            leg.tick(); // 沿用push终点、零速度及蹬伸增益。
        }
        leg.fly_to_land(landing);
        leg.land_arm(landing, arm_deg);
        leg.land_wait(landing, td_deg, td_vel, true);
        leg.absorb(v_abs0, absorption);
        leg.hold(land_deep, absorption, 0.15, "land_deep_hold");
        leg.return_to_stand();
        if (!quit) std::puts("跳跃完成，持续保持stand，Ctrl+C退出。");
        leg.hold_forever(stand, normal, "stand_hold");
    }

    stopBus(); // 先等待收发线程退出，再统一写盘；Ctrl+C也走此路径。
    setMotorCycleObserver(nullptr, nullptr);
    return log.save(v_push) ? 0 : 1;
}

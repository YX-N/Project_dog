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
#include <memory>
#include <mutex>
#include <stdexcept>
#include <vector>
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

    void save(const std::string& path) const {
        std::ofstream csv;
        csv.exceptions(std::ios::failbit | std::ios::badbit);
        csv.open(path);
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
    }
};

// 动作与逐周期报文解码沿用jump_mit；本程序只增加循环、质量缩放和分次统计。
struct TestLog {
    static constexpr double missing = std::numeric_limits<double>::quiet_NaN();
    struct Summary {
        int index;
        double start_time;
        std::array<double, 2> push_peak{missing, missing};
        std::array<double, 2> land_peak{missing, missing};
        double flight_time = missing;
    };
    struct Jump {
        Summary summary;
        CycleLog samples;
        bool started = false, completed = false;
        Jump(int index, double time, Clock::time_point start,
             std::chrono::system_clock::time_point wall_start)
            : summary{index, time}, samples(start, wall_start) {}
    };
    Clock::time_point start;
    std::chrono::system_clock::time_point wall_start;
    int requested_count;
    std::mutex mutex;
    std::shared_ptr<Jump> active, last_completed;
    std::vector<std::shared_ptr<Jump>> selected;
    std::vector<Summary> summaries;

    TestLog(Clock::time_point start, std::chrono::system_clock::time_point wall_start, int count)
        : start(start), wall_start(wall_start), requested_count(count) {}

    void begin(int index, Clock::time_point when) {
        auto next = std::make_shared<Jump>(index,
            std::chrono::duration<double>(when - start).count(), start, wall_start);
        std::lock_guard<std::mutex> lock(mutex);
        active = std::move(next);
    }

    static void observe(const EtherCAT_Msg& sent, const char* phase, void* context) {
        auto& log = *static_cast<TestLog*>(context);
        std::lock_guard<std::mutex> lock(log.mutex);
        if (!log.active) return;
        auto& jump = *log.active;
        if (!jump.started) {
            if (std::strcmp(phase, "crouch")) return;
            jump.started = true;
        }
        CycleLog::observe(sent, phase, &jump.samples);
        // 峰值取电流绝对值；蹬伸含push_wait，落地含布防、等待和absorb。
        const bool pushing_phase = !std::strcmp(phase, "push") || !std::strcmp(phase, "push_wait");
        const bool landing_phase = !std::strcmp(phase, "land_arm") ||
                                   !std::strcmp(phase, "land_wait") || !std::strcmp(phase, "absorb");
        if (jump.completed || (!pushing_phase && !landing_phase)) return;
        auto& peaks = pushing_phase ? jump.summary.push_peak : jump.summary.land_peak;
        const auto& row = jump.samples.rows.back();
        for (int i = 0; i < 2; ++i) {
            const double current = std::abs(row.motors[i][6]);
            if (std::isfinite(current) && (!std::isfinite(peaks[i]) || current > peaks[i])) peaks[i] = current;
        }
    }

    Summary complete(double flight_time) {
        std::lock_guard<std::mutex> lock(mutex);
        active->summary.flight_time = flight_time;
        active->completed = true; // absorb已完成，即使在周期等待中退出也保存这次汇总。
        summaries.push_back(active->summary);
        return active->summary;
    }

    void finish_period() {
        std::lock_guard<std::mutex> lock(mutex);
        if (!active) return;
        if (active->completed) {
            const int index = active->summary.index;
            if (index == 1 || index % 50 == 0 || index == requested_count) selected.push_back(active);
            last_completed = active; // 保留最近一次，Ctrl+C提前退出也有“最后一次”完整记录。
        }
        active.reset();
    }

    bool save(double mass, double period) const {
        // 必须在stopBus之后调用，此时不再有收发线程追加数据。
        try {
            std::filesystem::create_directories("logs");
            const auto stamp = std::chrono::system_clock::to_time_t(wall_start);
            std::tm local{};
            localtime_r(&stamp, &local);
            const auto micros = std::chrono::duration_cast<std::chrono::microseconds>(
                wall_start.time_since_epoch()).count() % 1000000;
            std::ostringstream name;
            name.imbue(std::locale::classic());
            name << "logs/test_" << std::put_time(&local, "%Y%m%d_%H%M%S") << '_'
                 << std::setfill('0') << std::setw(6) << micros << '_'
                 << std::setprecision(15) << mass << '_' << period;
            const std::string base = name.str();
            std::ofstream csv;
            csv.exceptions(std::ios::failbit | std::ios::badbit);
            csv.open(base + ".csv");
            csv.imbue(std::locale::classic());
            csv << "index,start_time_s,ID1_push_peak_A,ID2_push_peak_A,ID1_land_peak_A,ID2_land_peak_A,flight_time_s\n"
                << std::fixed << std::setprecision(9);
            for (const auto& row : summaries) {
                csv << row.index << ',' << row.start_time;
                for (double value : row.push_peak) csv << ',' << value;
                for (double value : row.land_peak) csv << ',' << value;
                csv << ',' << row.flight_time << '\n';
            }
            csv.close();
            auto details = selected;
            auto append = [&](const std::shared_ptr<Jump>& jump) {
                if (!jump || jump->samples.rows.empty()) return;
                for (const auto& saved : details)
                    if (saved->summary.index == jump->summary.index) return;
                details.push_back(jump);
            };
            append(last_completed);
            append(active); // 中途退出时另外保存当前未完成跳跃的已有数据。
            for (const auto& jump : details) {
                std::ostringstream path;
                path << base << "_jump" << std::setfill('0') << std::setw(6) << jump->summary.index;
                if (!jump->completed) path << "_partial";
                jump->samples.save(path.str() + ".csv");
            }
            std::printf("已保存汇总：%s.csv（%zu次完成），完整/部分逐周期记录：%zu份\n",
                        base.c_str(), summaries.size(), details.size());
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
    std::printf("用法: %s <V_PUSH deg/s> <周期 s> <次数> <总质量 kg> [V_ABS0=450] [Kp_ABS=40] [Kd_ABS=3.5] [TOR_FRIC=1.5]\n"
                "周期包含下蹲、跳跃、缓冲和回站，回站后等待周期结束；超期不截断动作。\n"
                "速度单位deg/s，TOR_FRIC单位N·m；完成后保持stand，Ctrl+C退出并保存CSV。\n", program);
}
}

int main(int argc, char** argv)
{
    const auto program_start = Clock::now();
    const auto wall_start = std::chrono::system_clock::now();
    if (argc < 5 || argc > 9 || !std::strcmp(argv[1], "--help") || !std::strcmp(argv[1], "-h")) {
        usage(argv[0]);
        return argc == 2 && (!std::strcmp(argv[1], "--help") || !std::strcmp(argv[1], "-h")) ? 0 : 1;
    }
    double v_push, period, mass;
    double v_abs0 = 450.0, tor_fric = 1.5;
    Gains absorption{40.0, 3.5};
    int count;
    try {
        auto number = [](const char* text) {
            std::size_t used = 0;
            const double value = std::stod(text, &used);
            if (text[used] || !std::isfinite(value) || value <= 0)
                throw std::invalid_argument("速度、周期和总质量必须为有限正数");
            return value;
        };
        v_push = number(argv[1]);
        period = number(argv[2]);
        mass = number(argv[4]);
        if (argc > 5) v_abs0 = number(argv[5]);
        if (argc > 6) absorption.kp = std::stod(argv[6]);
        if (argc > 7) absorption.kd = std::stod(argv[7]);
        if (argc > 8) tor_fric = std::stod(argv[8]);
        std::size_t used = 0;
        count = std::stoi(argv[3], &used);
        if (argv[3][used] || count <= 0) throw std::invalid_argument("次数必须为正整数");
    } catch (const std::exception& error) {
        std::fprintf(stderr, "参数错误: %s\n", error.what());
        usage(argv[0]);
        return 1;
    }

    const double mass_scale = mass / 6.2;
    const Gains landing{25.0 * mass_scale, std::min(5.0, 3.0 * mass_scale)};
    std::signal(SIGINT, request_exit);
    std::signal(SIGTERM, request_exit);
    TestLog log(program_start, wall_start, count);
    setMotorCycleObserver(TestLog::observe, &log);
    if (!startBus()) {
        stopBus();
        setMotorCycleObserver(nullptr, nullptr);
        log.save(mass, period);
        return 1;
    }

    Leg leg(-3.0 * mass_scale, -4.5 * mass_scale, tor_fric);
    leg.sample();
    leg.move("stand", stand, slow_deg_s, normal, Profile::Quintic);
    for (int n = 0; n < count && !quit; ++n) {
        const int index = n + 1;
        const auto cycle_start = Clock::now();
        const auto cycle_end = cycle_start + std::chrono::duration<double>(period);
        log.begin(index, cycle_start);
        // 上周期在stand等待；本周期先恢复蹬伸起点crouch，准备时间计入周期。
        leg.move("crouch", crouch, slow_deg_s, normal, Profile::Quintic);
        leg.push_off(v_push);
        // 沿用jump_mit：push终点继续保持到误差<3度，最多等待0.2秒。
        const auto push_deadline = Clock::now() + std::chrono::milliseconds(200);
        leg.phase = "push_wait";
        while (!quit && Clock::now() < push_deadline) {
            const auto feedback = motorFeedback(0, MOTOR_JOINT_CONFIGS[0].motor_id);
            if (std::abs(feedback.position_deg - push[0]) < 3.0) break;
            leg.tick();
        }
        if (quit) break;
        // 蹬伸保持结束至触地判定的时间；包括land_arm及land_wait。
        const auto flight_start = Clock::now();
        leg.fly_to_land(landing);
        leg.land_arm(landing, 4.0);
        leg.land_wait(landing, 3.0, 150.0, true);
        const auto touchdown = Clock::now();
        if (quit) break;
        leg.absorb(v_abs0, absorption);
        if (quit) break;
        const auto summary = log.complete(std::chrono::duration<double>(touchdown - flight_start).count());
        if (index % 10 == 0) {
            std::printf("%d/%d：蹬伸峰值电流 ID1 %.3f A，ID2 %.3f A；腾空时间 %.6f s\n",
                        index, count, summary.push_peak[0], summary.push_peak[1], summary.flight_time);
            std::fflush(stdout);
        }
        leg.hold(land_deep, absorption, 0.15, "land_deep_hold");
        leg.return_to_stand();
        leg.phase = "stand_hold";
        leg.in_flight = false;
        leg.set(stand, Pose{}, normal);
        while (!quit && Clock::now() < cycle_end) leg.tick();
        log.finish_period();
        // 已超期则下一次从当前时刻重新计时，不截断布防/缓冲，也不追赶旧周期。
    }
    if (!quit) {
        std::puts("连续跳跃完成，持续保持stand，Ctrl+C退出并保存CSV。");
        leg.hold_forever(stand, normal, "stand_hold");
    }

    stopBus();
    setMotorCycleObserver(nullptr, nullptr);
    return log.save(mass, period) ? 0 : 1;
}

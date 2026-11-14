#include "motor_bus.h"
#include "queue.h"
#include <map>
#include <mutex>
#include <utility>

std::atomic<bool> motor_bus_ready{false};

namespace {
constexpr double rad_to_deg = 57.29577951308232;
std::mutex feedback_mutex;
std::map<std::pair<int, std::uint16_t>, MotorFeedback> feedback;
}

void recordMotorFeedback(int slave, const EtherCAT_Msg& raw, const OD_Motor_Msg decoded[6])
{
    std::lock_guard<std::mutex> lock(feedback_mutex);
    const auto now = std::chrono::steady_clock::now();
    // 接收槽仅用于关联原始帧与解码结果；对外始终按从站和电机ID查询。
    for (int i = 0; i < 6; ++i) {
        const auto& frame = raw.motor[i];
        const auto& value = decoded[i];
        if (!frame.dlc || frame.id == 0 || frame.id >= 0x7ff) continue;
        auto& out = feedback[{slave, static_cast<std::uint16_t>(frame.id)}];
        out.status = frame.data[0];
        const int ack = frame.data[0] >> 5;
        if (ack == 1 && frame.dlc >= 7) {
            out.position_deg = value.angle_actual_rad * rad_to_deg;
            out.velocity_deg_s = value.speed_actual_rad * rad_to_deg;
            out.current_A = value.current_actual_float;
            out.observed.fill(now);
            out.temperature = (int(frame.data[6]) - 50) / 2.0;
            out.error = value.error;
        } else if (ack == 2 && frame.dlc == 8) {
            out.position_deg = value.angle_actual_float;
            out.current_A = value.current_actual_float;
            out.observed[0] = out.observed[2] = now;
            out.temperature = (int(frame.data[7]) - 50) / 2.0;
            out.error = value.error;
        } else if (ack == 3 && frame.dlc == 8) {
            out.velocity_deg_s = value.speed_actual_float * 6.0;
            out.current_A = value.current_actual_float;
            out.observed[1] = out.observed[2] = now;
            out.temperature = (int(frame.data[7]) - 50) / 2.0;
            out.error = value.error;
        } else if (ack == 5 && frame.dlc == 6) {
            const int field = frame.data[1] - 1;
            if (field < 0 || field > 2) continue;
            if (field == 0) out.position_deg = value.angle_actual_float;
            if (field == 1) out.velocity_deg_s = value.speed_actual_float * 6.0;
            if (field == 2) out.current_A = value.current_actual_float;
            out.observed[field] = now;
            out.error = value.error;
        }
    }
}

MotorFeedback motorFeedback(int slave, std::uint16_t motor_id)
{
    std::lock_guard<std::mutex> lock(feedback_mutex);
    return feedback[{slave, motor_id}];
}

void clearMotorFeedback()
{
    std::lock_guard<std::mutex> lock(feedback_mutex);
    feedback.clear();
}

bool queryMotorPosition(int slave, int passage, std::uint16_t motor_id, double& position_deg)
{
    EtherCAT_Msg request{};
    Motor_Msg reply{};
    get_motor_parameter(&request, passage, motor_id, param_get_pos);
    if (!queryMotor(slave, passage, request.motor[passage - 1], reply)) return false;
    position_deg = motorParameterValue(reply);
    return true;
}

/*
 * 功能：管理电机反馈数据，提供反馈查询、缓存清空和主动位置查询接口。
 * 流程：接收解码后的反馈 → 按从站和电机ID缓存 → 供应用程序读取。
 *       主动查询时，构造CAN查询报文并通过底层通信接口获取电机角度。
 */

#include "motor_bus.h"
#include "queue.h"
#include <map>
#include <mutex>
#include <utility>

std::atomic<bool> motor_bus_ready{false};  // 通信就绪标志

namespace {
constexpr double rad_to_deg = 57.29577951308232;  // 弧度转角度
std::mutex feedback_mutex;  // 保护反馈缓存，避免多线程同时访问
std::map<std::pair<int, std::uint16_t>, MotorFeedback> feedback;  // 按(从站索引, 电机ID)存储反馈
}

// 将接收到的电机反馈保存到缓存
void recordMotorFeedback(int slave, const EtherCAT_Msg& raw, const OD_Motor_Msg decoded[6])
{
    std::lock_guard<std::mutex> lock(feedback_mutex);  // 自动加锁，退出作用域时解锁
    const auto now = std::chrono::steady_clock::now();

    // 遍历PDO中的6个CAN报文槽位
    for (int i = 0; i < 6; ++i) {
        const auto& frame = raw.motor[i];  // 原始CAN报文
        const auto& value = decoded[i];    // 解码后的电机数据

        // 跳过无效报文
        if (!frame.dlc || frame.id == 0 || frame.id >= 0x7ff) continue;

        // 获取对应电机的反馈缓存，不存在则自动创建
        auto& out = feedback[{slave, static_cast<std::uint16_t>(frame.id)}];

        out.status = frame.data[0];
        const int ack = frame.data[0] >> 5;  // 提取高3位，判断反馈类型

        if (ack == 1 && frame.dlc >= 7) {
            // 类型1：位置、速度、电流、温度、错误码
            out.position_deg = value.angle_actual_rad * rad_to_deg;
            out.velocity_deg_s = value.speed_actual_rad * rad_to_deg;
            out.current_A = value.current_actual_float;
            out.observed.fill(now);  // 更新位置、速度、电流的时间戳
            out.temperature = (int(frame.data[6]) - 50) / 2.0;
            out.error = value.error;

        } else if (ack == 2 && frame.dlc == 8) {
            // 类型2：位置、电流、温度、错误码
            out.position_deg = value.angle_actual_float;
            out.current_A = value.current_actual_float;
            out.observed[0] = out.observed[2] = now;
            out.temperature = (int(frame.data[7]) - 50) / 2.0;
            out.error = value.error;

        } else if (ack == 3 && frame.dlc == 8) {
            // 类型3：速度、电流、温度、错误码
            out.velocity_deg_s = value.speed_actual_float * 6.0;
            out.current_A = value.current_actual_float;
            out.observed[1] = out.observed[2] = now;
            out.temperature = (int(frame.data[7]) - 50) / 2.0;
            out.error = value.error;

        } else if (ack == 5 && frame.dlc == 6) {
            // 类型5：指定参数查询结果
            const int field = frame.data[1] - 1;
            if (field < 0 || field > 2) continue;

            if (field == 0) out.position_deg = value.angle_actual_float;
            if (field == 1) out.velocity_deg_s = value.speed_actual_float * 6.0;
            if (field == 2) out.current_A = value.current_actual_float;

            out.observed[field] = now;  // 更新对应参数的时间戳
            out.error = value.error;
        }
    }
}

// 根据从站索引和电机ID读取缓存中的反馈
MotorFeedback motorFeedback(int slave, std::uint16_t motor_id)
{
    std::lock_guard<std::mutex> lock(feedback_mutex);
    return feedback[{slave, motor_id}];
}

// 清空全部电机反馈缓存
void clearMotorFeedback()
{
    std::lock_guard<std::mutex> lock(feedback_mutex);
    feedback.clear();
}

// 主动查询指定电机的实际位置
bool queryMotorPosition(int slave, int passage, std::uint16_t motor_id, double& position_deg)
{
    EtherCAT_Msg request{};  // 查询请求PDO
    Motor_Msg reply{};       // 接收电机CAN应答

    // 构造位置查询报文
    get_motor_parameter(&request, passage, motor_id, param_get_pos);

    // 通过指定从站和CAN通道发送查询，失败返回false
    if (!queryMotor(slave, passage, request.motor[passage - 1], reply)) return false;

    // 提取反馈中的位置值，写入引用参数
    position_deg = motorParameterValue(reply);
    return true;
}
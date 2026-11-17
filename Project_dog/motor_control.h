/*
 * @Description: 电机控制协议接口声明，定义电机命令、反馈数据结构及报文构建/解析函数。
 * @Author: kx zhang
 * @Date: 2022-09-20 09:45:09
 * @LastEditTime: 2022-11-13 17:16:19
 */
#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <inttypes.h>  // 提供 uint8_t、uint16_t 等固定宽度整数类型。
#include "pdo_types.h" // EtherCAT_Msg、Motor_Msg 等 EtherCAT/CAN 报文结构体。
#include "math_ops.h"  // 提供浮点数与协议整数值之间的转换函数。

// 电机参数查询命令：传给 get_motor_parameter()，指定要读取的参数。
#define param_get_pos   0x01 // 查询电机实际位置。
#define param_get_spd   0x02 // 查询电机实际速度。
#define param_get_cur   0x03 // 查询电机实际电流。
#define param_get_pwr   0x04 // 查询电机功率。
#define param_get_acc   0x05 // 查询电机加速度。
#define param_get_lkgKP 0x06 // 查询 linkage_KP 参数。
#define param_get_spdKI 0x07 // 查询 speed_KI 参数。
#define param_get_fdbKP 0x08 // 查询 feedback_KP 参数。
#define param_get_fdbKD 0x09 // 查询 feedback_KD 参数。

// 电机通信模式标识：用于指定电机采用的通信方式。
#define comm_ack  0x00 // 问答/应答模式（发送命令后由电机返回应答报文）。
#define comm_auto 0x01 // 自动反馈模式（电机按该模式主动反馈状态）。

// 电机配置类报文的反馈信息。
typedef struct
{
    uint16_t motor_id; // 电机 ID，或配置操作涉及的电机 ID。
    uint8_t INS_code;  // 指令/参数编号，用于标识当前反馈对应的操作或查询项。
    uint8_t motor_fbd; // 电机通信反馈结果码，例如模式设置、零点设置、ID 操作或查询结果。
} MotorCommFbd;

// 解包后的电机状态与参数数据；每个电机对应一个 OD_Motor_Msg。
typedef struct
{
    uint16_t angle_actual_int;   // 实际位置的协议整数值，保留驱动器反馈的原始表示。
    uint16_t angle_desired_int;  // 目标位置的协议整数值。
    int16_t speed_actual_int;    // 实际速度的协议整数值。
    int16_t speed_desired_int;   // 目标速度的协议整数值。
    int16_t current_actual_int;  // 实际电流的协议整数值。
    int16_t current_desired_int; // 目标电流的协议整数值。

    float speed_actual_rad;      // 实际速度，单位 rad/s。
    float speed_desired_rad;     // 目标速度，单位 rad/s。
    float angle_actual_rad;      // 实际位置，单位 rad。
    float angle_desired_rad;     // 目标位置，单位 rad。

    uint16_t motor_id;           // 电机 ID。
    uint8_t temperature;         // 电机温度反馈。
    uint8_t error;               // 电机反馈报文中的错误标志/错误码。

    float angle_actual_float;    // 实际位置，浮点数格式。
    float speed_actual_float;    // 实际速度，浮点数格式。
    float current_actual_float;  // 实际电流，浮点数格式。
    float angle_desired_float;   // 目标位置，浮点数格式。
    float speed_desired_float;   // 目标速度，浮点数格式。
    float current_desired_float; // 目标电流，浮点数格式。

    float power;                 // 电机功率参数。
    uint16_t acceleration;       // 电机加速度参数。
    uint16_t linkage_KP;         // linkage_KP 参数。   连杆相关控制比例增益	调节连杆控制误差的响应强度
    uint16_t speed_KI;           // speed_KI 参数。     速度环积分增益	       累积速度误差，减小稳态速度误差
    uint16_t feedback_KP;        // 反馈环比例参数 KP。   反馈控制比例增益	     根据反馈误差产生比例调节
    uint16_t feedback_KD;        // 反馈环微分参数 KD。   反馈控制微分增益	     根据误差变化趋势进行阻尼调节
} OD_Motor_Msg;

// 保存最近一次解析到的电机 ID，供电机反馈/查询处理逻辑使用。
extern uint16_t motor_id_check;

// 以下函数均在 EtherCAT_Msg 中构建 CAN 报文，实际发送由上层 EtherCAT 通信流程完成。
void MotorIDReset(EtherCAT_Msg* tx_msg); // 通过 CAN1 发送电机 ID 重置命令，重置后的 ID 为 1。

/**
 * @brief 设置电机 ID。
 * @param tx_msg       待写入命令报文的 EtherCAT 消息指针。
 * @param motor_id     电机当前 ID。
 * @param motor_id_new 要设置的新电机 ID。
 *
 * 注意：该配置命令使用 CAN1 通道。
 */
void MotorIDSetting(EtherCAT_Msg* tx_msg, uint16_t motor_id, uint16_t motor_id_new);

/**
 * @brief 设置电机通信模式或执行零点设置。
 * @param tx_msg   待写入命令报文的 EtherCAT 消息指针。
 * @param motor_id 目标电机 ID。
 * @param cmd      命令码：0x00 无操作，0x01 自动模式，0x02 问答模式，0x03 设置电机零点。
 *
 * 注意：该配置命令使用 CAN1 通道。
 */
void MotorSetting(EtherCAT_Msg* tx_msg, uint16_t motor_id, uint8_t cmd);

// 通过 CAN1 构建读取电机当前通信模式的命令报文。
void MotorCommModeReading(EtherCAT_Msg* tx_msg, uint16_t motor_id);

// 通过 CAN1 构建读取电机 ID 的命令报文。
void MotorIDReading(EtherCAT_Msg* tx_msg);

/**
 * @brief 构建电机零点设置命令。
 * @param tx_msg   待写入命令报文的 EtherCAT 消息指针。
 * @param passage  电机通道编号：CAN1 对应 1～3，CAN2 对应 4～6。
 * @param motor_id 目标电机 ID。
 */
void Motor_Setzero(EtherCAT_Msg* tx_msg, uint16_t passage, uint16_t motor_id);

/**
 * @brief 构建 MIT 力位混合控制命令。
 * @param tx_msg   待写入命令报文的 EtherCAT 消息指针。
 * @param passage  电机通道编号：CAN1 对应 1～3，CAN2 对应 4～6。
 * @param motor_id 目标电机 ID。
 * @param kp       位置比例增益，范围 0～500。
 * @param kd       速度微分增益，范围 0～5。
 * @param pos      期望位置，单位 rad，范围 -12.5～12.5。
 * @param spd      期望速度，单位 rad/s，范围 -18～18。
 * @param tor      前馈力矩，单位 Nm，范围 -30～30。
 *
 * 函数会对输入参数限幅，并将其编码为 CAN 控制报文。
 */
void send_motor_ctrl_cmd(EtherCAT_Msg* tx_msg, uint8_t passage, uint16_t motor_id, float kp, float kd, float pos,
                         float spd, float tor);

/**
 * @brief 构建伺服位置控制命令（问答模式）。
 * @param tx_msg    待写入命令报文的 EtherCAT 消息指针。
 * @param passage   电机通道编号：CAN1 对应 1～3，CAN2 对应 4～6。
 * @param motor_id  目标电机 ID。
 * @param pos       期望位置，单位为度。
 * @param spd       期望速度，范围 0～18000，对应 0～1800.0 rpm，比例为 10:1。
 * @param cur       电流阈值，范围 0～3000，对应 0～300.0 A，比例为 10:1。
 * @param ack_status 应答类型：0 不返回，1～3 分别请求反馈报文类型 1～3。
 */
void set_motor_position(EtherCAT_Msg* tx_msg, uint8_t passage, uint16_t motor_id, float pos, uint16_t spd, uint16_t cur,
                        uint8_t ack_status);

/**
 * @brief 构建伺服速度控制命令（问答模式）。
 * @param tx_msg     待写入命令报文的 EtherCAT 消息指针。
 * @param passage    电机通道编号：CAN1 对应 1～3，CAN2 对应 4～6。
 * @param motor_id   目标电机 ID。
 * @param spd        期望速度，单位 rpm，范围 -18000～18000。
 * @param cur        电流阈值，范围 0～3000，对应 0～300.0 A，比例为 10:1。
 * @param ack_status 应答类型：0 不返回，1～3 分别请求反馈报文类型 1～3。
 */
void set_motor_speed(EtherCAT_Msg* tx_msg, uint8_t passage, uint16_t motor_id, float spd, uint16_t cur,
                     uint8_t ack_status);

/**
 * @brief 构建电流、力矩或制动控制命令（问答模式）。
 * @param tx_msg      待写入命令报文的 EtherCAT 消息指针。
 * @param passage     电机通道编号：CAN1 对应 1～3，CAN2 对应 4～6。
 * @param motor_id    目标电机 ID。
 * @param cur_tor     期望电流/力矩，范围 -3000～3000，对应 -30.00～30.00 A/Nm，比例为 100:1。
 * @param ctrl_status 控制类型：0 电流控制，1 力矩控制，2 变量阻尼制动，3 能耗制动，4 再生制动。
 * @param ack_status  应答类型：0 不返回，1～3 分别请求反馈报文类型 1～3。
 */
void set_motor_cur_tor(EtherCAT_Msg* tx_msg, uint8_t passage, uint16_t motor_id, int16_t cur_tor, uint8_t ctrl_status,
                       uint8_t ack_status);

/**
 * @brief 设置电机加速度参数（问答模式）。
 * @param tx_msg     待写入命令报文的 EtherCAT 消息指针。
 * @param passage    电机通道编号：CAN1 对应 1～3，CAN2 对应 4～6。
 * @param motor_id   目标电机 ID。
 * @param acc        加速度参数，范围 0～2000，对应 0～20.00 rad/s，比例为 100:1。
 * @param ack_status 应答类型：0 不返回，1～2 请求相应反馈。
 */
void set_motor_acceleration(EtherCAT_Msg* tx_msg, uint8_t passage, uint16_t motor_id, uint16_t acc, uint8_t ack_status);

/**
 * @brief 设置 linkage_KP 和 speed_KI 两项参数（问答模式）。
 * @param tx_msg     待写入命令报文的 EtherCAT 消息指针。
 * @param passage    电机通道编号：CAN1 对应 1～3，CAN2 对应 4～6。
 * @param motor_id   目标电机 ID。
 * @param linkage    linkage_KP 参数值，范围 0～10000。
 * @param speedKI    speed_KI 参数值，范围 0～10000。
 * @param ack_status 应答类型，源码中大于 2 时不构建有效命令。
 */
void set_motor_linkage_speedKI(EtherCAT_Msg* tx_msg, uint8_t passage, uint16_t motor_id, uint16_t linkage,
                               uint16_t speedKI, uint8_t ack_status);

/**
 * @brief 设置反馈环 KP、KD 参数（问答模式）。
 * @param tx_msg     待写入命令报文的 EtherCAT 消息指针。
 * @param passage    电机通道编号：CAN1 对应 1～3，CAN2 对应 4～6。
 * @param motor_id   目标电机 ID。
 * @param fdbKP      反馈环比例参数 KP，范围 0～10000。
 * @param fdbKD      反馈环微分参数 KD，范围 0～10000。
 * @param ack_status 应答类型，源码中大于 2 时不构建有效命令。
 */
void set_motor_feedbackKP_KD(EtherCAT_Msg* tx_msg, uint8_t passage, uint16_t motor_id, uint16_t fdbKP,
                             uint16_t fdbKD, uint8_t ack_status);

/**
 * @brief 构建电机参数查询命令（问答模式）。
 * @param tx_msg    待写入查询报文的 EtherCAT 消息指针。
 * @param passage   电机通道编号：CAN1 对应 1～3，CAN2 对应 4～6。
 * @param motor_id  目标电机 ID。
 * @param param_cmd 查询参数编号，使用 param_get_pos～param_get_fdbKD 中定义的宏。
 */
void get_motor_parameter(EtherCAT_Msg* tx_msg, uint8_t passage, uint16_t motor_id, uint8_t param_cmd);



/**
 * @brief 解包单个 EtherCAT 从站收到的 CAN 电机报文，并更新电机反馈结构体。
 * @param rx_msg    当前从站收到的 EtherCAT 消息，内部最多包含 6 路 CAN 报文。
 * @param comm_mode 电机通信模式：comm_ack（0x00）为问答模式，comm_auto（0x01）为自动反馈模式。
 * @param motor_msg 长度为 6 的电机反馈数组，每个元素保存一路电机的解析结果。
 * @param slave     当前从站编号，用于区分/打印从站报文。
 * @param print     是否打印解析过程和反馈数据，非 0 表示打印。
 */
void RV_can_data_repack(EtherCAT_Msg* rx_msg, uint8_t comm_mode, OD_Motor_Msg motor_msg[6], uint8_t slave, int print);



/**
 * @brief 批量解包多个 EtherCAT 从站的 CAN 电机反馈报文。
 * @param rx_msg    从站接收报文数组，每个从站对应一个 EtherCAT_Msg。
 * @param comm_mode 电机通信模式：comm_ack（0x00）为问答模式，comm_auto（0x01）为自动反馈模式。
 * @param motor_msg 二维电机反馈数组，形状为 [slave_num][6]，每个从站对应 6 路电机反馈。
 * @param slave_num 需要处理的从站数量。
 * @param print     是否打印解析过程和反馈数据，非 0 表示打印。
 */
void RV_can_data_repack_all(EtherCAT_Msg *rx_msg, uint8_t comm_mode, OD_Motor_Msg (*motor_msg)[6], uint8_t slave_num,
                            int print);

#endif
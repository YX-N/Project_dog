/*
 * @Description:
 * @Author: kx zhang
 * @Date: 2022-09-20 09:21:54
 * @LastEditTime: 2022-11-13 17:00:10
 */
#pragma once

#include <inttypes.h>
#include <string.h>

//********************************************//
//***********EtherCAT Message*****************//
//********************************************//

// 设置结构体按 1 字节对齐，取消编译器默认的内存填充，确保报文结构与通信协议一致。
#pragma pack(push, 1)

// 单路 CAN 电机报文结构。
typedef struct 
{
    uint32_t id;       // CAN 报文 ID。
    uint8_t rtr;       // 远程帧标志：0 表示数据帧，1 表示远程帧。
    uint8_t dlc;       // 数据长度，表示 data 中有效数据的字节数（0～8）。
    uint8_t data[8];   // CAN 报文数据，最多 8 字节。
} Motor_Msg;

// EtherCAT 从站与主站之间传输的电机 CAN 报文集合。
typedef struct
{
    uint8_t motor_num; // 本帧包含的电机报文数量。
    uint8_t can_ide;   // CAN 标识符类型：0 标准帧，1 扩展帧。
    Motor_Msg motor[6]; // 6 路 CAN 电机报文。数组有 6 个元素，下标为 0～5

} EtherCAT_Msg;

// 恢复编译器之前的结构体对齐设置。
#pragma pack(pop)

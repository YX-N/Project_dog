/*
 * @Description:
 * @Author: kx zhang
 * @Date: 2022-09-20 11:17:58
 * @LastEditTime: 2022-11-13 17:16:18
 */
#ifndef TRANSMIT_H
#define TRANSMIT_H

#define SLAVE_NUMBER 1 //可该最大从机数

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "pdo_types.h"
#include "ethercat.h"
#include "sys/time.h"

#ifdef __cplusplus
extern "C" {
#endif



void EtherCAT_Transmit(EtherCAT_Msg *MasterCommand); // 将主站控制数据发送至 EtherCAT 从站
void EtherCAT_Init(char *ifname);                    // 初始化 EtherCAT 主站，指定通信网卡
void EtherCAT_Init_Failure(const char *step);        // 处理 EtherCAT 初始化失败，并标识失败阶段
void EtherCAT_Run();                                 // 执行 EtherCAT 通信运行流程
void EtherCAT_Command_Set();                         // 准备本次通信需要发送的主站控制数据
void EtherCAT_Data_Get();                            // 获取并处理本次通信接收到的从站反馈数据
void startRun();                                     // 启动 EtherCAT 通信运行任务
int startBus();                                      // 启动 EtherCAT 总线，返回启动结果
void stopBus();                                      // 停止 EtherCAT 总线通信

#ifdef __cplusplus
};
#endif

#endif // PROJECT_RT_ETHERCAT_H
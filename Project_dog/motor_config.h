#ifndef PROJECT_DOG_MOTOR_CONFIG_H
#define PROJECT_DOG_MOTOR_CONFIG_H

#include <stdint.h>

/* 数组顺序固定为电机 1、2、3，默认使用 CAN1 的通道 1、2、3。 */
enum { MOTOR_JOINT_COUNT = 3 };

typedef struct
{
    uint16_t motor_id;
    uint8_t passage;
} MotorJointConfig;

static const MotorJointConfig MOTOR_JOINT_CONFIGS[MOTOR_JOINT_COUNT] = {
    {1, 1},
    {2, 2},
    {3, 3}
};

#endif /* PROJECT_DOG_MOTOR_CONFIG_H */

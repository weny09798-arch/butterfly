#ifndef __AS5600_PWM_H__
#define __AS5600_PWM_H__
#include "main.h"
#include "adc.h"

#define MAX_VALUE 4096  // 输入范围 0~4095，所以最大值为 4096  ggggggggghgggghghgghghgh88

// 各电机1024点位定义
#define MOTOR1_MIDPOINT 3410  // 电机1 1024点编码器值
#define MOTOR3_MIDPOINT 330  // 电机3 1024点编码器值 原来420

#define MOTOR2_MIDPOINT 3500 // 电机2 1024点编码器值  
#define MOTOR4_MIDPOINT 300  // 电机4 1024点编码器值


#define PROCESS_VALUE(raw, zero) \
    (((raw) + 4096u + (((zero) + 3072u) & 0x0FFFu)) & 0x0FFFu)



extern void StarAndGetResult(void);

#endif


#ifndef __MOTOR_H
#define __MOTOR_H

#include "pid.h"
#include "main.h"
#include "AS5600_PWM.h"
#include "stm32g0xx_hal.h"

typedef struct {
    struct {
        int16_t Magnet_Flag;        // 磁编码器状态标志（0:磁铁丢失, 1:磁铁正常, 2:磁铁过强等）
        int16_t Raw_Angle;          // 原始角度值（AS5600原始编码器值，范围0-4095）
        uint16_t Corrective_Angle;   // 校正后角度（经过零位校准和滤波处理后的角度）
        int16_t Target_Angle;       // 目标角度（PID控制的目标位置，单位与Corrective_Angle一致）
        int16_t Out_Chassis;        // 底盘输出值（发送给电机的控制量，通常是PWM或电流值）
        int16_t Speed;              // 电机转速（单位：RPM 或 度/秒，根据具体配置决定）
        int16_t Target_Speed;       // 电机转速（单位：RPM 或 度/秒，根据具体配置决定）
        int16_t Position;           // 电机绝对位置（累积角度值，用于记录多圈运动）
    } Wings_motor[5];   // 翅翼电机数组：[0]左前翅, [1]右前翅, [2]左后翅, [3]右后翅, [4]预留或特殊用途
    
} WINGS_DATA;

extern WINGS_DATA Wings_Data;
extern pid_type_def motor_1_pid, motor_3_pid, motor_2_pid, motor_4_pid;

#define KP 	10.0f
#define KD  00.0f

#define MOTOR_1_SPEED_PID_KP 18.0f
#define MOTOR_1_SPEED_PID_KI 0.0f
#define MOTOR_1_SPEED_PID_KD 00.0f

#define MOTOR_3_SPEED_PID_KP 20.0f
#define MOTOR_3_SPEED_PID_KI 0.0f
#define MOTOR_3_SPEED_PID_KD 00.0f

#define MOTOR_2_SPEED_PID_KP 12.0f
#define MOTOR_2_SPEED_PID_KI 0.0f
#define MOTOR_2_SPEED_PID_KD 00.0f

#define MOTOR_4_SPEED_PID_KP 20.0f
#define MOTOR_4_SPEED_PID_KI 0.0f
#define MOTOR_4_SPEED_PID_KD 00.0f

// 电机布局  原版前四者kp均为20     定义kp为15
// 1	2
// 
// 4	3

/*-------------Motor_PWM_M1--------------*/
#define PWM_M1_1 	  TIM2->CCR1	 //PWM_M1
#define PWM_M1_2 	  TIM2->CCR2	 //PWM_M1
/*------------------------------------*/

/*-------------Motor_PWM_M2--------------*/
#define PWM_M2_1 	  TIM2->CCR3	 //PWM_M2
#define PWM_M2_2 	  TIM2->CCR4	 //PWM_M2
/*------------------------------------*/

/*-------------Motor_PWM_M3--------------*/
#define PWM_M3_1 	  TIM3->CCR1	 //PWM_M3
#define PWM_M3_2 	  TIM3->CCR2	 //PWM_M3
/*------------------------------------*/

/*-------------Motor_PWM_M4--------------*/
#define PWM_M4_1 	  TIM3->CCR3	 //PWM_M4
#define PWM_M4_2 	  TIM3->CCR4	 //PWM_M4
/*------------------------------------*/

extern void Motor_PID_Control(void);
extern void Chassis_PID_Init(void);
extern void Set_Pwm(int16_t motor1_out, int16_t motor2_out, int16_t motor3_out, int16_t motor4_out);
extern uint16_t myabs(int16_t a);
extern void Motor_ECD_Control(void);
#endif
/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32g0xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "as5600.h"
#include "struct_typedef.h"
#include "pid.h"
//#include "encoder.h"
//#include "CRSF.h"
#include "math.h"
#include "motor.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
// 电机1（左前）引脚定义 - TIM2
#define PWM_M1_1_Pin GPIO_PIN_15
#define PWM_M1_1_GPIO_Port GPIOA
#define PWM_M1_2_Pin GPIO_PIN_3
#define PWM_M1_2_GPIO_Port GPIOB

// 电机2（左后）引脚定义 - TIM2（新增）
#define PWM_M2_1_Pin GPIO_PIN_2      // PA2 -> TIM2_CH3
#define PWM_M2_1_GPIO_Port GPIOA
#define PWM_M2_2_Pin GPIO_PIN_3      // PA3 -> TIM2_CH4
#define PWM_M2_2_GPIO_Port GPIOA

// 电机3（右前）引脚定义 - TIM3
#define PWM_M3_1_Pin GPIO_PIN_4
#define PWM_M3_1_GPIO_Port GPIOB
#define PWM_M3_2_Pin GPIO_PIN_5
#define PWM_M3_2_GPIO_Port GPIOB

// 电机4（右后）引脚定义 - TIM3（新增）
#define PWM_M4_1_Pin GPIO_PIN_0      // PB0 -> TIM3_CH3
#define PWM_M4_1_GPIO_Port GPIOB
#define PWM_M4_2_Pin GPIO_PIN_1      // PB1 -> TIM3_CH4
#define PWM_M4_2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
extern int motor_L_set;          // 左电机目标角度
extern int motor_R_set;          // 右电机目标角度
extern int motor_after_L_set;    // 左后电机目标角度
extern int motor_after_R_set;    // 右后电机目标角度
extern int Throttle,Pitch,Roll,Yaw;   // 遥控通道：油门2，俯仰1，横滚0，偏航3

#define PI 3.14159265358979323846f

// 快速三角函数（优化版本）
float fastCos(float angle);
float fastSin(float angle);
float Angle_Convert_Radians(float Ang);

// 电机控制相关函数声明
void motor_disable(void);    // 翅膀失能
void motor_stop(void);       // 翅膀暂停  
void motor_test(void);       // 电机测试

// 工具函数
long map(long x, long in_min, long in_max, long out_min, long out_max);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
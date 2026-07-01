/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : 扑翼飞行器主控程�?
  ******************************************************************************
  * @attention
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
  
/**
************************************************************************************************
* @brief    扑翼飞行器控制系�? - 独立驱动、姿态调节与参数控制
* @param    None
* @return   None
* @author   创源启明  2025.11.06
* @note     本程序实现四扑翼飞行器的控制，包括：
*           - 4个独立无刷电机驱�?
*           - AS5600角度传感器反�?
*           - PID闭环控制
*           - 多种飞行模式切换
*           - ELRS遥控器信号接�?
************************************************************************************************
**/
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "elrs.h"           // ELRS遥控器信号接收库
#include "AS5600_PWM.h"     // AS5600磁编码器PWM接口�?
#include <stdint.h>
#include <stdbool.h>
#include <math.h>          // 用于fabs函数
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* 用户自定义类型定�? */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/*******************************************************************************
 * 电机参数定义
 ******************************************************************************/
// 电机中位角度定义（原始PWM值，需根据实际机械结构校准�?
#define MOTOR_L_MIDPOINT_DEFAULT   2230// 左前电机中位角度
#define MOTOR_R_MIDPOINT_DEFAULT   1900   // 右前电机中位角度
#define MOTOR_LB_MIDPOINT_DEFAULT  2050   // 左后电机中位角度
#define MOTOR_RB_MIDPOINT_DEFAULT  2050   // 右后电机中位角度

// 扑翼摆动参数
#define FLAPPING_AMPLITUDE_BASE    850    // 扑翼基本摆动幅�?
#define FLAPPING_STEPS_PER_CYCLE   9      // 余弦表步数（20°-160°，共9个点�?
#define FLAPPING_MIN_STEP_MS       1      // 最小步进时间（防除零）
#define FLAPPING_CYCLE_STEPS       45U    // 一个完整周期步�?

// PID参数
#define PID_KP_HIGH                20     // 扑翼模式下的Kp�?       30
#define PID_KP_LOW                 8      // 姿态调节模式下的Kp�?   12
#define PID_KP_SAFE                5     // 安全模式下的Kp�?        12

// 平滑控制参数
#define SMOOTH_INTERVAL_MS         5      // 姿态调节平滑周�?
#define SMOOTH_MAX_STEP            70     // 最大单步变化量    原版�?70
#define SAFE_POSITION              1024   // 安全模式目标位置

// 自适应PID调整参数
#define ADAPTIVE_PID_THRESHOLD     200    // 误差阈值，大于此值启动自适应调整
#define ADAPTIVE_PID_MIN_RATIO     0.1f   // 最小调整比�?
#define ADAPTIVE_PID_MAX_RATIO     5.0f   // 最大调整比�?

/*******************************************************************************
 * 全局变量声明
 ******************************************************************************/
// 电机中位角度配置
int16_t motor_L_midpoint   = MOTOR_L_MIDPOINT_DEFAULT;   // 左前电机中位
int16_t motor_R_midpoint   = MOTOR_R_MIDPOINT_DEFAULT;   // 右前电机中位
int16_t motor_LB_midpoint  = MOTOR_LB_MIDPOINT_DEFAULT;  // 左后电机中位
int16_t motor_RB_midpoint  = MOTOR_RB_MIDPOINT_DEFAULT;  // 右后电机中位

// 扑翼状态机变量
static uint16_t g_phase16 = 0;           // 相位累加器（0-65535表示完整周期�?
static uint32_t g_last_ms = 0;           // 上一次循环时间戳（ms�?

// 电机目标角度变量
int motor_L_set;      // 左前电机目标角度
int motor_R_set;      // 右前电机目标角度
int motor_LB_set;     // 左后电机目标角度
int motor_RB_set;     // 右后电机目标角度

// 遥控器通道�?
int Throttle, Pitch, Roll, Yaw;  // 油门(通道2)，俯�?(通道1)，横�?(通道0)，偏�?(通道3)

/*******************************************************************************
 * 函数声明
 ******************************************************************************/
/**
  * @brief  数值映射函�?
  */
long map(long x, long in_min, long in_max, long out_min, long out_max);

/**
  * @brief  电机失能函数
  */
void motor_disable(void);

/**
  * @brief  电机暂停函数
  */
void motor_stop(void);

/**
  * @brief  电机测试函数
  */
void motor_test(void);

/**
  * @brief  快速绝对值计算（无分支优化）
  */
static inline uint16_t abs16_fast(int16_t x);

/**
  * @brief  Q15定点数乘�?
  */
static inline int16_t q15_mul(int16_t a, int16_t b);

/**
  * @brief  自适应PID调整函数
  * @param  pid_struct: PID结构体指针数�?
  * @param  errors: 角度误差数组
  * @param  num_motors: 电机数量
  * @param  base_kp: 基准Kp�?
  * @note   根据电机角度误差动态调整PID参数
  */
void adaptive_pid_adjust(pid_type_def *pid_struct[], float errors[], int num_motors, float base_kp);

/*******************************************************************************
 * 扑翼状态机变量
 ******************************************************************************/
static uint8_t  sm_idx = 13;        // 状态机索引�?0..8，对应余弦表�?
static int8_t   sm_dir = 1;         // 方向�?+1正向/-1反向
static uint32_t sm_next_tick = 0;   // 下一次步进的时刻（ms�?
static uint8_t  thr = 0;            // 油门�?

/*******************************************************************************
 * 余弦查找表（Q15格式�?
 ******************************************************************************/
static const int16_t COS_Q15_15[9] = {
    30784, 25133,  16384,   // 20°, 40°, 60° 余弦�?
    11207,     0, -11207,   // 80°, 100°, 120° 余弦�?
    -16384, -25133, -30784  // 140°, 160°, 180° 余弦�?
};

/*******************************************************************************
 * 辅助宏定�?
 ******************************************************************************/
#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))  // 最小值宏定义
#endif

#ifndef MAX
#define MAX(a, b) ((a) > (b) ? (a) : (b))  // 最大值宏定义
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */

/*******************************************************************************
 * 调试和状态变�?
 ******************************************************************************/
int ii1 = 0;               // 调试变量1
int ii2 = 100;             // 调试变量2
int motor_i = 0;           // 电机索引
int Take_off_Flag = 0;     // 起飞标志

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/*******************************************************************************
 * 主函�?
 ******************************************************************************/
int main(void)


{
    /* MCU初始化配�?--------------------------------------------------------*/
    HAL_Init();                     // 初始化HAL�?
    SystemClock_Config();           // 配置系统时钟

    /* 外设初始�? */
    MX_GPIO_Init();                 // GPIO初始�?
    MX_DMA_Init();                  // DMA初始�?
    MX_USART1_UART_Init();          // 串口1初始化（ELRS通信�?
    MX_TIM2_Init();                 // TIM2初始化（电机PWM1-4�?
    MX_TIM3_Init();                 // TIM3初始化（电机PWM5-8�?
    MX_TIM1_Init();                 // TIM1初始�?
    MX_ADC1_Init();                 // ADC1初始�?

    /* USER CODE BEGIN 2 */
    /* 外设启动与系统启�? */
    
    HAL_ADCEx_Calibration_Start(&hadc1);  // ADC校准
    
    /* 启动TIM2的PWM输出（控制电�?1�?2�? */
    HAL_TIM_Base_Start(&htim2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);

    /* 启动TIM3的PWM输出（控制电�?3�?4�? */
    HAL_TIM_Base_Start(&htim3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
    
    HAL_Delay(1000);               // 等待1秒，确保系统稳定
    
    MX_USART1_UART_Init();         // 重新初始化串�?
    ELRS_Init();                   // ELRS接收机初始化
    Chassis_PID_Init();            // 电机PID控制器初始化
    
    g_last_ms = HAL_GetTick();     // 初始化时间戳

    /* USER CODE END 2 */

    /* 主循�? */
    /* USER CODE BEGIN WHILE */
    while (1)
    {
        /* USER CODE END WHILE */
        /* USER CODE BEGIN 3 */
        
        /***********************************************************************
         * 主循环执行步骤：
         * 1. 读取传感器数�?
         * 2. 读取遥控器信�?
         * 3. 根据安全开关和模式选择执行不同控制策略
         ***********************************************************************/
        
        // 步骤1: 读取AS5600角度传感器数�?
        StarAndGetResult();
        
        // 步骤2: 获取当前时间�?
        uint32_t now = HAL_GetTick();
        
        // 步骤3: 检查遥控器安全开关状�?
        if (elrs_data.Switch == 1)  // 安全开关已打开
        {
            // Switch1=0: 前后电机统一控制
            if (elrs_data.Switch1 == 0)
            {
                // 模式2: 扑翼飞行模式（实现飞行扑动）
                if (elrs_data.Mode == 2)
                {
                    /*******************************************************************
                     * 扑翼飞行模式控制逻辑�?
                     * 1. 设置较高的PID参数以获得更快的响应
                     * 2. 根据偏航通道调整左右扑翼对称�?
                     * 3. 使用状态机实现正弦波扑�?
                     * 4. 根据油门调整扑动频率
                     *******************************************************************/
                    
                    // 设置PID参数（较高增益，快速响应）
                    motor_1_pid.Kp = PID_KP_HIGH;  // 左前电机Kp
                    motor_3_pid.Kp = PID_KP_HIGH;  // 右前电机Kp
                    motor_2_pid.Kp = PID_KP_HIGH;  // 左后电机Kp
                    motor_4_pid.Kp = PID_KP_HIGH;  // 右后电机Kp
                    
                    // 计算各电机的基准角度 = 中位 + 遥控器偏移调�?
                    const int16_t motor_L_ready  = motor_L_midpoint  + elrs_data.midpoint + elrs_data.midpoint_1;
                    const int16_t motor_R_ready  = motor_R_midpoint  + elrs_data.midpoint + elrs_data.midpoint_1;
                    const int16_t motor_LB_ready = motor_LB_midpoint + elrs_data.midpoint + elrs_data.midpoint_1;
                    const int16_t motor_RB_ready = motor_RB_midpoint + elrs_data.midpoint + elrs_data.midpoint_1;
                    
                    // 计算扑翼摆动幅值（根据偏航通道调整左右对称性）
                    const int16_t ampL  = (int16_t)(FLAPPING_AMPLITUDE_BASE - elrs_data.Yaw);
                    const int16_t ampR  = (int16_t)(FLAPPING_AMPLITUDE_BASE + elrs_data.Yaw);
                    const int16_t ampLB = (int16_t)(FLAPPING_AMPLITUDE_BASE +elrs_data.Yaw );
                    const int16_t ampRB = (int16_t)(FLAPPING_AMPLITUDE_BASE - elrs_data.Yaw);
                    
                    thr = elrs_data.Throttle;  // 获取当前油门�?
                    
                    // 根据油门计算扑动频率
                    const uint32_t STEPS = FLAPPING_CYCLE_STEPS;
                    uint32_t step_ms = (5000U + (STEPS * thr / 2U)) / (STEPS * thr);
                    if (step_ms == 0) step_ms = FLAPPING_MIN_STEP_MS;
                    
                    uint32_t tick = HAL_GetTick();
                    
                    /*******************************************************************
                     * 状态机步进逻辑
                     *******************************************************************/
                    if ((int32_t)(tick - sm_next_tick) >= 0)
                    {
                        int16_t c = COS_Q15_15[sm_idx];
                        
                        Wings_Data.Wings_motor[0].Target_Angle = (int16_t)(motor_L_ready  - q15_mul(ampL,  c));
                        Wings_Data.Wings_motor[1].Target_Angle = (int16_t)(motor_LB_ready - q15_mul(ampLB, c));
                        Wings_Data.Wings_motor[2].Target_Angle = (int16_t)(motor_R_ready  - q15_mul(ampR,  c));
                        Wings_Data.Wings_motor[3].Target_Angle = (int16_t)(motor_RB_ready - q15_mul(ampRB, c));
                        
                        sm_next_tick += step_ms;
                        
                        if (sm_dir > 0) 
                        {
                            if (++sm_idx >= 8)
                            { 
                                sm_idx = 8;
                                sm_dir = -1;
                            }
                        } 
                        else 
                        {
                            if (sm_idx-- == 0)
                            { 
                                sm_idx = 0;
                                sm_dir = +1;
                            }
                        }
                    }
                    
                    StarAndGetResult();
                    Motor_PID_Control();
                }
                // 模式1: 姿态调节模式（地面调试用）
                else if (elrs_data.Mode == 1)
                {
                    /*******************************************************************
                     * 姿态调节模式控制逻辑�?
                     * 1. 设置较低的PID参数以获得平滑响�?
                     * 2. 根据遥控器输入计算目标角�?
                     * 3. 使用平滑过渡避免突变
                     * 4. 支持手动偏航调节
                     * 5. 加入自适应PID调整
                     *******************************************************************/
                    
                    // 计算各电机目标角度（中位+偏移+偏航调节�?
                    const int16_t ago_targetL  = motor_L_midpoint  + elrs_data.midpoint + elrs_data.midpoint_1 - elrs_data.Yaw;
                    const int16_t ago_targetR  = motor_R_midpoint  + elrs_data.midpoint + elrs_data.midpoint_1 + elrs_data.Yaw;
                    const int16_t ago_targetLB = motor_LB_midpoint + elrs_data.midpoint + elrs_data.midpoint_1 + elrs_data.Yaw;
                    const int16_t ago_targetRB = motor_RB_midpoint + elrs_data.midpoint + elrs_data.midpoint_1 - elrs_data.Yaw;
                    
                    // 从当前值缓慢移动到目标�?
                    static uint32_t last_smooth_time = 0;
                    const uint32_t SMOOTH_INTERVAL = SMOOTH_INTERVAL_MS;
                    const int16_t MAX_STEP = SMOOTH_MAX_STEP;
                    
                    if ((int32_t)(now - last_smooth_time) >= SMOOTH_INTERVAL)
                    {
                        // 左前电机平滑移动
                        int16_t current0 = Wings_Data.Wings_motor[0].Corrective_Angle;
                        if (current0 < ago_targetL) {
                            current0 += MIN(MAX_STEP, ago_targetL - current0);
                        } else if (current0 > ago_targetL) {
                            current0 -= MIN(MAX_STEP, current0 - ago_targetL);
                        }
                        Wings_Data.Wings_motor[0].Target_Angle = current0;
                        
                        // 左后电机平滑移动
                        int16_t current1 = Wings_Data.Wings_motor[1].Corrective_Angle;
                        if (current1 < ago_targetLB) {
                            current1 += MIN(MAX_STEP, ago_targetLB - current1);
                        } else if (current1 > ago_targetLB) {
                            current1 -= MIN(MAX_STEP, current1 - ago_targetLB);
                        }
                        Wings_Data.Wings_motor[1].Target_Angle = current1;
                        
                        // 右前电机平滑移动
                        int16_t current2 = Wings_Data.Wings_motor[2].Corrective_Angle;
                        if (current2 < ago_targetR) {
                            current2 += MIN(MAX_STEP, ago_targetR - current2);
                        } else if (current2 > ago_targetR) {
                            current2 -= MIN(MAX_STEP, current2 - ago_targetR);
                        }
                        Wings_Data.Wings_motor[2].Target_Angle = current2;
                        
                        // 右后电机平滑移动
                        int16_t current3 = Wings_Data.Wings_motor[3].Corrective_Angle;
                        if (current3 < ago_targetRB) {
                            current3 += MIN(MAX_STEP, ago_targetRB - current3);
                        } else if (current3 > ago_targetRB) {
                            current3 -= MIN(MAX_STEP, current3 - ago_targetRB);
                        }
                        Wings_Data.Wings_motor[3].Target_Angle = current3;
                        
                        last_smooth_time = now;
                    }
                    
                    /*******************************************************************
                     * 自适应PID调整逻辑（基于最初版本）
                     * 原理：当电机角度误差较大时，根据相邻电机的误差比例调整Kp
                     * 避免单个电机响应过快导致振荡
                     *******************************************************************/
                    
                    // 计算各电机角度误�?
                    float error_L  = fabs(ago_targetL  - Wings_Data.Wings_motor[0].Corrective_Angle);
                    float error_R  = fabs(ago_targetR  - Wings_Data.Wings_motor[2].Corrective_Angle);
                    float error_LB = fabs(ago_targetLB - Wings_Data.Wings_motor[1].Corrective_Angle);
                    float error_RB = fabs(ago_targetRB - Wings_Data.Wings_motor[3].Corrective_Angle);
                    
                    // 检查是否需要自适应调整（任意电机误差超过阈值）
                    if (error_L > ADAPTIVE_PID_THRESHOLD && error_R > ADAPTIVE_PID_THRESHOLD)
                    {
                        // 根据最初版本的逻辑：Kp = 基准Kp * (本机误差/相邻机误�?)
                        // 但需要加入限幅防止过大或过小
                        
                        // 左前电机：参考右前电机误�?
                        float ratio_L = (error_R > 0) ? (error_L / error_R) : 1.0f;
                        ratio_L = MAX(ADAPTIVE_PID_MIN_RATIO, MIN(ADAPTIVE_PID_MAX_RATIO, ratio_L));
                        motor_1_pid.Kp = PID_KP_LOW * ratio_L;
                        
                        // 右前电机：参考左前电机误�?
                        float ratio_R = (error_L > 0) ? (error_R / error_L) : 1.0f;
                        ratio_R = MAX(ADAPTIVE_PID_MIN_RATIO, MIN(ADAPTIVE_PID_MAX_RATIO, ratio_R));
                        motor_3_pid.Kp = PID_KP_LOW * ratio_R;
                        
                        // 左后电机：参考右后电机误�?
                        float ratio_LB = (error_RB > 0) ? (error_LB / error_RB) : 1.0f;
                        ratio_LB = MAX(ADAPTIVE_PID_MIN_RATIO, MIN(ADAPTIVE_PID_MAX_RATIO, ratio_LB));
                        motor_2_pid.Kp = PID_KP_LOW * ratio_LB;
                        
                        // 右后电机：参考左后电机误�?
                        float ratio_RB = (error_LB > 0) ? (error_RB / error_LB) : 1.0f;
                        ratio_RB = MAX(ADAPTIVE_PID_MIN_RATIO, MIN(ADAPTIVE_PID_MAX_RATIO, ratio_RB));
                        motor_4_pid.Kp = PID_KP_LOW * ratio_RB;
                    }
                    else
                    {
                        // 误差较小，使用基准Kp
                        motor_1_pid.Kp = PID_KP_LOW;
                        motor_3_pid.Kp = PID_KP_LOW;
                        motor_2_pid.Kp = PID_KP_LOW;
                        motor_4_pid.Kp = PID_KP_LOW;
                    }
                    
                    StarAndGetResult();
                    Motor_PID_Control();
                    
                    sm_idx = 0;
                    sm_dir = 1;
                    sm_next_tick = HAL_GetTick();
                }
                // 模式0: 安全模式（所有电机回到中位）
                else
                {
                    /*******************************************************************
                     * 安全模式控制逻辑�?
                     * 1. 设置较低的PID参数
                     * 2. 缓慢将所有电机移动到安全位置
                     * 3. 加入自适应PID调整，防止突然断电导致的机械冲击
                     *******************************************************************/
                    
                    const int16_t ago_targetL  = SAFE_POSITION;
                    const int16_t ago_targetR  = SAFE_POSITION;
                    const int16_t ago_targetLB = SAFE_POSITION;
                    const int16_t ago_targetRB = SAFE_POSITION;
                    
                    // 从当前值缓慢移动到安全位置
                    static uint32_t last_smooth_time_mode0 = 0;
                    const uint32_t SMOOTH_INTERVAL = 10;
                    const int16_t MAX_STEP = SMOOTH_MAX_STEP;
                    
                    if ((int32_t)(now - last_smooth_time_mode0) >= SMOOTH_INTERVAL)
                    {
                        int16_t current0 = Wings_Data.Wings_motor[0].Corrective_Angle;
                        if (current0 < ago_targetL) {
                            current0 += MIN(MAX_STEP, ago_targetL - current0);
                        } else if (current0 > ago_targetL) {
                            current0 -= MIN(MAX_STEP, current0 - ago_targetL);
                        }
                        Wings_Data.Wings_motor[0].Target_Angle = current0;
                        
                        int16_t current1 = Wings_Data.Wings_motor[1].Corrective_Angle;
                        if (current1 < ago_targetLB) {
                            current1 += MIN(MAX_STEP, ago_targetLB - current1);
                        } else if (current1 > ago_targetLB) {
                            current1 -= MIN(MAX_STEP, current1 - ago_targetLB);
                        }
                        Wings_Data.Wings_motor[1].Target_Angle = current1;
                        
                        int16_t current2 = Wings_Data.Wings_motor[2].Corrective_Angle;
                        if (current2 < ago_targetR) {
                            current2 += MIN(MAX_STEP, ago_targetR - current2);
                        } else if (current2 > ago_targetR) {
                            current2 -= MIN(MAX_STEP, current2 - ago_targetR);
                        }
                        Wings_Data.Wings_motor[2].Target_Angle = current2;
                        
                        int16_t current3 = Wings_Data.Wings_motor[3].Corrective_Angle;
                        if (current3 < ago_targetRB) {
                            current3 += MIN(MAX_STEP, ago_targetRB - current3);
                        } else if (current3 > ago_targetRB) {
                            current3 -= MIN(MAX_STEP, current3 - ago_targetRB);
                        }
                        Wings_Data.Wings_motor[3].Target_Angle = current3;
                        
                        last_smooth_time_mode0 = now;
                    }
                    
                    /*******************************************************************
                     * 安全模式下的自适应PID调整（基于最初版本）
                     * 注意：最初版本中这个逻辑只在安全模式中有
                     *******************************************************************/
                    
                    // 计算角度误差
                    float error_L  = fabs(ago_targetL  - Wings_Data.Wings_motor[0].Corrective_Angle);
                    float error_R  = fabs(ago_targetR  - Wings_Data.Wings_motor[2].Corrective_Angle);
                    float error_LB = fabs(ago_targetLB - Wings_Data.Wings_motor[1].Corrective_Angle);
                    float error_RB = fabs(ago_targetRB - Wings_Data.Wings_motor[3].Corrective_Angle);
                    
                    // 检查是否需要自适应调整
                    if (error_L > ADAPTIVE_PID_THRESHOLD && error_R > ADAPTIVE_PID_THRESHOLD &&
                        error_LB > ADAPTIVE_PID_THRESHOLD && error_RB > ADAPTIVE_PID_THRESHOLD)
                    {
                        // 最初版本的自适应逻辑：Kp = 12 * (本机误差/相邻机误�?)
                        // 这里扩展�?4个电机的版本
                        
                        // 计算对角的误差比�?
                        float diag_ratio_1 = (error_R > 0) ? (error_L / error_R) : 1.0f;
                        float diag_ratio_2 = (error_LB > 0) ? (error_RB / error_LB) : 1.0f;
                        
                        // 平均对角比例
                        float avg_ratio = (diag_ratio_1 + diag_ratio_2) / 2.0f;
                        
                        // 应用比例调整，但加入限幅
                        avg_ratio = MAX(ADAPTIVE_PID_MIN_RATIO, MIN(ADAPTIVE_PID_MAX_RATIO, avg_ratio));
                        
                        // 左前和右后电机使用相同调�?
                        motor_1_pid.Kp = PID_KP_SAFE * (error_L / error_R);
                        motor_4_pid.Kp = PID_KP_SAFE * (error_RB / error_LB);
                        
                        // 右前和左后电机使用相反比�?
                        motor_3_pid.Kp = PID_KP_SAFE * (error_R / error_L);
                        motor_2_pid.Kp = PID_KP_SAFE * (error_LB / error_RB);
                        
                        // 加入限幅防止过大或过�?
                        motor_1_pid.Kp = MAX(ADAPTIVE_PID_MIN_RATIO * PID_KP_SAFE, 
                                            MIN(ADAPTIVE_PID_MAX_RATIO * PID_KP_SAFE, motor_1_pid.Kp));
                        motor_2_pid.Kp = MAX(ADAPTIVE_PID_MIN_RATIO * PID_KP_SAFE, 
                                            MIN(ADAPTIVE_PID_MAX_RATIO * PID_KP_SAFE, motor_2_pid.Kp));
                        motor_3_pid.Kp = MAX(ADAPTIVE_PID_MIN_RATIO * PID_KP_SAFE, 
                                            MIN(ADAPTIVE_PID_MAX_RATIO * PID_KP_SAFE, motor_3_pid.Kp));
                        motor_4_pid.Kp = MAX(ADAPTIVE_PID_MIN_RATIO * PID_KP_SAFE, 
                                            MIN(ADAPTIVE_PID_MAX_RATIO * PID_KP_SAFE, motor_4_pid.Kp));
                    }
                    else
                    {
                        // 误差较小，使用基准Kp
                        motor_1_pid.Kp = PID_KP_SAFE;
                        motor_3_pid.Kp = PID_KP_SAFE;
                        motor_2_pid.Kp = PID_KP_SAFE;
                        motor_4_pid.Kp = PID_KP_SAFE;
                    }
                    
                    StarAndGetResult();
                    Motor_PID_Control();
                    
                    sm_idx = 0;
                    sm_dir = 1;
                    sm_next_tick = HAL_GetTick();
                }
            }  // Switch1=0 结束
            // Switch1=1: 前后电机分别控制
            else
            {
                /*******************************************************************
                 * 单控模式：前后电机独立控�?
                 * 1. 前电机由Mode控制�?0=安全�?1=姿态调整）
                 * 2. 后电机由Mode1控制�?0=安全�?1=姿态调整）
                 * 3. 两个控制逻辑独立，互不干�?
                 *******************************************************************/
                
                // 前电机控�?
                if (elrs_data.Mode == 0)  // 前电机安全模�?
                {
                    /*******************************************************************
                     * 前电机安全模式控制逻辑（只控制前电机）�?
                     * 1. 设置前电机的PID参数
                     * 2. 缓慢将前电机移动到安全位�?
                     * 3. 后电机保持当前位�?
                     *******************************************************************/
                    
                    // 设置PID参数（只设置前电机）
                    motor_1_pid.Kp = 9;  // 左前电机Kp   原版�?9
                    motor_3_pid.Kp = 9;  // 右前电机Kp   原版�?9
                    
                    // 前电机安全位置（回到1024位置�?
                    const int16_t ago_targetL  = SAFE_POSITION;  // 左前
                    const int16_t ago_targetR  = SAFE_POSITION;  // 右前
                    
                    // 从当前值缓慢移动到安全位置
                    static uint32_t last_smooth_time_front_safe = 0;
                    const uint32_t SMOOTH_INTERVAL = 15;  // 平滑间隔10ms     原版�?10
                    const int16_t MAX_STEP = SMOOTH_MAX_STEP;  // 最大步进�?
                    
                    if ((int32_t)(now - last_smooth_time_front_safe) >= SMOOTH_INTERVAL)
                    {
                        // 左前电机平滑移动到安全位�?
                        int16_t current0 = Wings_Data.Wings_motor[0].Corrective_Angle;
                        if (current0 < ago_targetL) {
                            current0 += MIN(MAX_STEP, ago_targetL - current0);
                        } else if (current0 > ago_targetL) {
                            current0 -= MIN(MAX_STEP, current0 - ago_targetL);
                        }
                        Wings_Data.Wings_motor[0].Target_Angle = current0;
                        
                        // 右前电机平滑移动到安全位�?
                        int16_t current2 = Wings_Data.Wings_motor[2].Corrective_Angle;
                        if (current2 < ago_targetR) {
                            current2 += MIN(MAX_STEP, ago_targetR - current2);
                        } else if (current2 > ago_targetR) {
                            current2 -= MIN(MAX_STEP, current2 - ago_targetR);
                        }
                        Wings_Data.Wings_motor[2].Target_Angle = current2;
                        
                        // 注意：不设置后电机的Target_Angle，保持它们原来的�?
                        // Wings_Data.Wings_motor[1].Target_Angle 保持不变
                        // Wings_Data.Wings_motor[3].Target_Angle 保持不变
                        
                        last_smooth_time_front_safe = now;  // 更新平滑计时�?
                    }
                    
                    // 重置扑翼状态机
                    sm_idx = 0;
                    sm_dir = 1;
                    sm_next_tick = HAL_GetTick();
                }
                else if (elrs_data.Mode == 1)  // 前电机姿态调整模�?
                {
                    /*******************************************************************
                     * 前电机姿态调整模式控制逻辑（只控制前电机）�?
                     * 1. 设置前电机的PID参数
                     * 2. 根据遥控器输入计算前电机目标角度
                     * 3. 使用平滑过渡避免突变
                     *******************************************************************/
                    
                    // 设置PID参数（只设置前电机）
                    motor_1_pid.Kp = 9;  // 左前电机Kp
                    motor_3_pid.Kp = 9;  // 右前电机Kp
                    
                    // 只计算前电机目标角度（中�?+偏移+偏航调节�?
                    const int16_t ago_targetL  = motor_L_midpoint  + elrs_data.midpoint + elrs_data.midpoint_1 - 3 * elrs_data.Yaw;
                    const int16_t ago_targetR  = motor_R_midpoint  + elrs_data.midpoint + elrs_data.midpoint_1 + 3 * elrs_data.Yaw;
                    
                    // 从当前值缓慢移动到目标�?
                    static uint32_t last_smooth_time_front = 0;
                    const uint32_t SMOOTH_INTERVAL = SMOOTH_INTERVAL_MS;  // 平滑间隔5ms
                    const int16_t MAX_STEP = SMOOTH_MAX_STEP;             // 最大步进�?
                    
                    if ((int32_t)(now - last_smooth_time_front) >= SMOOTH_INTERVAL)
                    {
                        // 左前电机平滑移动
                        int16_t current0 = Wings_Data.Wings_motor[0].Corrective_Angle;
                        if (current0 < ago_targetL) {
                            current0 += MIN(MAX_STEP, ago_targetL - current0);
                        } else if (current0 > ago_targetL) {
                            current0 -= MIN(MAX_STEP, current0 - ago_targetL);
                        }
                        Wings_Data.Wings_motor[0].Target_Angle = current0;
                        
                        // 右前电机平滑移动
                        int16_t current2 = Wings_Data.Wings_motor[2].Corrective_Angle;
                        if (current2 < ago_targetR) {
                            current2 += MIN(MAX_STEP, ago_targetR - current2);
                        } else if (current2 > ago_targetR) {
                            current2 -= MIN(MAX_STEP, current2 - ago_targetR);
                        }
                        Wings_Data.Wings_motor[2].Target_Angle = current2;
                        
                        // 注意：不设置后电机的Target_Angle
                        // 后电机会由后电机控制逻辑处理
                        
                        last_smooth_time_front = now;  // 更新平滑计时�?
                    }
                    
                    // 重置扑翼状态机
                    sm_idx = 0;
                    sm_dir = 1;
                    sm_next_tick = HAL_GetTick();
                }
                
                // 后电机控�?
                if (elrs_data.Mode1 == 0)  // 后电机安全模�?
                {
                    /*******************************************************************
                     * 后电机安全模式控制逻辑（只控制后电机）�?
                     * 1. 设置后电机的PID参数
                     * 2. 缓慢将后电机移动到安全位�?
                     * 3. 前电机保持当前位�?
                     *******************************************************************/
                    
                    // 设置PID参数（只设置后电机）
                    motor_2_pid.Kp = 9;  // 左后电机Kp
                    motor_4_pid.Kp = 9;  // 右后电机Kp
                    
                    // 后电机安全位置（回到1024位置�?
                    const int16_t ago_targetLB = SAFE_POSITION;  // 左后
                    const int16_t ago_targetRB = SAFE_POSITION;  // 右后
                    
                    // 从当前值缓慢移动到安全位置
                    static uint32_t last_smooth_time_rear_safe = 0;
                    const uint32_t SMOOTH_INTERVAL = 15;  // 平滑间隔10ms 原来10
                    const int16_t MAX_STEP = SMOOTH_MAX_STEP;  // 最大步进�?
                    
                    if ((int32_t)(now - last_smooth_time_rear_safe) >= SMOOTH_INTERVAL)
                    {
                        // 左后电机平滑移动到安全位�?
                        int16_t current1 = Wings_Data.Wings_motor[1].Corrective_Angle;
                        if (current1 < ago_targetLB) {
                            current1 += MIN(MAX_STEP, ago_targetLB - current1);
                        } else if (current1 > ago_targetLB) {
                            current1 -= MIN(MAX_STEP, current1 - ago_targetLB);
                        }
                        Wings_Data.Wings_motor[1].Target_Angle = current1;
                        
                        // 右后电机平滑移动到安全位�?
                        int16_t current3 = Wings_Data.Wings_motor[3].Corrective_Angle;
                        if (current3 < ago_targetRB) {
                            current3 += MIN(MAX_STEP, ago_targetRB - current3);
                        } else if (current3 > ago_targetRB) {
                            current3 -= MIN(MAX_STEP, current3 - ago_targetRB);
                        }
                        Wings_Data.Wings_motor[3].Target_Angle = current3;
                        
                        // 注意：不设置前电机的Target_Angle
                        // 前电机会由前电机控制逻辑处理
                        
                        last_smooth_time_rear_safe = now;  // 更新平滑计时�?
                    }
                }
                else if (elrs_data.Mode1 == 1)  // 后电机姿态调整模�?
                {
                    /*******************************************************************
                     * 后电机姿态调整模式控制逻辑（只控制后电机）�?
                     * 1. 设置后电机的PID参数
                     * 2. 根据遥控器输入计算后电机目标角度
                     * 3. 使用平滑过渡避免突变
                     * 注意：后电机偏航控制方向与前电机相反
                     *******************************************************************/
                    
                    // 设置PID参数（只设置后电机）
                    motor_2_pid.Kp = 9;  // 左后电机Kp
                    motor_4_pid.Kp = 9;  // 右后电机Kp
                    
                    // 只计算后电机目标角度（中�?+偏移+偏航调节�?
                    const int16_t ago_targetLB = motor_LB_midpoint + elrs_data.midpoint + elrs_data.midpoint_1 + 3 * elrs_data.Yaw;
                    const int16_t ago_targetRB = motor_RB_midpoint + elrs_data.midpoint + elrs_data.midpoint_1 - 3 * elrs_data.Yaw;
                    
                    // 从当前值缓慢移动到目标�?
                    static uint32_t last_smooth_time_rear = 0;
                    const uint32_t SMOOTH_INTERVAL = SMOOTH_INTERVAL_MS;  // 平滑间隔5ms
                    const int16_t MAX_STEP = SMOOTH_MAX_STEP;             // 最大步进�?
                    
                    if ((int32_t)(now - last_smooth_time_rear) >= SMOOTH_INTERVAL)
                    {
                        // 左后电机平滑移动
                        int16_t current1 = Wings_Data.Wings_motor[1].Corrective_Angle;
                        if (current1 < ago_targetLB) {
                            current1 += MIN(MAX_STEP, ago_targetLB - current1);
                        } else if (current1 > ago_targetLB) {
                            current1 -= MIN(MAX_STEP, current1 - ago_targetLB);
                        }
                        Wings_Data.Wings_motor[1].Target_Angle = current1;
                        
                        // 右后电机平滑移动
                        int16_t current3 = Wings_Data.Wings_motor[3].Corrective_Angle;
                        if (current3 < ago_targetRB) {
                            current3 += MIN(MAX_STEP, ago_targetRB - current3);
                        } else if (current3 > ago_targetRB) {
                            current3 -= MIN(MAX_STEP, current3 - ago_targetRB);
                        }
                        Wings_Data.Wings_motor[3].Target_Angle = current3;
                        
                        // 注意：不设置前电机的Target_Angle
                        // 前电机会由前电机控制逻辑处理
                        
                        last_smooth_time_rear = now;  // 更新平滑计时�?
                    }
                }
                
                // 在单控模式下，确保前电机Mode=2时，后电机控制仍然有�?
                // 这里不需要额外处理，因为前后电机控制逻辑是独立的
                
                // 更新传感器数据并执行PID控制
                StarAndGetResult();
                Motor_PID_Control();
            }  // Switch1=1 结束
        }
        // 安全开关关闭：紧急停止所有电�?
        else
        {
            motor_disable();
            
            // 重置扑翼状态机
            sm_idx = 0;
            sm_dir = 1;
            sm_next_tick = HAL_GetTick();
        }
    }
    /* USER CODE END 3 */
}

/*******************************************************************************
 * 时钟配置函数
 ******************************************************************************/
void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
    
    HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
    
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
    RCC_OscInitStruct.PLL.PLLN = 8;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV4;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        Error_Handler();
    }
    
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                  | RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        Error_Handler();
    }
}

/* USER CODE BEGIN 4 */
/*******************************************************************************
 * 串口接收中断回调函数
 ******************************************************************************/
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)  
{
    if (huart == &huart1)
    {
        ELRS_UARTE_RxCallback(Size);
    }
}
/* USER CODE END 4 */

/*******************************************************************************
 * 错误处理函数
 ******************************************************************************/
void Error_Handler(void)
{
    __disable_irq();
    
    while (1)
    {
    }
}

/*******************************************************************************
 * 断言失败处理函数（调试用�?
 ******************************************************************************/
#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t *file, uint32_t line)
{
}
#endif /* USE_FULL_ASSERT */

/*******************************************************************************
 * 辅助函数实现
 ******************************************************************************/
/* USER CODE BEGIN 5 */

long map(long x, long in_min, long in_max, long out_min, long out_max) 
{
    const long run = in_max - in_min;
    if (run == 0) {
        return -1;
    }
    const long rise = out_max - out_min;
    const long delta = x - in_min;
    return (delta * rise) / run + out_min;
}

void motor_disable(void)
{
    TIM2->CCR1 = 0;
    TIM2->CCR2 = 0;
    TIM2->CCR3 = 0;
    TIM2->CCR4 = 0;
    TIM3->CCR1 = 0;
    TIM3->CCR2 = 0;
    TIM3->CCR3 = 0;
    TIM3->CCR4 = 0;
}

void motor_stop(void)
{
    TIM2->CCR1 = 19999;
    TIM2->CCR2 = 19999;
    TIM2->CCR3 = 19999;
    TIM2->CCR4 = 19999;
    TIM3->CCR1 = 19999;
    TIM3->CCR2 = 19999;
    TIM3->CCR3 = 19999;
    TIM3->CCR4 = 19999;
}

void motor_test(void)
{
    Set_Pwm(0, -1200, 0, 0);
    HAL_Delay(100);
    Set_Pwm(0, 0, 0, 0);
    while(1);
}

static inline uint16_t abs16_fast(int16_t x) 
{
    int16_t m = x >> 15;
    return (uint16_t)((x ^ m) - m);
}

static inline int16_t q15_mul(int16_t a, int16_t b) 
{
    return (int16_t)(((int32_t)a * (int32_t)b) >> 15);
}

/**
  * @brief  自适应PID调整函数
  * @param  pid_struct: PID结构体指针数�?
  * @param  errors: 角度误差数组
  * @param  num_motors: 电机数量
  * @param  base_kp: 基准Kp�?
  * @note   根据电机角度误差动态调整PID参数
  *         原理：当一个电机误差较大时，降低其Kp，提高相邻电机Kp
  *         这样可以避免振荡，同时保持整体响应速度
  */
void adaptive_pid_adjust(pid_type_def *pid_struct[], float errors[], int num_motors, float base_kp)
{
    if (num_motors < 2) return;
    
    // 计算平均误差
    float avg_error = 0;
    for (int i = 0; i < num_motors; i++) {
        avg_error += errors[i];
    }
    avg_error /= num_motors;
    
    // 如果平均误差较小，使用基准Kp
    if (avg_error < ADAPTIVE_PID_THRESHOLD) {
        for (int i = 0; i < num_motors; i++) {
            pid_struct[i]->Kp = base_kp;
        }
        return;
    }
    
    // 根据误差比例调整Kp
    for (int i = 0; i < num_motors; i++) {
        // 计算相邻电机索引
        int prev_idx = (i - 1 + num_motors) % num_motors;
        int next_idx = (i + 1) % num_motors;
        
        // 计算相邻电机的平均误�?
        float neighbor_avg_error = (errors[prev_idx] + errors[next_idx]) / 2.0f;
        
        // 计算调整比例：本机误�?/相邻机平均误�?
        float ratio = (neighbor_avg_error > 0) ? (errors[i] / neighbor_avg_error) : 1.0f;
        
        // 限幅
        ratio = MAX(ADAPTIVE_PID_MIN_RATIO, MIN(ADAPTIVE_PID_MAX_RATIO, ratio));
        
        // 应用调整
        pid_struct[i]->Kp = base_kp * ratio;
    }
}
/* USER CODE END 5 */
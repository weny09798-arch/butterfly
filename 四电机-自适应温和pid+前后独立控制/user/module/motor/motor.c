#include "motor.h"

/**
************************************************************************************************
* @brief    电机控制任务，运动算法
* @param    None
* @return   None
* @author   创源启明		2025.11.06
************************************************************************************************
**/

// 外部变量声明 - 存储翼部（翅膀）数据，包括电机状态和控制参数
WINGS_DATA Wings_Data;

/************ 运动速度变量定义 *************/
int motor_1_set_pwm;  // 电机1的PWM设定值
int motor_3_set_pwm;  // 电机3的PWM设定值
int motor_2_set_pwm;  // 电机2的PWM设定值
int motor_4_set_pwm;  // 电机4的PWM设定值

/************ PID结构体变量定义 *************/
pid_type_def motor_1_pid, motor_3_pid, motor_2_pid, motor_4_pid;  // 四个电机的PID控制器结构体

/************** 速度PID限幅设置 *************/
const float motore_max_out = 18999, motor_max_iout = 4095;  // 速度环总输出限幅，积分项限幅//18999 12000

/**
  * @brief  底盘PID初始化函数
  * @note   初始化四个电机的PID控制器，使用相同的PID参数
  * @param  None
  * @retval None
  */
void Chassis_PID_Init(void) 
{
    // 所有电机使用相同的PID参数
    const float motor_speed_pid[3] = {MOTOR_1_SPEED_PID_KP, MOTOR_1_SPEED_PID_KI, MOTOR_1_SPEED_PID_KD};
    
    // 初始化四个电机的PID控制器
    PID_init(&motor_1_pid, PID_POSITION, motor_speed_pid, motore_max_out, motor_max_iout);
    PID_init(&motor_3_pid, PID_POSITION, motor_speed_pid, motore_max_out, motor_max_iout);
    PID_init(&motor_2_pid, PID_POSITION, motor_speed_pid, motore_max_out, motor_max_iout);
    PID_init(&motor_4_pid, PID_POSITION, motor_speed_pid, motore_max_out, motor_max_iout);
}

/**
  * @brief  电机PID控制函数
  * @note   根据目标角度和校正角度计算每个电机的PWM输出
  *         电机1和电机3逻辑相同，电机2和电机4逻辑相同
  * @param  None
  * @retval None
  */
void Motor_PID_Control(void)
{
    // 电机1和电机3控制逻辑：
    // 通过PID计算电机输出，PID输入为校正角度和目标角度
    // 电机3输出取反，可能是为了实现对称控制
    Wings_Data.Wings_motor[0].Target_Speed = motor_1_set_pwm = 
        PID_calc(&motor_1_pid, Wings_Data.Wings_motor[0].Corrective_Angle, Wings_Data.Wings_motor[0].Target_Angle);
    Wings_Data.Wings_motor[2].Target_Speed = motor_3_set_pwm = 
        -PID_calc(&motor_3_pid, Wings_Data.Wings_motor[2].Corrective_Angle, Wings_Data.Wings_motor[2].Target_Angle);
    
    // 电机2和电机4控制逻辑（与电机1和3相同）
    Wings_Data.Wings_motor[1].Target_Speed = motor_2_set_pwm = 
        PID_calc(&motor_2_pid, Wings_Data.Wings_motor[1].Corrective_Angle, Wings_Data.Wings_motor[1].Target_Angle);
    Wings_Data.Wings_motor[3].Target_Speed = motor_4_set_pwm = 
        -PID_calc(&motor_4_pid, Wings_Data.Wings_motor[3].Corrective_Angle, Wings_Data.Wings_motor[3].Target_Angle);

    // 将计算得到的PWM值设置到电机
    Set_Pwm(motor_1_set_pwm, motor_2_set_pwm, motor_3_set_pwm, motor_4_set_pwm);
}

/**
  * @brief  快速计算16位有符号整数绝对值（无分支优化）
  * @note   使用位运算避免分支预测，提高执行效率
  *         适用于嵌入式系统等对性能要求高的场景
  * @param  x: 输入的有符号16位整数
  * @retval 输入值的绝对值（无符号16位整数）
  */
static inline uint16_t abs16_fast(int16_t x) {
    int16_t m = x >> 15;            // 右移15位：x<0时得到-1（0xFFFF），x>=0时得到0
    return (uint16_t)((x ^ m) - m); // 异或后减去m，等价于取绝对值
}

/**
  * @brief  设置四个电机的PWM输出
  * @note   将PWM值转换为电机驱动器的控制信号
  *         每个电机有两个PWM输出引脚，分别控制正转和反转
  * @param  m1: 电机1的PWM值（有符号，正负表示方向）
  * @param  m2: 电机2的PWM值
  * @param  m3: 电机3的PWM值
  * @param  m4: 电机4的PWM值
  * @retval None
  */
void Set_Pwm(int16_t m1, int16_t m2, int16_t m3, int16_t m4)
{
    // -------- 电机1 PWM设置 --------
    uint16_t pwm1  = abs16_fast(m1);          // 获取电机1PWM的绝对值
    uint16_t mask1 = (uint16_t)-(m1 > 0);     // m1>0时mask1=0xFFFF，m1<=0时mask1=0
    PWM_M1_2 = (uint16_t)(pwm1 & mask1);      // 正转控制信号
    PWM_M1_1 = (uint16_t)(pwm1 & ~mask1);     // 反转控制信号

    // -------- 电机2 PWM设置 --------
    uint16_t pwm2  = abs16_fast(m2);
    uint16_t mask2 = (uint16_t)-(m2 > 0);
    PWM_M2_2 = (uint16_t)(pwm2 & mask2);
    PWM_M2_1 = (uint16_t)(pwm2 & ~mask2);

    // -------- 电机3 PWM设置 --------
    uint16_t pwm3  = abs16_fast(m3);
    uint16_t mask3 = (uint16_t)-(m3 > 0);
    PWM_M3_2 = (uint16_t)(pwm3 & mask3);
    PWM_M3_1 = (uint16_t)(pwm3 & ~mask3);

    // -------- 电机4 PWM设置 --------
    uint16_t pwm4  = abs16_fast(m4);
    uint16_t mask4 = (uint16_t)-(m4 > 0);
    PWM_M4_2 = (uint16_t)(pwm4 & mask4);
    PWM_M4_1 = (uint16_t)(pwm4 & ~mask4);
}

/**
  * @brief  计算有符号16位整数的绝对值（常规实现）
  * @note   使用条件判断的简单实现，可读性好但可能有分支预测开销
  * @param  a: 输入的有符号16位整数
  * @retval 输入值的绝对值（无符号16位整数）
  */
uint16_t myabs(int16_t a)
{
    if(a < 0) 
        return -a;  // 负数时返回相反数
    else 
        return a;   // 正数时直接返回
}

/**
  * @brief  电机编码器控制函数
  * @note   根据编码器反馈控制电机的函数
  *         当前为空实现，需要根据具体编码器类型和控制算法完善
  * @param  None
  * @retval None
  */
void Motor_ECD_Control(void)
{
    // 编码器控制逻辑
    // 通常包括：读取编码器值、计算实际速度、进行闭环控制等
    // 需要根据具体的编码器类型（如增量式、绝对值）和控制需求实现
}
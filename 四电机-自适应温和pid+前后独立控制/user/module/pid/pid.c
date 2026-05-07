#include "pid.h"
#include "main.h"

/**
  * @brief          数值限幅宏定义
  * @param[in]      input: 输入值
  * @param[in]      max: 限制的最大绝对值
  * @note           将输入值限制在[-max, max]范围内
  * @retval         none
  */
#define LimitMax(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

/*========================================
  PID结构体变量说明：
  - mode: PID工作模式(PID_POSITION/PID_DELTA)
  - Kp: 比例系数
  - Ki: 积分系数
  - Kd: 微分系数
  - max_out: PID总输出最大值
  - max_iout: PID积分输出最大值
  - Dbuf[3]: 微分项环形缓冲区，存储最近3次误差差值
  - error[3]: 误差环形缓冲区，存储最近3次误差
  - set: 目标设定值
  - fdb: 当前反馈值
  - Pout: 比例项输出
  - Iout: 积分项输出
  - Dout: 微分项输出
  - out: PID总输出
========================================*/

/**
  * @brief          PID控制器初始化函数
  * @param[out]     pid: PID控制结构体指针
  * @param[in]      mode: PID控制模式
  *                 PID_POSITION: 位置式PID（全量式）
  *                 PID_DELTA: 增量式PID
  * @param[in]      PID: PID参数数组，依次为[Kp, Ki, Kd]
  * @param[in]      max_out: PID总输出的最大绝对值
  * @param[in]      max_iout: 积分项输出的最大绝对值（抗积分饱和）
  * @retval         none
  * @note           1. 需先分配pid_type_def结构体内存
  *                 2. 调用此函数前确保PID参数数组有效
  *                 3. 初始化会清零所有历史数据和输出
  */
void PID_init(pid_type_def *pid, uint8_t mode, const fp32 PID[3], fp32 max_out, fp32 max_iout)
{
    /* 参数合法性检查 */
    if (pid == NULL || PID == NULL)
    {
        return;
    }
    
    /* 设置PID工作模式 */
    pid->mode = mode;
    
    /* 设置PID参数 */
    pid->Kp = PID[0];  // 比例系数
    pid->Ki = PID[1];  // 积分系数
    pid->Kd = PID[2];  // 微分系数
    
    /* 设置输出限制 */
    pid->max_out = max_out;   // 总输出限幅
    pid->max_iout = max_iout; // 积分输出限幅（抗积分饱和）
    
    /* 初始化微分缓冲区（存储最近3次误差差值）*/
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    
    /* 初始化误差缓冲区（存储最近3次误差）*/
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    
    /* 初始化各项输出 */
    pid->Pout = 0.0f;  // 比例项输出
    pid->Iout = 0.0f;  // 积分项输出
    pid->Dout = 0.0f;  // 微分项输出
    pid->out = 0.0f;   // PID总输出
    
    /* 初始目标值和反馈值 */
    pid->set = 0.0f;   // 目标设定值
    pid->fdb = 0.0f;   // 反馈值
}

/**
  * @brief          PID控制计算函数
  * @param[in/out]  pid: PID控制结构体指针
  * @param[in]      ref: 当前系统反馈值
  * @param[in]      set: 目标设定值
  * @retval         pid->out: 计算得到的PID输出值
  * @note           1. 需周期性调用（如定时中断中）
  *                 2. 位置式PID公式：u(k) = Kp*e(k) + Ki*∑e(k) + Kd*[e(k)-e(k-1)]
  *                 3. 增量式PID公式：Δu(k) = Kp*[e(k)-e(k-1)] + Ki*e(k) + Kd*[e(k)-2e(k-1)+e(k-2)]
  *                 4. 自动进行积分限幅和输出限幅
  */
fp32 PID_calc(pid_type_def *pid, fp32 ref, fp32 set)
{
    /* 参数合法性检查 */
    if (pid == NULL)
    {
        return 0.0f;
    }
    
    /*==============================
      数据缓冲区更新（滑动窗口）
    ==============================*/
    /* 更新误差历史：e(k-2) ← e(k-1), e(k-1) ← e(k) */
    pid->error[2] = pid->error[1];
    pid->error[1] = pid->error[0];
    
    /* 记录当前目标值和反馈值 */
    pid->set = set;   // 目标设定值
    pid->fdb = ref;   // 反馈值
    
    /* 计算当前误差：e(k) = 目标值 - 反馈值 */
    pid->error[0] = set - ref;
    
    /*==============================
      根据PID模式进行计算
    ==============================*/
    if (pid->mode == PID_POSITION)  /* 位置式PID（全量式）*/
    {
        /* 比例项：Pout = Kp * e(k) */
        pid->Pout = pid->Kp * pid->error[0];
        
        /* 积分项：Iout += Ki * e(k)（累加积分）*/
        pid->Iout += pid->Ki * pid->error[0];
        
        /* 微分项计算 */
        /* 更新微分历史：Dbuf(k-2) ← Dbuf(k-1), Dbuf(k-1) ← Dbuf(k) */
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        /* 计算当前微分差值：e(k) - e(k-1) */
        pid->Dbuf[0] = (pid->error[0] - pid->error[1]);
        /* 微分项：Dout = Kd * [e(k) - e(k-1)] */
        pid->Dout = pid->Kd * pid->Dbuf[0];
        
        /* 积分限幅（抗积分饱和）*/
        LimitMax(pid->Iout, pid->max_iout);
        
        /* 总输出 = 比例项 + 积分项 + 微分项 */
        pid->out = pid->Pout + pid->Iout + pid->Dout;
        
        /* 总输出限幅 */
        LimitMax(pid->out, pid->max_out);
    }
    else if (pid->mode == PID_DELTA)  /* 增量式PID */
    {
        /* 比例项：ΔP = Kp * [e(k) - e(k-1)] */
        pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
        
        /* 积分项：ΔI = Ki * e(k) */
        pid->Iout = pid->Ki * pid->error[0];
        
        /* 微分项计算 */
        /* 更新微分历史：Dbuf(k-2) ← Dbuf(k-1), Dbuf(k-1) ← Dbuf(k) */
        pid->Dbuf[2] = pid->Dbuf[1];
        pid->Dbuf[1] = pid->Dbuf[0];
        /* 计算二阶差分：e(k) - 2*e(k-1) + e(k-2) */
        pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
        /* 微分项：ΔD = Kd * [e(k) - 2*e(k-1) + e(k-2)] */
        pid->Dout = pid->Kd * pid->Dbuf[0];
        
        /* 增量式PID：总输出 = 上次输出 + 本次增量（ΔP + ΔI + ΔD）*/
        pid->out += pid->Pout + pid->Iout + pid->Dout;
        
        /* 总输出限幅 */
        LimitMax(pid->out, pid->max_out);
    }
    
    /* 返回PID输出值 */
    return pid->out;
}

/**
  * @brief          PID控制器清除函数
  * @param[in/out]  pid: PID控制结构体指针
  * @retval         none
  * @note           1. 在系统启动、模式切换或需要重置PID时调用
  *                 2. 清除所有历史数据，避免累积误差影响
  *                 3. 不会改变PID参数，只清除状态数据
  */
void PID_clear(pid_type_def *pid)
{
    /* 参数合法性检查 */
    if (pid == NULL)
    {
        return;
    }
    
    /* 清除误差历史缓冲区 */
    pid->error[0] = pid->error[1] = pid->error[2] = 0.0f;
    
    /* 清除微分历史缓冲区 */
    pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
    
    /* 清除所有输出项 */
    pid->out = 0.0f;   // 总输出
    pid->Pout = 0.0f;  // 比例输出
    pid->Iout = 0.0f;  // 积分输出
    pid->Dout = 0.0f;  // 微分输出
    
    /* 清除目标值和反馈值记录 */
    pid->fdb = 0.0f;   // 反馈值
    pid->set = 0.0f;   // 设定值
}

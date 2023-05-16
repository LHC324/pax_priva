#include "lhc_tool.h"

const char *text_name[co_type_max + 1U] = {
    [co_string] = "string",
    [co_bool] = "bool",
    [co_uint8] = "uint8_t",
    [co_int8] = "int8_t",
    [co_uint16] = "uint16_t",
    [co_int16] = "int16_t",
    [co_uint32] = "uint32_t",
    [co_int32] = "int32_t",
    [co_float] = "float",
    [co_long] = "long",
    [co_ulong] = "ulong",
    [co_type_max] = "null",
};

#if (TOOL_USING_STM32HAL)
/**
 * @brief  串口接收数据：DMA+空闲中断接收不定长数据
 * @param  pu 串口对象句柄
 * @retval None
 */
void uartx_recive_handle(pUartHandle pu)
{
    /*Gets the idle flag so that the idle flag is set*/
    if ((__HAL_UART_GET_FLAG((UART_HandleTypeDef *)pu->huart, UART_FLAG_IDLE) != RESET))
    {
        /*Clear idle interrupt flag*/
        __HAL_UART_CLEAR_IDLEFLAG((UART_HandleTypeDef *)pu->huart);
        if (pu && (pu->rx.pbuf) && (pu->phdma) && (pu->rx.size))
        {
            /*Stop DMA transmission to prevent busy receiving data and interference during data processing*/
            HAL_UART_DMAStop((UART_HandleTypeDef *)pu->huart);
            /*Get the number of untransmitted data in DMA*/
            /*Number received = buffersize - the number of data units remaining in the current DMA channel transmission */
            pu->rx.count = pu->rx.size - __HAL_DMA_GET_COUNTER((DMA_HandleTypeDef *)pu->phdma);
            /*Reopen DMA reception*/
            HAL_UART_Receive_DMA((UART_HandleTypeDef *)pu->huart, pu->rx.pbuf, pu->rx.size);
        }
#if (TOOL_USING_RTOS)
        /*After opening the serial port interrupt, the semaphore has not been created*/
        if (pu->semaphore)
        {
            /*Notification task processing*/
            release_semaphore(pu->semaphore);
        }
#else
        pu->recive_finish_flag = true;
#endif
    }
}
#endif

#if (TOOL_USING_CRC16)
/**
 * @brief  取得16bitCRC校验码
 * @param  ptr   当前数据串指针
 * @param  length  数据长度
 * @param  init_dat 校验所用的初始数据
 * @retval 16bit校验码
 */
uint16_t get_crc16(uint8_t *ptr, uint16_t length, uint16_t init_dat)
{
    uint16_t crc16 = init_dat;

    if (ptr == NULL)
        return 0;
    for (uint16_t i = 0; i < length; i++)
    {
        crc16 ^= *ptr++;

        for (uint16_t j = 0; j < 8; j++)
        {
            crc16 = (crc16 & 0x0001) ? ((crc16 >> 1) ^ 0xa001) : (crc16 >> 1U);
        }
    }
    return (crc16);
}
#endif

#if (TOOOL_USING_ENDIAN)
/**
 * @brief  大小端数据类型交换
 * @note   对于一个单精度浮点数的交换仅仅需要2次
 * @param  pdata 数据
 * @param  start 开始位置
 * @param  length  数据长度
 * @retval None
 */
void endian_swap(uint8_t *pdata, uint8_t start, uint8_t length)
{
    uint8_t i = 0;
    uint8_t tmp = 0;
    uint8_t count = length / 2U;
    uint8_t end = start + length - 1U;

    for (; pdata && i < count; i++)
    {
        tmp = pdata[start + i];
        pdata[start + i] = pdata[end - i];
        pdata[end - i] = tmp;
    }
}
#endif

#if (TOOOL_USING_SITE_PID)
void init_site_pid(site_pid_t *pid,
                   float kp,
                   float ki,
                   float kd)
{
    if (pid == NULL)
        return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->last_ek = 0;
    pid->sum_ek = 0;
}

/**
 * @brief	位置式pid初始化
 * @details 参看：https://blog.csdn.net/yunddun/article/details/107720644
 * @param	None
 * @retval  None
 */
float get_site_pid_out(site_pid_t *pid, float cur_val, float tar_val)
{
#define SITE_PID_ERROR_MIN 1E-10
    if (pid == NULL)
        return 0;
    float cur_ek = tar_val - cur_val;
    float delta_ek = cur_ek - pid->last_ek; // Δ
    float p_out = pid->kp * cur_ek;
    float i_out = pid->ki * pid->sum_ek;
    float d_out = pid->kd * delta_ek;

    pid->sum_ek += cur_ek;
    pid->last_ek = cur_ek; // 更新偏差
    /*积分项抗饱和:https://zhuanlan.zhihu.com/p/226304120*/
    if (fabsf(cur_ek) < SITE_PID_ERROR_MIN)
        pid->sum_ek = 0;

    return (p_out + i_out + d_out);
#undef SITE_PID_ERROR_MIN
}
#endif

#if (TOOOL_USING_INCREMENT_PID)
void init_increment_pid(incremental_pid_t *pid,
                        float kp,
                        float ki,
                        float kd)
{
    if (pid == NULL)
        return;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->last_ek = 0;
    pid->last_last_ek = 0;
}
/**
 * @brief	增量式pid输出
 * @details
 * @param	None
 * @retval  None
 */
float get_incremental_pid_out(incremental_pid_t *pid,
                              float cur_val,
                              float tar_val)
{
    if (pid == NULL)
        return 0;
    float cur_ek = tar_val - cur_val;     // 本次误差
    float error1 = cur_ek - pid->last_ek; // 本次偏差与上次偏差之差
    float error2 = cur_ek - 2.0F * pid->last_ek + pid->last_last_ek;

    float p_out = pid->kp * error1;
    float i_out = pid->ki * cur_ek;
    float d_out = pid->kd * error2;

    pid->last_last_ek = pid->last_ek; // 更新偏差
    pid->last_ek = cur_ek;

    return (p_out + i_out + d_out);
}
#endif

#if (TOOOL_USING_LINUX_STAMP == 1)
/*
 *  Beijing time to linux timestamp
 */
unsigned int std_time_to_linux_stamp(rtc_t *prtc)
{
    if (prtc == NULL)
        return 0;

    unsigned short monthdays[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};
    unsigned short year = prtc->date.year + 2000;
    unsigned int linux_stamp = (year - 1970) * 365 * 24 * 3600 +
                               (monthdays[prtc->date.month - 1] +
                                prtc->date.date - 1) *
                                   24 * 3600 +
                               (prtc->time.hours - 8) * 3600 + prtc->time.minutes * 60 + prtc->time.seconds;

    linux_stamp += (prtc->date.month > 2 && (year % 4 == 0) &&
                    (year % 100 != 0 || year % 400 == 0)) *
                   24 * 3600; // 闰月

    year -= 1970;
    linux_stamp += (year / 4 - year / 100 + year / 400) * 24 * 3600; // 闰年

    return linux_stamp;
}

/*
 * Judge leap year and average year
 */
static unsigned char is_leap_year(unsigned short year)
{
    return ((((year) % 4 == 0 &&
              (year) % 100 != 0) ||
             (year) % 400 == 0));
}

/*
 *  Function function: get the week according to the specific date
 */
void get_weekday(rtc_t *prtc)
{
    unsigned short year = 0;
    unsigned char month = 0;

    // if (prtc->date.month == 1 || prtc->date.month == 2)
    if (prtc->date.month < 3)
    {
        year = prtc->date.year - 1 + 2000;
        month = prtc->date.month + 12;
    }
    else
    {
        year = prtc->date.year + 2000;
        month = prtc->date.month;
    }

    prtc->date.weelday = ((prtc->date.date + 2 * month + 3 * (month + 1) / 5 + year + year / 4 - year / 100 + year / 400) % 7) + 1;
}

/*
 *    Linux Time Stamp to std time
 */
void linux_stamp_to_std_time(rtc_t *prtc, unsigned int cur_stamp, int time_zone)
{
    unsigned short year = 1970;
    unsigned int counter = 0, count_temp; // 随着年份迭加，Counter记录￿??1970 ￿?? 1 ￿?? 1 日（00:00:00 GMT）到累加到的年份的最后一天的秒数
    unsigned char month[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

    cur_stamp += time_zone * 3600; // 修正时区

    while (counter <= cur_stamp) // 假设今天￿??2018年某￿??天，则时间戳应小于等￿??1970-1-1 0:0:0 ￿?? 2018-12-31 23:59:59的�?�秒￿??
    {
        count_temp = counter; // CounterTemp记录完全1970-1-1 0:0:0 ￿?? 2017-12-31 23:59:59的�?�秒数后￿??出循￿??
        counter += 31536000;  // 加上今年（平年）的秒￿??

        if (is_leap_year(year))
        {
            counter += 86400; // 闰年多加￿??￿??
        }
        year++;
    }

    prtc->date.year = year - 1 - 2000; // 跳出循环即表示到达计数�?�当前年
    month[1] = (is_leap_year(year - 1) ? 29 : 28);
    counter = cur_stamp - count_temp; // counter = cur_stamp - count_temp  记录2018年已走的总秒￿??
    count_temp = counter / 86400;     // count_temp = counter/(24*3600)  记录2018年已【过去�?�天￿??
    counter -= count_temp * 86400;    // 记录今天已走的�?�秒￿??
    prtc->time.hours = counter / 3600;
    prtc->time.minutes = counter % 3600 / 60;
    prtc->time.seconds = counter % 60;

    for (unsigned char i = 0; i < 12; i++)
    {
        if (count_temp < month[i]) // 不能包含相等的情况，相等会导致最后一天切换到下一个月第一天时
        {
            // （即CounterTemp已走天数刚好为n个月完整天数相加时（31+28+31...））￿??
            prtc->date.month = i + 1;         // 月份不加1，日期溢出（如：出现32号）
            prtc->date.date = count_temp + 1; // 应不作处理，CounterTemp = month[i] = 31时，会继续循环，月份加一￿??
            break;                            // 日期变为1，刚好符合实际日￿??
        }

        count_temp -= month[i];
    }

    get_weekday(prtc);
}
#endif

#if (TOOL_USING_KALMAN)
#define EPS 1e-5f
/**
 *卡尔曼滤波器
 *@param KFP *kfp 卡尔曼结构体参数
 *   float input 需要滤波的参数的测量值（即传感器的采集值）
 *@return 滤波后的参数（最优值）
 */
float kalmanFilter(KFP *kfp, float input)
{
    /*预测协方差方程：k时刻系统估算协方差 = k-1时刻的系统协方差 + 过程噪声协方差*/
    kfp->Now_Covariance = kfp->Last_Covariance + kfp->Q;
    /*卡尔曼增益方程：卡尔曼增益 = k时刻系统估算协方差 / （k时刻系统估算协方差 + 观测噪声协方差）*/
    kfp->Kg = kfp->Now_Covariance / (kfp->Now_Covariance + kfp->R);
    /*更新最优值方程：k时刻状态变量的最优值 = 状态变量的预测值 + 卡尔曼增益 * （测量值 - 状态变量的预测值）*/
    kfp->Output = kfp->Output + kfp->Kg * (input - kfp->Output); // 因为这一次的预测值就是上一次的输出值
    /*更新协方差方程: 本次的系统协方差付给 kfp->Last_Covariance 为下一次运算准备。*/
    kfp->Last_Covariance = (1 - kfp->Kg) * kfp->Now_Covariance;
    /*当kfp->Output不等于0时，负方向迭代导致发散到无穷小*/
    if (kfp->Output - 0.0f < EPS)
    {
        kfp->Kg = 0;
        kfp->Output = 0;
    }
    return kfp->Output;
#undef EPS
}
#else
/**
 *滑动滤波器
 *@param SideParm *side 滑动结构体参数
 *   float input 需要滤波的参数的测量值（即传感器的采集值）
 *@return 滤波后的参数（最优值）
 */
float sidefilter(SideParm *side, float input)
{
    // 第一次滤波
    if (side->First_Flag)
    {

        for (int i = 0; i < sizeof(side->SideBuff) / sizeof(float); i++)
        {
            side->SideBuff[i] = input;
        }

        side->First_Flag = false;
        side->Head = &side->SideBuff[0];
        side->Sum = input * (sizeof(side->SideBuff) / sizeof(float));
    }
    else
    {
        side->Sum = side->Sum - *side->Head + input;
        *side->Head = input;

        if (++side->Head > &side->SideBuff[sizeof(side->SideBuff) / sizeof(float) - 1])
        {
            side->Head = &side->SideBuff[0];
        }

        input = side->Sum / (1 << FILTER_SHIFT);
    }

    return input;
}
#endif

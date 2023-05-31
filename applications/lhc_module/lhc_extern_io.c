/*
 * Copyright (c) 2006-2023, Lhc Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-04-28     LHC       the first version
 */
#include "lhc_extern_io.h"
#include "main.h"
#include "lhc_modbus_port.h"
// #include <rtthread.h>
#if defined(LHC_MODBUS_USING_INPUT_REGISTER) || defined(LHC_MODBUS_USING_HOLD_REGISTER)
// #include "Mcp4822.h"
#endif

#ifdef DBG_TAG
#undef DBG_TAG
#define DBG_TAG "extern.io"
#endif
#define DBG_LVL DBG_LOG

#define CURRENT_CP 0.005378F
#define CURRENT_CQ 0.375224F
#define VOLTAGE_CP 0.005378F
#define VOLTAGE_CQ 0.375224F
#define Get_Target_Value(__Value, __CP, __CQ) \
    ((__Value) ? ((__CP) * (float)(__Value) + (__CQ)) : 0)

static Dac_HandleTypeDef dac_map[] = {
    {.Channel = DAC_OUT1, .Mcpxx_Id = Input_Other},
    {.Channel = DAC_OUT2, .Mcpxx_Id = Input_Other},
};
static float dac_param[][2] = {
    {201.8393825F, -6.724913779F},
    {201.3104013F, -7.379197379F},
};

/*dac校准互斥标志*/
static bool set_dac_flag = false;

static void output_current(Dac_HandleTypeDef *ch, float data);
/*通过DMA循环采集的ADC数据缓存区*/
unsigned int adc_buffer[ADC_DMA_SIZE] = {0};


/**
 * @brief  Get ADC channel value of DMA transmission
 * @note   Channel 1 and Channel 2 of ADC1 are used
 * @param  channel Channel ID
 * @retval ADC average value
 */
static unsigned int get_adc_value(const unsigned int channel)
{
    unsigned int sum = 0;
    unsigned int *padc = &adc_buffer[0];

    /*Prevent array out of bounds*/
    if (channel > ADC_DMA_CHANNEL)
    {
        return 0;
    }
    /*Add channel offset*/
    padc += channel;

    for (; padc < adc_buffer + ADC_DMA_SIZE; padc += ADC_DMA_CHANNEL)
    {
        sum += *padc;
    }

    return (sum >> ADC_FILTER_SHIFT);
}


#if defined(LHC_MODBUS_USING_INPUT_COIL)

#if (!TOOL_USING_AC_DI)

static Gpiox_info di_gpio[] = {
    {DI0_GPIO_Port, DI0_Pin},
    {DI1_GPIO_Port, DI1_Pin},
    {DI2_GPIO_Port, DI2_Pin},
    {DI3_GPIO_Port, DI3_Pin},
    {DI4_GPIO_Port, DI4_Pin},
    {DI5_GPIO_Port, DI5_Pin},
};

#else
/*检测输入引脚延时*/
#define EXTERN_INIO_CHECK_DELAY_XMS 5U
static void di_timeout_check(di_input *pi);
di_input extern_io_dix;

/**
 * @brief This function is called to increment  a global variable "uwTick"
 *        used as application time base.
 * @note In the default implementation, this variable is incremented each 1ms
 *       in SysTick ISR.
 * @note This function is declared as __weak to be overwritten in case of other
 *      implementations in user file.
 * @retval None
 */
// void HAL_IncTick(void)
// {
//     // extern __IO uint32_t uwTick;
//     uwTick += uwTickFreq;
//     di_timeout_check(&extern_io_dix);
// }

static void di_collect(void)
{
    di_timeout_check(&extern_io_dix);
}

di_input_group di_gpio[] = {
    {   
        "PE.1",
        {DI0_GPIO_Port, DI0_Pin},
        0,
    },
    {
        "PE.2",
        {DI1_GPIO_Port, DI1_Pin},
        0,
    },
    {
        "PE.3",
        {DI2_GPIO_Port, DI2_Pin},
        0,
    },
    {
        "PE.4",
        {DI3_GPIO_Port, DI3_Pin},
        0,
    },
    {
        "PE.5",
        {DI4_GPIO_Port, DI4_Pin},
        0,
    },
    {
        "PE.6",
        {DI5_GPIO_Port, DI5_Pin},
        0,
    },
};

di_input extern_io_dix = {
    .group = di_gpio,
    .group_size = sizeof(di_gpio) / sizeof(di_input_group),
    .data = NULL,
};

/**
 * @brief  Timer timeout detection.
 * @param  None
 * @retval None
 */
static void di_timeout_check(di_input *pi)
{
    uint8_t offset = 0;

    RT_ASSERT(pi);

    for (di_input_group *p = pi->group;
         pi && pi->group && p < pi->group + pi->group_size; ++p, ++offset)
    {
        if (p->count)
        {
            p->count--;
        }
        else
        {
            p->count = EXTERN_INIO_CHECK_DELAY_XMS;

            /*读取外部数字引脚状态:翻转光耦的输入信号*/
            if (p->gpio.port &&
                !HAL_GPIO_ReadPin(p->gpio.port, p->gpio.pin))
            {
                pi->bits |= 1U << offset;
            }
        }
    }
}

/**
 * @brief  EXTI line detection callbacks.
 * @param  GPIO_Pin: Specifies the pins connected EXTI line
 * @retval None
 */
// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
// {
//     di_input *pi = &extern_io_dix;

//     for (di_input_group *p = pi->group;
//          pi->group && p < pi->group + pi->group_size; ++p)
//     {
//         if (p->gpio.pin == GPIO_Pin)
//         {
//             p->count = EXTERN_INIO_CHECK_DELAY_XMS;
//         }
//     }
// }

/**
 * @brief   di引脚外部中断刷新
 * @details 被HAL_GPIO_EXTI_Callback调用
 * @param   GPIO_Pin 引脚编号
 * @retval  None
 */
// void di_pin_refresh(uint16_t GPIO_Pin)
// {
//     di_input *pi = &extern_io_dix;

//     for (di_input_group *p = pi->group;
//          pi->group && p < pi->group + pi->group_size; ++p)
//     {
//         if (p->gpio.pin == GPIO_Pin)
//         {
//             p->count = EXTERN_INIO_CHECK_DELAY_XMS;
//         }
//     }
// }

void di_pin_refresh(void *args)
{
    uint16_t GPIO_Pin = *(uint16_t *)args;
    di_input *pi = &extern_io_dix;

    for (di_input_group *p = pi->group;
         pi->group && p < pi->group + pi->group_size; ++p)
    {
        if (p->gpio.pin == GPIO_Pin)
        {
            p->count = EXTERN_INIO_CHECK_DELAY_XMS;
        }
    }
}

#include <rtdevice.h>

/**
 * @brief   为每个di引脚挂载外部中断
 * @details 被HAL_GPIO_EXTI_Callback调用(不破坏rtt gpio_drv源码)
 * @param   None
 * @retval  None
 */
static void extern_io_init_di(void)
{
    rt_int32_t pin_num;

    /* 绑定中断，下降沿模式，回调函数名为di_pin_refresh */
    for (di_input_group *pi = di_gpio;
         pi < di_gpio + sizeof(di_gpio) / sizeof(di_gpio[0]);
         ++pi)
    {
        pin_num = rt_pin_get(pi->gpio_name);
        rt_pin_attach_irq(pin_num, PIN_IRQ_MODE_FALLING, di_pin_refresh, &pi->gpio.pin);
        /* 使能中断 */
        rt_pin_irq_enable(pin_num, PIN_IRQ_ENABLE);
    }
}

#endif

#ifdef EXTERN_IO_DI_MAX
#undef EXTERN_IO_DI_MAX
#define EXTERN_IO_DI_MAX (sizeof(di_gpio) / sizeof(di_gpio[0]))
#endif

/**
 * @brief   外部数字量输入处理
 * @details 
 * @param   None
 * @retval  None
 */
int extern_io_read_di(void *modbus,
                      unsigned short addr,
                      unsigned short num,
                      int flag)
{
    RT_ASSERT(modbus);
    RT_ASSERT(num <= EXTERN_IO_DI_MAX);

    modbus_t pd = (modbus_t)modbus;
    unsigned char wbits[EXTERN_IO_DI_MAX], *bit = wbits;

    lhc_tool_memset(wbits, 0x00, sizeof(wbits));

#if (!TOOL_USING_AC_DI)
    Gpiox_info *pi = di_gpio;
    for (; pi < di_gpio + sizeof(di_gpio) / sizeof(Gpiox_info),
           bit < wbits + sizeof(wbits) / sizeof(wbits[0]);
         ++pi, ++bit)
    {
        /*读取外部数字引脚状态:翻转光耦的输入信号*/
        if (pi->port)
        {
            *bit = (unsigned char)HAL_GPIO_ReadPin((GPIO_TypeDef *)pi->port, pi->pin);

            if (flag & ex_io_di_roll)
                *bit = !*bit;
        }
        else
            return -1;
    }
#else
    di_input *pi = &extern_io_dix;
    for (uint8_t i = 0; i < EXTERN_IO_DI_MAX; ++i)
    {
        wbits[i] = (pi->bits >> i) & 0x01;
    }
    pi->bits = 0; // 清空本次结果
#endif

    return (pd->Mod_Operatex(pd, InputCoil, lhc_modbus_write, addr, wbits, num) ? 0 : -1);
}
#endif

#if defined(LHC_MODBUS_USING_COIL)
static Gpiox_info do_gpio[] = {
    {Q0_GPIO_Port, Q0_Pin},
    {Q1_GPIO_Port, Q1_Pin},
    {Q2_GPIO_Port, Q2_Pin},
    {Q3_GPIO_Port, Q3_Pin},
    {Q4_GPIO_Port, Q4_Pin},
    {Q5_GPIO_Port, Q5_Pin},
    {Q6_GPIO_Port, Q6_Pin},
    {Q7_GPIO_Port, Q7_Pin},
    {Q8_GPIO_Port, Q8_Pin},
    {Q9_GPIO_Port, Q9_Pin},
    {Q10_GPIO_Port, Q10_Pin},
    {Q11_GPIO_Port, Q11_Pin},
    {Q12_GPIO_Port, Q12_Pin},
    {Q13_GPIO_Port, Q13_Pin},
    {Q14_GPIO_Port, Q14_Pin},
    {Q15_GPIO_Port, Q15_Pin},
    {Q16_GPIO_Port, Q16_Pin},
    {Q17_GPIO_Port, Q17_Pin},
    {Q18_GPIO_Port, Q18_Pin},
    {Q19_GPIO_Port, Q19_Pin},
    {Q20_GPIO_Port, Q20_Pin},
    {Q21_GPIO_Port, Q21_Pin},
    {Q22_GPIO_Port, Q22_Pin},
};

/**
 * @brief   数字量输出
 * @details STM32F030F4共在io口扩展了26路数字输出
 * @param   None
 * @retval  None
 */
int extern_io_write_do(void *modbus,
                       unsigned short addr,
                       unsigned short num,
                       int flag)
{
#ifdef EXTERN_IO_DO_MAX
#undef EXTERN_IO_DO_MAX
#define EXTERN_IO_DO_MAX (sizeof(do_gpio) / sizeof(do_gpio[0]))
#endif
    RT_ASSERT(modbus);
    RT_ASSERT(num  <= EXTERN_IO_DO_MAX);

    modbus_t pd = (modbus_t)modbus;
    unsigned char rbits[EXTERN_IO_DO_MAX], *bit = rbits;
    Gpiox_info *po = do_gpio;

    /*读取对应寄存器*/
    if (!pd->Mod_Operatex(pd, Coil, lhc_modbus_read, addr, rbits, num))                
    {
#if (LHC_MODBUS_USING_DEBUG)
        LHC_MODBUS_DEBUG_D("@error:Coil reading failed!\r\n");
        return -1;
#endif
    }

    for (; po < do_gpio + sizeof(do_gpio) / sizeof(Gpiox_info),
           bit < rbits + sizeof(rbits) / sizeof(rbits[0]);
         ++po, ++bit)
    {
        if(po->port && (!(*bit & INDIRC_COIL))) //带直接启动标识的才允许直接控制
            HAL_GPIO_WritePin((GPIO_TypeDef *)po->port, po->pin, (GPIO_PinState)*bit);
        else
            return -1;
    }

    return 0;
}
#endif

#if defined(LHC_MODBUS_USING_INPUT_REGISTER)

    /*滤波结构需要不断迭代，否则滤波器无法正常工作*/
#if defined(TOOL_USING_KALMAN)
    KFP hkfp = {
        .Last_Covariance = LASTP,
        .Kg = 0,
        .Now_Covariance = 0,
        .Output = 0,
        .Q = COVAR_Q,
        .R = COVAR_R,
    };
    static KFP pkfp[EXTERN_IO_AI_MAX];
#else
    SideParm side = {
        .First_Flag = true,
        .Head = &side.SideBuff[0],
        .SideBuff = {0},
        .Sum = 0,
    };
    static SideParm pside[EXTERN_IO_AI_MAX];
#endif

static bool first_flag = false;

/**
 * @brief   外部模拟量输入处理
 * @details 
 * @param   None
 * @retval  ret
 */
int extern_io_read_ai(void *modbus,
                      unsigned short addr,
                      unsigned short num,
                      int flag)
{
    RT_ASSERT(modbus);
    RT_ASSERT(num  <= EXTERN_IO_AI_MAX);

    modbus_t pd = (modbus_t)modbus;
    int ret = 0;
    float data[EXTERN_IO_AI_MAX * sizeof(float)];
    unsigned short ch = 0;

    lhc_tool_memset(data, 0x00, EXTERN_IO_AI_MAX * sizeof(float));

    /*保证仅首次copy*/
    if (!first_flag)
    {
        first_flag = true;
        for (ch = 0; ch < num; ch++)
        {
#if defined(TOOL_USING_KALMAN)
            lhc_tool_memcpy(&pkfp[ch], &hkfp, sizeof(hkfp));
#else
            lhc_tool_memcpy(&pside[ch], &side, sizeof(pside));
#endif
        }
    }

    for (ch = 0; ch < num; ch++)
    {
        /*获取DAC值*/
        uint32_t adc_value = get_adc_value(ch);
        /*前8路电流，后8路电压*/
        data[ch] = ch < (EXTERN_IO_AI_MAX / 2U)
                        ? Get_Target_Value(adc_value, CURRENT_CP, CURRENT_CQ)
                        : Get_Target_Value(adc_value, VOLTAGE_CP, VOLTAGE_CQ);
#if (LHC_MODBUS_USING_DEBUG)
        //        LHC_DEBUG_D("ADC[%d] = %d.", ch, adc_value);
#endif

        /*滤波处理*/
#if defined(TOOL_USING_KALMAN)
        data[ch] = kalmanFilter(&pkfp[ch], data[ch]);
#else
        data[ch] = sidefilter(&pside[ch], data[ch]);
#endif
    }

    /*写入对应寄存器*/
    if (!pd->Mod_Operatex(pd, InputRegister, lhc_modbus_write, addr,
                          (uint8_t *)data, num * sizeof(float)))
    {
#if defined(LHC_MODBUS_USING_DEBUG)
        LHC_MODBUS_DEBUG_D("@error:Input register write failed!\r\n");
#endif
        ret = -1;
    }

    return ret;
}

#endif

/**
 * @brief	对目标通道输出电流
 * @details
 * @param   ch :目标通道
 * @param	data:写入的数据
 * @retval	None
 */
static void output_current(Dac_HandleTypeDef *ch, float data)
{
    extern DAC_HandleTypeDef hdac;
    void *padc = &hdac;
    uint16_t value = (uint16_t)(dac_param[ch->Channel][0] * data +
                                dac_param[ch->Channel][1]);

    value = data ? ((data > 20.0F) ? 0x0FFF : value) : 0U;
    uint32_t channel = ch->Channel ? DAC_CHANNEL_2 : DAC_CHANNEL_1;

    HAL_DAC_SetValue((DAC_HandleTypeDef *)padc, channel,
                     DAC_ALIGN_12B_R, (value & 0x0FFF));
}

/**
 * @brief   模拟量输出
 * @details STM32F030F4共在io口扩展了8路数字输出
 * @param   None
 * @retval  None
 */
#if defined(LHC_MODBUS_USING_HOLD_REGISTER)
int extern_io_write_ao(void *modbus,
                       unsigned short addr,
                       unsigned short num,
                       int flag)
{
    RT_ASSERT(modbus);
    RT_ASSERT(num  <= EXTERN_IO_AO_MAX);

    int ret = 0;
    float data[EXTERN_IO_AO_MAX * sizeof(float)];
    modbus_t pd = (modbus_t)modbus;

    lhc_tool_memset(data, 0x00, EXTERN_IO_AO_MAX * sizeof(float));

        /*读出保持寄存器*/
    if (!pd->Mod_Operatex(pd, HoldRegister, lhc_modbus_read, addr,
                          (uint8_t *)&data, num * sizeof(float)))
    {
#if (LHC_MODBUS_USING_DEBUG)
        LHC_MODBUS_DEBUG_D("@error:Hold register read failed!\r\n");
#endif
        ret = -1;
    }

    for (uint16_t ch = 0; ch < num; ch++)
    {
#if defined(LHC_MODBUS_USING_DEBUG)
        // shellPrint(Shell_Object, "W_AD[%d] = %.3f\r\n", ch, pdata[ch]);
#endif
        if (!set_dac_flag)
            output_current(&dac_map[ch], data[ch]);
    }

    return ret;
}
#endif

/**
 * @brief	rt_thread 软件定时器回调函数
 * @details
 * @param	parameter:线程初始参数
 * @retval  None
 */
static void timer_callback(void *parameter)
{
    UNUSED(parameter);
    di_collect();
}

/**
 * @brief	控制线程定时动作
 * @details
 * @param	parameter:线程初始参数
 * @retval  None
 */
void extern_io_thread_entry(void *parameter)
{
    extern_io_init_di();

    for (;;)
    {
        extern_io_read_di(share_lhc_modbus, 0, EXTERN_IO_DI_MAX, ex_io_di_roll);
        extern_io_write_do(share_lhc_modbus, 0, EXTERN_IO_DO_MAX, 0);
        extern_io_read_ai(share_lhc_modbus, 0, EXTERN_IO_AI_MAX, 0);
        extern_io_write_ao(share_lhc_modbus, 0, EXTERN_IO_AO_MAX, 0);

        rt_thread_mdelay(100);
    }
}

/**
 * @brief	外部io初始化
 * @details
 * @param	None
 * @retval  none
 */
static int extern_io_init(void)
{
    rt_timer_t timer = RT_NULL;
    /* 创建定时器1  周期定时器 */
    timer = rt_timer_create(
        "di_timer",
        timer_callback,
        RT_NULL,
        1,
        RT_TIMER_FLAG_PERIODIC);
    /* 启动timer定时器 */
    if (timer != RT_NULL)
        rt_timer_start(timer);

    rt_thread_t tid = rt_thread_create(
        "extern_io",
        extern_io_thread_entry,
        RT_NULL,
        2048, 0x0F, 20);

    RT_ASSERT(tid != RT_NULL);

    rt_thread_startup(tid);

    return (RT_EOK);
}
INIT_ENV_EXPORT(extern_io_init);

#ifdef RT_USING_FINSH
#include <finsh.h>

/**
 * @brief   查看数字输入状态
 * @details
 * @param   none
 * @retval  none
 */
static void see_di(void)
{
    modbus_t pd = share_lhc_modbus;
    uint8_t rbits[EXTERN_IO_DI_MAX];
    lhc_tool_memset(rbits, 0x00, sizeof(rbits));

    if (pd)
    {
        /*写入对应寄存器*/
        if (!pd->Mod_Operatex(pd, InputCoil, lhc_modbus_read, SIGNAL_IO_DI_START_ADDR,
                              rbits, EXTERN_IO_DI_MAX))
        {
#if (LHC_MODBUS_USING_DEBUG)
            LHC_MODBUS_DEBUG_D("@error:Input coil write failed.");
#endif
            return;
        }
        LHC_MODBUS_DEBUG_R("\ndigital output:\nsn\tpinx\n");
        LHC_MODBUS_DEBUG_R("--\t----\n");
        for (uint16_t i = 0; i < EXTERN_IO_DI_MAX; i++)
            LHC_MODBUS_DEBUG_R("%d\t%d\n", i, rbits[i]);
    }
}
MSH_CMD_EXPORT(see_di, display digital output value.);

/**
 * @brief   查看数字输出状态
 * @details
 * @param   none
 * @retval  none
 */
static void see_do(void)
{
    modbus_t pd = share_lhc_modbus;
    uint8_t rbits[EXTERN_IO_DO_MAX];
    lhc_tool_memset(rbits, 0x00, sizeof(rbits));
    if (pd)
    {
        if (!pd->Mod_Operatex(pd, Coil, lhc_modbus_read, SIGNAL_IO_DO_START_ADDR,
                              rbits, EXTERN_IO_DO_MAX))
        {
#if (LHC_MODBUS_USING_DEBUG)
            LHC_MODBUS_DEBUG_D("@error:coil read failed.");
#endif
        }
        LHC_MODBUS_DEBUG_R("\ndigital output:\nsn\tpinx\n");
        LHC_MODBUS_DEBUG_R("---\t----\n");
        for (uint16_t i = 0; i < EXTERN_IO_DO_MAX; ++i)
            LHC_MODBUS_DEBUG_R("%d\t%d\n", i, rbits[i]);
        // LOG_HEX("see do", 8, rbits, EXTERN_IO_DO_MAX);
    }
}
MSH_CMD_EXPORT(see_do, display digital output value.);

/**
 * @brief   设置数字输出状态
 * @details
 * @param   none
 * @retval  none
 */
static void set_do(int argc, char **argv)
{
    modbus_t pd = share_lhc_modbus;

    if (argc < 2)
    {
        LHC_MODBUS_DEBUG_R("@error: Please input'set_do <(0~maxpin) | (0/1)>.'\n");
        return;
    }
    if (argc > 3)
    {
        LHC_MODBUS_DEBUG_R("@error:parameter is too long,please input'set_do <(0~9) | (0/1)>.'\n");
        return;
    }

    uint8_t pin = (uint8_t)atoi(argv[1]);
    uint8_t state = (uint8_t)atoi(argv[2]);

    if (pin >= EXTERN_IO_DO_MAX)
    {
        LHC_MODBUS_DEBUG_D("@error:parameter[1]%d error,please input'0~%d'.\n", pin, EXTERN_IO_DO_MAX);
        return;
    }
    if (state > 1U)
    {
        LHC_MODBUS_DEBUG_D("@error:parameter[2]:%d error,please input'0/1'.\n", state);
        return;
    }

    if (pd)
    {
        uint8_t start_addr = SIGNAL_IO_DO_START_ADDR + pin;
        /*写入对应寄存器*/
        if (!pd->Mod_Operatex(pd, Coil, lhc_modbus_write, start_addr, &state, 1U))
        {
#if (LHC_MODBUS_USING_DEBUG)
            LHC_MODBUS_DEBUG_D("@error:coil write failed.");
#endif
        }
    }
    see_do();
}
MSH_CMD_EXPORT(set_do, set_do sample
               : set_do<(0 ~pinmax) | (0 / 1)>);

/**
 * @brief   查看adc通道值
 * @details
 * @param   none
 * @retval  none
 */
static void see_adc(void)
{
    LHC_MODBUS_DEBUG_R("\nadc:\nchannel\tvalue\n");
    LHC_MODBUS_DEBUG_R("-----\t------\n");
    for (uint16_t ch = 0; ch < EXTERN_IO_AI_MAX; ++ch)
    {
        /*获取DAC值*/
        uint32_t adc_value = get_adc_value(ch);

        LHC_MODBUS_DEBUG_R("%d\t%d\n", ch, adc_value);
    }
}
MSH_CMD_EXPORT(see_adc, display adc channel value.);

/**
 * @brief   查看adc通道buf
 * @details
 * @param   none
 * @retval  none
 */
// static void see_adc_buf(void)
// {
//     LHC_MODBUS_DEBUG_R("\t\t\t\t[----adc_buf----]\nsn\tch0\tch1\tch2\tch3\tch4\tch5\tch6\tch7\tch8\tch9\tch10\tch11\tch12\tch13\tch14\tch15\n");
//     LHC_MODBUS_DEBUG_R("----\t----\t----\t----\t----\t----\t----\t----\t----\t----\t----\t----\t----\t----\t----\t----\t----\n");
//     for (uint8_t i = 0; i < EXTERN_IO_DI_MAX; ++i)
//     {
//         uint8_t base_addr = i * 16U;
//         /*获取DAC值*/
//         LHC_MODBUS_DEBUG_R("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
//                         i, adc_buffer[0 + base_addr], adc_buffer[1 + base_addr], adc_buffer[2 + base_addr], adc_buffer[3 + base_addr],
//                         adc_buffer[4 + base_addr], adc_buffer[5 + base_addr], adc_buffer[6 + base_addr], adc_buffer[7 + base_addr],
//                         adc_buffer[8 + base_addr], adc_buffer[9 + base_addr], adc_buffer[10 + base_addr], adc_buffer[11 + base_addr],
//                         adc_buffer[12 + base_addr], adc_buffer[13 + base_addr], adc_buffer[14 + base_addr], adc_buffer[15 + base_addr]);
//     }
// }
// MSH_CMD_EXPORT(see_adc_buf, display adc channel buf.);

/**
 * @brief   设置dac值
 * @details
 * @param   none
 * @retval  none
 */
static void set_dac(int argc, char **argv)
{
    if (argc < 3)
    {
        LHC_MODBUS_DEBUG_R("@error:too few parameters,please input'set_dac <(0~1)|(0~4095)>.'\n");
        return;
    }
    if (argc > 3)
    {
        LHC_MODBUS_DEBUG_R("@error:too many parameters,please input'set_dac <(0~1)|(0~4095)>.'\n");
        return;
    }
    uint32_t channel = (uint16_t)atoi(argv[1]);
    uint16_t data = (uint16_t)atoi(argv[2]);
    if (channel > 1)
    {
        LHC_MODBUS_DEBUG_R("@error:wrong channel number,please input'set_dac (0~1)|(0~4095).'\n");
        return;
    }
    if (data > 4095U)
    {
        LHC_MODBUS_DEBUG_R("@error:invalid input,please input'set_dac (0~1)|(0~4095).'\n");
        return;
    }
    LHC_MODBUS_DEBUG_R("channel[%d]= %d.\n", channel, data);
    set_dac_flag = data ? true : false;
    channel = channel ? DAC_CHANNEL_2 : DAC_CHANNEL_1;
    void *pHandle = &hdac;
    HAL_DAC_SetValue((DAC_HandleTypeDef *)pHandle, channel,
                     DAC_ALIGN_12B_R, (data & 0x0FFF));
}
MSH_CMD_EXPORT(set_dac, set_dac sample
               : set_dac<(ch : 0 ~1) | (val : 0 ~4095)>);

/**
 * @brief   设置AO输出4-20ma
 * @details
 * @param   none
 * @retval  none
 */
static void set_ao(int argc, char **argv)
{
    if (argc < 3)
    {
        LHC_MODBUS_DEBUG_R("@error:too few parameters,please input'set_ao <(0~1)|(0~20)>.'\n");
        return;
    }
    if (argc > 3)
    {
        LHC_MODBUS_DEBUG_R("@error:too many parameters,please input'set_ao <(0~1)|(0~20)>.'\n");
        return;
    }
    uint32_t channel = (uint16_t)atoi(argv[1]);
    float data = (float)atof(argv[2]);
    if (channel > 1)
    {
        LHC_MODBUS_DEBUG_R("@error:wrong channel number,please input'set_ao <(0~1)|(0~20)>.'\n");
        return;
    }
    if (data > 20.0F)
    {
        LHC_MODBUS_DEBUG_R("@error:invalid input,please input'set_ao <(0~1)|(0~20)>.'\n");
        return;
    }
    LHC_MODBUS_DEBUG_R("channel[%d]= %.3f.\n", channel, data);
    set_dac_flag = data ? true : false;
        output_current(&dac_map[channel], data);
}
MSH_CMD_EXPORT(set_ao, set_ao sample
               : set_ao<(ch : 0 ~1) | (val : 0 ~20)>);

/**
 * @brief   查看dac值
 * @details
 * @param   none
 * @retval  none
 */
static void see_dac(void)
{
    LHC_MODBUS_DEBUG_R("\ndac_info\nsn\tcp\t\tcq\n");
    LHC_MODBUS_DEBUG_R("---\t------\t------\n");
    for (uint16_t ch = 0; ch < EXTERN_IO_AO_MAX; ch++)
        LHC_MODBUS_DEBUG_R("%d\t%.3f\t\t%.3f\n", ch, dac_param[ch][0], dac_param[ch][1]);
}
MSH_CMD_EXPORT(see_dac, display dac output value.);
#endif

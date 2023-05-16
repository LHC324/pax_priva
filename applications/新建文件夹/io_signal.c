/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-09-15     LHC       the first version
 */
#include "io_signal.h"
#include "main.h"
#include "small_modbus_port.h"
// #include <rtthread.h>
#if defined(SMODBUS_USING_INPUT_REGISTER) || defined(SMODBUS_USING_HOLD_REGISTER)
// #include "Mcp4822.h"
#endif

#ifdef DBG_TAG
#undef DBG_TAG
#define DBG_TAG "io_signal"
#endif
#define DBG_LVL DBG_LOG

#define Get_DI_GPIO_Port(__Port, __P1, __P2) \
    ((__Port) ? (__P1) : (__P2))

#define Get_DI_GPIO_Pin(__Pin, __Offset, __P1, __P2) \
    ((__Pin) < (__Offset) ? ((__P1) >> (__Pin)) : (__P2) >> ((__Pin) - (__Offset)))

#define Get_DO_Port(__Pin)                       \
    ((__Pin) < 1U ? GPIOA : (__Pin) < 4U ? GPIOC \
                                         : GPIOD)

#define Get_DO_Pin(__Pin)                                            \
    ((__Pin) < 1U ? Q_0_Pin : (__Pin) < 4U ? Q_1_Pin << ((__Pin)-1U) \
                                           : Q_4_Pin << ((__Pin)-4U))

#define Get_ADC_Channel(__Ch)                                 \
    ((__Ch) < 4U ? ((__Ch) + 8U) : (__Ch) < 12U ? ((__Ch)-4U) \
                                                : (__Ch))
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

static void Output_Current(Dac_HandleTypeDef *p_ch, float data);
/*通过DMA循环采集的ADC数据缓存区*/
unsigned int Adc_buffer[ADC_DMA_SIZE] = {0};

/*adc采集对象*/
// adc_t adc_group[] = {{0}, {0}, {0}};
unsigned int adc_buf[IO_SIGNAL_ADC_NUM][IO_SIGNAL_ADC_DATA_SIZE];

/**
 * @brief   初始化adc
 * @details
 * @param   none
 * @retval  none
 */
// static int rt_stm32_adc_init(void)
// {
//     // HAL_ADCEx_Calibration_Start(&hadc1);
//     // HAL_ADCEx_Calibration_Start(&hadc2);
//     // HAL_ADCEx_Calibration_Start(&hadc3);

//     HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_group[0].buf, IO_SIGNAL_ADC_DATA_SIZE);
//     HAL_ADC_Start_IT(&hadc2);
//     HAL_ADC_Start_DMA(&hadc3, (uint32_t *)adc_group[2].buf, IO_SIGNAL_ADC_DATA_SIZE);

//     return 0;
// }
// /*在内核对象中初始化:https://blog.csdn.net/yang1111111112/article/details/93982354*/
// INIT_DEVICE_EXPORT(rt_stm32_adc_init);

extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern ADC_HandleTypeDef hadc3;
extern TIM_HandleTypeDef htim8;

/**
 * @brief   开启adc转换
 * @details duoadc同步采样：https://blog.csdn.net/qq_34022877/article/details/124099296
 * @param   none
 * @retval  none
 */
void start_adc_conv(void)
{
    /*adc1、adc2采用同一触发源，高16bit保存adc2转换结果、低16bit保存adc1转换结果*/
    HAL_ADC_Start_DMA(&hadc1, &adc_buf[0][0], IO_SIGNAL_ADC_DATA_SIZE);
    // HAL_ADC_Start_IT(&hadc2);
    HAL_ADC_Start(&hadc2);
    // HAL_ADCEx_MultiModeStart_DMA(&hadc1, (uint32_t *)&adc_buf[0][0], IO_SIGNAL_ADC_DATA_SIZE);
    HAL_ADC_Start_DMA(&hadc3, &adc_buf[2][0], IO_SIGNAL_ADC_DATA_SIZE);
    /*清空定时器计数值*/
    __HAL_TIM_SET_COUNTER(&htim8, 0U);
    HAL_TIM_Base_Start(&htim8); // 一定要在ADC开启后在开启触发
}
MSH_CMD_EXPORT(start_adc_conv, Enable adc conversion.);

/**
 * @brief   停止adc转换
 * @details
 * @param   none
 * @retval  none
 */
void stop_adc_conv(void)
{
    HAL_TIM_Base_Stop(&htim8);
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop(&hadc2);
    HAL_ADC_Stop_DMA(&hadc3);
}

/**
 * @brief   根据目标频率设置采样频率
 * @details
 * @param   cur_freq 当前输出频率
 * @retval  none
 */
void set_timer_sampling_freq(float cur_freq)
{

    static uint32_t target_time = 0;

    target_time = (uint32_t)((float)HAL_RCC_GetHCLKFreq() /
                                 (cur_freq * TIMER_SAMPLING_RATIO) -
                             1.0F);
    /*硬件限制：最高采样频率1M*/
    target_time = !target_time            ? 1
                  : target_time <= 0xFFFF ? target_time
                                          : 0xFFFF;

    HAL_TIM_Base_Stop(&htim8);
    __HAL_TIM_SET_AUTORELOAD(&htim8, target_time);
}

/**
 * @brief   adc 的dma转换完成回调函数
 * @details
 * @param   none
 * @retval  none
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    // if (hadc->Instance == ADC2)
    // {
    //     adc_t *padc = &adc_group[1];
    //     if (padc->count < IO_SIGNAL_ADC_DATA_SIZE)
    //     {
    //         padc->buf[padc->count++] = HAL_ADC_GetValue(hadc);
    //     }
    //     else
    //         padc->finish_flag = true;
    // }

    // static uint8_t syn_signal = 0;

    UNUSED(hadc);

    if (ADC1 == hadc->Instance) // adc1转换完成同时：adc2、adc3也完成了
    {
        extern rt_sem_t adc_semaphore;
        release_semaphore(adc_semaphore);
    }

    // if (ADC1 == hadc->Instance) // adc1转换完成同时：adc2、adc3也完成了
    //     syn_signal = 1;

    // if ((ADC3 == hadc->Instance) && (1 == syn_signal))
    //     syn_signal = 2;

    // if (2 == syn_signal)
    // {
    //     syn_signal = 0;
    //     extern rt_sem_t adc_semaphore;
    //     release_semaphore(adc_semaphore);
    // }
}

/**
 * @brief   adc dma传输过半中断
 * @details
 * @param   none
 * @retval  none
 */
// void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
// {
//     UNUSED(hadc);
//     extern rt_sem_t adc_semaphore;
//     release_semaphore(adc_semaphore);
// }

/**
 * @brief   通过均方根公式得AC有效值（电压/电流）
 * @details https://blog.csdn.net/qq_20222919/article/details/105093717
 * @param   cur_freq 当前输出频率
 * @retval  none
 */
// void get_ac_effective_value(float cur_freq)
// {
// }

/**
 * @brief  Get ADC channel value of DMA transmission
 * @note   Channel 1 and Channel 2 of ADC1 are used
 * @param  Channel Channel ID
 * @retval ADC average value
 */
static unsigned int Get_AdcValue(const unsigned int Channel)
{
    unsigned int sum = 0;
    unsigned int *pAdc_Str = &Adc_buffer[0];

    /*Prevent array out of bounds*/
    if (Channel > ADC_DMA_CHANNEL)
    {
        return 0;
    }
    /*Add channel offset*/
    pAdc_Str += Channel;

    for (; pAdc_Str < Adc_buffer + ADC_DMA_SIZE; pAdc_Str += ADC_DMA_CHANNEL)
    {
        sum += *pAdc_Str;
    }

    return (sum >> ADC_FILTER_SHIFT);
}

/**
 * @brief   外部数字量输入处理
 * @details STM32F103VET6共在io口扩展了8路数字输入
 * @param   None
 * @retval  None
 */
#if defined(SMODBUS_USING_INPUT_COIL)
void Read_Digital_Io(void)
{ /*引脚默认指向输入第一路*/
    GPIO_TypeDef *pGPIOx = NULL;
    uint16_t GPIO_Pinx = 0x00;
    pModbusHandle pd = Modbus_Object;
    uint8_t wbits[SIGNAL_IO_DI_MAX];
    memset(wbits, 0x00, sizeof(wbits));

    for (uint16_t i = 0; i < SIGNAL_IO_DI_MAX; i++)
    {
        // pGPIOx = Get_DI_GPIO_Port(!!(i < 3U), GPIOC, GPIOD);
        // GPIO_Pinx = Get_DI_GPIO_Pin(i, 3U, DI_0_Pin, DI_3_Pin);
        /*读取外部数字引脚状态:翻转光耦的输入信号*/
        // wbits[i] = (uint8_t)!HAL_GPIO_ReadPin(pGPIOx, GPIO_Pinx);
#if (SMODBUS_USING_DEBUG)
        // SMODBUS_DEBUG_D("@note:write[%d] = %d.", i, wbits[i]);
#endif
    }

    if (pd && pGPIOx)
    {
        /*写入对应寄存器*/
        if (!pd->Mod_Operatex(pd, InputCoil, Write, SIGNAL_IO_DI_START_ADDR,
                              wbits, SIGNAL_IO_DI_MAX))
        {
#if (SMODBUS_USING_DEBUG)
            SMODBUS_DEBUG_D("@error:Input coil write failed.");
#endif
        }
    }
}
#endif

#if defined(SMODBUS_USING_COIL)
// typedef struct
// {
//     uint8_t sn;
//     Gpiox_info gpio;
// } digital_output_struct;

/**
 * @brief   获取目标io口结构
 * @details
 * @param   sn gpio端口的目标位置
 * @retval  目标gpio的结构指针
 */
static Gpiox_info *get_target_gpio(uint8_t sn)
{

    static Gpiox_info gpio_group[] = {
        {.pGPIOx = Q10_GPIO_Port, .Gpio_Pin = Q10_Pin},
        {.pGPIOx = Q11_GPIO_Port, .Gpio_Pin = Q11_Pin},
        {.pGPIOx = Q12_GPIO_Port, .Gpio_Pin = Q12_Pin},
        {.pGPIOx = Q13_GPIO_Port, .Gpio_Pin = Q13_Pin},
        {.pGPIOx = Q14_GPIO_Port, .Gpio_Pin = Q14_Pin},
        {.pGPIOx = Q15_GPIO_Port, .Gpio_Pin = Q15_Pin},
        {.pGPIOx = Q16_GPIO_Port, .Gpio_Pin = Q16_Pin},
        {.pGPIOx = Q17_GPIO_Port, .Gpio_Pin = Q17_Pin},
        {.pGPIOx = Q18_GPIO_Port, .Gpio_Pin = Q18_Pin},
        {.pGPIOx = Q19_GPIO_Port, .Gpio_Pin = Q19_Pin},
        {.pGPIOx = Q1A_GPIO_Port, .Gpio_Pin = Q1A_Pin},
        {.pGPIOx = Q1B_GPIO_Port, .Gpio_Pin = Q1B_Pin},
        {.pGPIOx = NULL, .Gpio_Pin = 0},
        {.pGPIOx = NULL, .Gpio_Pin = 0},
        {.pGPIOx = NULL, .Gpio_Pin = 0},
        {.pGPIOx = NULL, .Gpio_Pin = 0},
        {.pGPIOx = Q22_GPIO_Port, .Gpio_Pin = Q22_Pin},
        {.pGPIOx = Q23_GPIO_Port, .Gpio_Pin = Q23_Pin},
        {.pGPIOx = Q24_GPIO_Port, .Gpio_Pin = Q24_Pin},
        {.pGPIOx = Q25_GPIO_Port, .Gpio_Pin = Q25_Pin},
        {.pGPIOx = Q26_GPIO_Port, .Gpio_Pin = Q26_Pin},
        {.pGPIOx = Q27_GPIO_Port, .Gpio_Pin = Q27_Pin},
        {.pGPIOx = Q28_GPIO_Port, .Gpio_Pin = Q28_Pin},
        {.pGPIOx = Q29_GPIO_Port, .Gpio_Pin = Q29_Pin},
        {.pGPIOx = Q2A_GPIO_Port, .Gpio_Pin = Q2A_Pin},
        {.pGPIOx = NULL, .Gpio_Pin = 0},
        {.pGPIOx = NULL, .Gpio_Pin = 0},
        {.pGPIOx = NULL, .Gpio_Pin = 0},
    };
#define IO_GPIO_SIZE() (sizeof(gpio_group) / sizeof(gpio_group[0]))

    if (sn < IO_GPIO_SIZE())
        return &gpio_group[sn];
    return NULL;
}

/**
 * @brief   io端口输出规则检查
 * @details
 * @param   pdata 数据指针
 * @param   len   数据长度
 * @retval  true/false
 */
static bool gpio_rule_check(uint8_t *pdata, uint8_t site)
{
#define K20_OFFSET (SIGNAL_IO_DO_MAX / 2U)
    bool result = false;
    if (pdata)
    {
        /*rule1:K1x与K2x不能同时打开*/
        if (site < K20_OFFSET)
            result = (*(pdata + site)) && (*(pdata + (K20_OFFSET + site))) ? true : false;
        else
            result = (*(pdata + (site - K20_OFFSET))) && (*(pdata + site)) ? true : false;
        /*rule2:K1(x)与K1(x+1)、K2(x)与K2(x+1)没有必要同时打开*/
    }
    return result;
#undef K20_OFFSET
}

/**
 * @brief   数字量输出
 * @details STM32F030F4共在io口扩展了26路数字输出
 * @param   None
 * @retval  None
 */
void Write_Digital_IO(void)
{
    pModbusHandle pd = Modbus_Object;
    uint8_t rbits[SIGNAL_IO_DO_MAX];

    if (pd)
    {
        /*读取对应寄存器*/
        if (!pd->Mod_Operatex(pd, Coil, Read, SIGNAL_IO_DO_START_ADDR,
                              rbits, SIGNAL_IO_DO_MAX))
        {
#if (SMODBUS_USING_DEBUG)
            SMODBUS_DEBUG_D("@error:Coil reading failed!\r\n");
            return;
#endif
        }

        for (uint8_t i = 0; i < SIGNAL_IO_DO_MAX; i++)
        {
            Gpiox_info *pgpio = get_target_gpio(i);
            /*检查K1x和K2x不允许同时打开*/
            bool result = gpio_rule_check(rbits, i);
            if (result)
            {
#if (SMODBUS_USING_DEBUG)
                SMODBUS_DEBUG_D("@error:Illegal coil operation, position[%d]:%#x.\r\n", i, rbits[i]);
#endif
                continue;
            }
            if (pgpio && pgpio->pGPIOx)
                HAL_GPIO_WritePin((GPIO_TypeDef *)pgpio->pGPIOx, pgpio->Gpio_Pin, (GPIO_PinState)rbits[i]);
        }
    }
}
#endif

/**
 * @brief   外部模拟量输入处理
 * @details STM32F030F4共在io口扩展了8路模拟输入
 * @param   None
 * @retval  None
 */
#if defined(SMODBUS_USING_INPUT_REGISTER)
void Read_Analog_Io(void)
{
    static bool first_flag = false;
    pModbusHandle pd = Modbus_Object;
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
    static KFP pkfp[SIGNAL_IO_AI_MAX];
#else
    SideParm side = {
        .First_Flag = true,
        .Head = &side.SideBuff[0],
        .SideBuff = {0},
        .Sum = 0,
    };
    static SideParm pside[SIGNAL_IO_AI_MAX];
#endif
    /*保证仅首次copy*/
    if (!first_flag)
    {
        first_flag = true;
        for (uint16_t ch = 0; ch < SIGNAL_IO_AI_MAX; ch++)
        {
#if defined(TOOL_USING_KALMAN)
            memcpy(&pkfp[ch], &hkfp, sizeof(hkfp));
#else
            memcpy(&pside[ch], &side, sizeof(pside));
#endif
        }
    }
    float *pdata = (float *)smd_malloc(SIGNAL_IO_AI_MAX * sizeof(float));
    if (!pdata)
        goto __exit;
    memset(pdata, 0x00, SIGNAL_IO_AI_MAX * sizeof(float));

    for (uint16_t ch = 0; ch < SIGNAL_IO_AI_MAX; ch++)
    {
        /*获取DAC值*/
        uint32_t adc_value = Get_AdcValue(ch);
        /*前8路电流，后8路电压*/
        pdata[ch] = ch < (SIGNAL_IO_AI_MAX / 2U)
                        ? Get_Target_Value(adc_value, CURRENT_CP, CURRENT_CQ)
                        : Get_Target_Value(adc_value, VOLTAGE_CP, VOLTAGE_CQ);
#if (SMODBUS_USING_DEBUG)
        //        SMODBUS_DEBUG_D("ADC[%d] = %d.", ch, adc_value);
#endif
        // pdata[ch] = (pdata[ch] <= AI_CQ) ? 0 : pdata[ch];
        /*滤波处理*/
#if defined(TOOL_USING_KALMAN)
        pdata[ch] = kalmanFilter(&pkfp[ch], pdata[ch]);
#else
        pdata[ch] = sidefilter(&pside[ch], pdata[ch]);
#endif
        /*大小端转换*/
        //        endian_swap((uint8_t *)&pdata[ch], 0U, sizeof(float));

#if (SMODBUS_USING_DEBUG)
        // SMODBUS_DEBUG_D("R_AD[%d] = %.3f.", ch, pdata[ch]);
#endif
    }
    if (pd)
    {
        /*写入对应寄存器*/
        if (!pd->Mod_Operatex(pd, InputRegister, Write, SIGNAL_IO_AI_START_ADDR,
                              (uint8_t *)pdata, SIGNAL_IO_AI_MAX * sizeof(float)))
        {
#if defined(SMODBUS_USING_DEBUG)
            SMODBUS_DEBUG_D("@error:Input register write failed!\r\n");
#endif
        }
    }

__exit:
    smd_free(pdata);
}
#endif

/**
 * @brief	对目标通道输出电流
 * @details
 * @param   p_ch :目标通道
 * @param	data:写入的数据
 * @retval	None
 */
static void Output_Current(Dac_HandleTypeDef *p_ch, float data)
{
    // void *pHandle = &hdac;
    // uint16_t value = (uint16_t)(dac_param[p_ch->Channel][0] * data +
    //                             dac_param[p_ch->Channel][1]);

    // value = data ? ((data > 20.0F) ? 0x0FFF : value) : 0U;

    // uint32_t channel = (p_ch->Channel < DAC_OUT2) ? DAC_CHANNEL_2 : DAC_CHANNEL_1;
    // if (pHandle)
    //     HAL_DAC_SetValue((DAC_HandleTypeDef *)pHandle, channel,
    //                      DAC_ALIGN_12B_R, (value & 0x0FFF));
}

/**
 * @brief   模拟量输出
 * @details STM32F030F4共在io口扩展了8路数字输出
 * @param   None
 * @retval  None
 */
#if defined(SMODBUS_USING_HOLD_REGISTER)
void Write_Analog_IO(void)
{
    pModbusHandle pd = Modbus_Object;

    float *pdata = (float *)smd_malloc(SIGNAL_IO_AO_MAX * sizeof(float));
    if (!pdata)
        goto __exit;
    memset(pdata, 0x00, SIGNAL_IO_AO_MAX * sizeof(float));

    if (!pd)
    {
        goto __exit;
    }
    /*读出保持寄存器*/
    if (!pd->Mod_Operatex(pd, HoldRegister, Read, SIGNAL_IO_AO_START_ADDR,
                          (uint8_t *)pdata, SIGNAL_IO_AO_MAX * sizeof(float)))
    {
#if (SMODBUS_USING_DEBUG)
        SMODBUS_DEBUG_D("@error:Hold register read failed!\r\n");
#endif
    }

    for (uint16_t ch = 0; ch < SIGNAL_IO_AO_MAX; ch++)
    {
#if defined(SMODBUS_USING_DEBUG)
        // shellPrint(Shell_Object, "W_AD[%d] = %.3f\r\n", ch, pdata[ch]);
#endif
        if (!set_dac_flag)
            Output_Current(&dac_map[ch], pdata[ch]);
    }

__exit:
    smd_free(pdata);
}
#endif

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
    pModbusHandle pd = Modbus_Object;
    uint8_t rbits[SIGNAL_IO_DI_MAX];
    memset(rbits, 0x00, sizeof(rbits));

    if (pd)
    {
        /*写入对应寄存器*/
        if (!pd->Mod_Operatex(pd, InputCoil, Read, SIGNAL_IO_DI_START_ADDR,
                              rbits, SIGNAL_IO_DI_MAX))
        {
#if (SMODBUS_USING_DEBUG)
            SMODBUS_DEBUG_D("@error:Input coil write failed.");
#endif
            return;
        }
        SMODBUS_DEBUG_R("\t\t\t[----digital_output_info----]\nsn\tpinx\n");
        SMODBUS_DEBUG_R("--\t----\n");
        for (uint16_t i = 0; i < SIGNAL_IO_DI_MAX; i++)
            SMODBUS_DEBUG_R("%d\t%d\n", i, rbits[i]);
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
    pModbusHandle pd = Modbus_Object;
    uint8_t rbits[SIGNAL_IO_DO_MAX];
    memset(rbits, 0x00, sizeof(rbits));
    if (pd)
    {
        if (!pd->Mod_Operatex(pd, Coil, Read, SIGNAL_IO_DO_START_ADDR,
                              rbits, SIGNAL_IO_DO_MAX))
        {
#if (SMODBUS_USING_DEBUG)
            SMODBUS_DEBUG_D("@error:coil read failed.");
#endif
        }
        SMODBUS_DEBUG_R("\t\t\t[----digital_output_info----]\nsn\tpinx\n");
        SMODBUS_DEBUG_R("---\t----\n");
        for (uint16_t i = 0; i < SIGNAL_IO_DO_MAX; ++i)
            SMODBUS_DEBUG_R("%d\t%d\n", i, rbits[i]);
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
    pModbusHandle pd = Modbus_Object;

    if (argc < 2)
    {
        SMODBUS_DEBUG_R("@error: Please input'set_do <(0~9) | (0/1)>.'\n");
        return;
    }
    if (argc > 3)
    {
        SMODBUS_DEBUG_R("@error:parameter is too long,please input'set_do <(0~9) | (0/1)>.'\n");
        return;
    }

    uint8_t pin = (uint8_t)atoi(argv[1]);
    uint8_t state = (uint8_t)atoi(argv[2]);

    if (pin >= SIGNAL_IO_DO_MAX)
    {
        SMODBUS_DEBUG_D("@error:parameter[1]%d error,please input'0~25'.\n", pin);
        return;
    }
    if (state > 1U)
    {
        SMODBUS_DEBUG_D("@error:parameter[2]:%d error,please input'0/1'.\n", state);
        return;
    }

    if (pd)
    {
        uint8_t start_addr = SIGNAL_IO_DI_START_ADDR + pin;
        /*写入对应寄存器*/
        if (!pd->Mod_Operatex(pd, Coil, Write, start_addr, &state, 1U))
        {
#if (SMODBUS_USING_DEBUG)
            SMODBUS_DEBUG_D("@error:coil write failed.");
#endif
        }
    }
    see_do();
}
MSH_CMD_EXPORT(set_do, set_do sample
               : set_do<(0 ~9) | (0 / 1)>);

/**
 * @brief   查看adc通道值
 * @details
 * @param   none
 * @retval  none
 */
static void see_adc(void)
{
    SMODBUS_DEBUG_R("\t\t\t\t[----adc_info----]\nchannel\tcur_val\ttar_val(A/V)\n");
    SMODBUS_DEBUG_R("--------\t------\t-----------\n");
    for (uint16_t i = 0; i < SIGNAL_IO_AI_MAX; ++i)
    {
        /*获取DAC值*/
        uint32_t adc_value = Get_AdcValue(i);
        float value = i < (SIGNAL_IO_AI_MAX / 2U)
                          ? Get_Target_Value(adc_value, CURRENT_CP, CURRENT_CQ)
                          : Get_Target_Value(adc_value, VOLTAGE_CP, VOLTAGE_CQ);
        SMODBUS_DEBUG_R("%d\t%d\t%.3f\n", i, adc_value, value);
    }
}
MSH_CMD_EXPORT(see_adc, display adc channel value.);

/**
 * @brief   查看adc通道buf
 * @details
 * @param   none
 * @retval  none
 */
static void see_adc_buf(void)
{
    SMODBUS_DEBUG_R("\t\t\t\t[----adc_buf----]\nsn\tch0\tch1\tch2\tch3\tch4\tch5\tch6\tch7\tch8\tch9\tch10\tch11\tch12\tch13\tch14\tch15\n");
    SMODBUS_DEBUG_R("----\t----\t----\t----\t----\t----\t----\t----\t----\t----\t----\t----\t----\t----\t----\t----\t----\n");
    for (uint8_t i = 0; i < SIGNAL_IO_DI_MAX; ++i)
    {
        uint8_t base_addr = i * 16U;
        /*获取DAC值*/
        SMODBUS_DEBUG_R("%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
                        i, Adc_buffer[0 + base_addr], Adc_buffer[1 + base_addr], Adc_buffer[2 + base_addr], Adc_buffer[3 + base_addr],
                        Adc_buffer[4 + base_addr], Adc_buffer[5 + base_addr], Adc_buffer[6 + base_addr], Adc_buffer[7 + base_addr],
                        Adc_buffer[8 + base_addr], Adc_buffer[9 + base_addr], Adc_buffer[10 + base_addr], Adc_buffer[11 + base_addr],
                        Adc_buffer[12 + base_addr], Adc_buffer[13 + base_addr], Adc_buffer[14 + base_addr], Adc_buffer[15 + base_addr]);
    }
}
MSH_CMD_EXPORT(see_adc_buf, display adc channel buf.);

/**
 * @brief   设置dac值
 * @details
 * @param   none
 * @retval  none
 */
static void set_dac(int argc, char **argv)
{
    if (argc > 3)
    {
        SMODBUS_DEBUG_R("@error:too many parameters,please input'set_dac <(0~1)|(0~4095)>.'\n");
        return;
    }
    uint32_t channel = (uint16_t)atoi(argv[1]);
    uint16_t data = (uint16_t)atoi(argv[2]);
    if (channel > 1)
    {
        SMODBUS_DEBUG_R("@error:wrong channel number,please input'set_dac (0~1)|(0~4095).'\n");
        return;
    }
    if (data > 4095U)
    {
        SMODBUS_DEBUG_R("@error:invalid input,please input'set_dac (0~1)|(0~4095).'\n");
        return;
    }
    SMODBUS_DEBUG_R("channel[%d]= %d.\n", channel, data);
    set_dac_flag = data ? true : false;
    // void *pHandle = &hdac;
    // HAL_DAC_SetValue((DAC_HandleTypeDef *)pHandle, channel,
    //                  DAC_ALIGN_12B_R, (data & 0x0FFF));
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
    if (argc > 3)
    {
        SMODBUS_DEBUG_R("@error:too many parameters,please input'set_ao <(0~1)|(0~20)>.'\n");
        return;
    }
    uint32_t channel = (uint16_t)atoi(argv[1]);
    float data = (float)atof(argv[2]);
    if (channel > 1)
    {
        SMODBUS_DEBUG_R("@error:wrong channel number,please input'set_ao <(0~1)|(0~20)>.'\n");
        return;
    }
    if (data > 20.0F)
    {
        SMODBUS_DEBUG_R("@error:invalid input,please input'set_ao <(0~1)|(0~20)>.'\n");
        return;
    }
    SMODBUS_DEBUG_R("channel[%d]= %.3f.\n", channel, data);
    set_dac_flag = data ? true : false;
    for (uint16_t ch = 0; ch < SIGNAL_IO_AO_MAX; ch++)
        Output_Current(&dac_map[ch], data);
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
    SMODBUS_DEBUG_R("\t\t\t\t[----dac_info----]\nsn\tcp\t\tcq\n");
    SMODBUS_DEBUG_R("---\t------\t------\n");
    for (uint16_t ch = 0; ch < SIGNAL_IO_AO_MAX; ch++)
        SMODBUS_DEBUG_R("%d\t%.3f\t\t%.3f\n", ch, dac_param[ch][0], dac_param[ch][1]);
}
MSH_CMD_EXPORT(see_dac, display dac output value.);
#endif

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
#define DBG_TAG "extern_io"
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

static Gpiox_info di_gpio[] = {0};

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
    RT_ASSERT(num  <= EXTERN_IO_DI_MAX);

    modbus_t pd = (modbus_t)modbus;
    unsigned char wbits[EXTERN_IO_DI_MAX], *bit = wbits;
    Gpiox_info *pi = di_gpio;

     lhc_tool_memset(wbits, 0x00, sizeof(wbits));

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

     return (pd->Mod_Operatex(pd, InputCoil, lhc_modbus_write, addr, wbits, num) ? 0 : -1);
}
#endif

#if defined(LHC_MODBUS_USING_COIL)
static Gpiox_info do_gpio[] = {0};

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
        if(po->port)
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
    RT_ASSERT(num  <= EXTERN_IO_AI_MAX);

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
    modbus_t pd = Lte_Modbus_Object;
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
        LHC_MODBUS_DEBUG_R("\t\t\t[----digital_output_info----]\nsn\tpinx\n");
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
    modbus_t pd = Lte_Modbus_Object;
    uint8_t rbits[EXTERN_IO_DI_MAX];
    lhc_tool_memset(rbits, 0x00, sizeof(rbits));
    if (pd)
    {
        if (!pd->Mod_Operatex(pd, Coil, lhc_modbus_read, SIGNAL_IO_DO_START_ADDR,
                              rbits, EXTERN_IO_DI_MAX))
        {
#if (LHC_MODBUS_USING_DEBUG)
            LHC_MODBUS_DEBUG_D("@error:coil read failed.");
#endif
        }
        LHC_MODBUS_DEBUG_R("\t\t\t[----digital_output_info----]\nsn\tpinx\n");
        LHC_MODBUS_DEBUG_R("---\t----\n");
        for (uint16_t i = 0; i < EXTERN_IO_DI_MAX; ++i)
            LHC_MODBUS_DEBUG_R("%d\t%d\n", i, rbits[i]);
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
    modbus_t pd = Lte_Modbus_Object;

    if (argc < 2)
    {
        LHC_MODBUS_DEBUG_R("@error: Please input'set_do <(0~9) | (0/1)>.'\n");
        return;
    }
    if (argc > 3)
    {
        LHC_MODBUS_DEBUG_R("@error:parameter is too long,please input'set_do <(0~9) | (0/1)>.'\n");
        return;
    }

    uint8_t pin = (uint8_t)atoi(argv[1]);
    uint8_t state = (uint8_t)atoi(argv[2]);

    if (pin >= EXTERN_IO_DI_MAX)
    {
        LHC_MODBUS_DEBUG_D("@error:parameter[1]%d error,please input'0~25'.\n", pin);
        return;
    }
    if (state > 1U)
    {
        LHC_MODBUS_DEBUG_D("@error:parameter[2]:%d error,please input'0/1'.\n", state);
        return;
    }

    if (pd)
    {
        uint8_t start_addr = SIGNAL_IO_DI_START_ADDR + pin;
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
               : set_do<(0 ~9) | (0 / 1)>);

/**
 * @brief   查看adc通道值
 * @details
 * @param   none
 * @retval  none
 */
static void see_adc(void)
{
    LHC_MODBUS_DEBUG_R("\t\t\t\t[----adc_info----]\nchannel\tcur_val\ttar_val(A/V)\n");
    LHC_MODBUS_DEBUG_R("--------\t------\t-----------\n");
    for (uint16_t i = 0; i < EXTERN_IO_AI_MAX; ++i)
    {
        /*获取DAC值*/
        uint32_t adc_value = get_adc_value(i);
        float value = i < (EXTERN_IO_AI_MAX / 2U)
                          ? Get_Target_Value(adc_value, CURRENT_CP, CURRENT_CQ)
                          : Get_Target_Value(adc_value, VOLTAGE_CP, VOLTAGE_CQ);
        LHC_MODBUS_DEBUG_R("%d\t%d\t%.3f\n", i, adc_value, value);
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
    for (uint16_t ch = 0; ch < EXTERN_IO_AO_MAX; ch++)
        output_current(&dac_map[ch], data);
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
    LHC_MODBUS_DEBUG_R("\t\t\t\t[----dac_info----]\nsn\tcp\t\tcq\n");
    LHC_MODBUS_DEBUG_R("---\t------\t------\n");
    for (uint16_t ch = 0; ch < EXTERN_IO_AO_MAX; ch++)
        LHC_MODBUS_DEBUG_R("%d\t%.3f\t\t%.3f\n", ch, dac_param[ch][0], dac_param[ch][1]);
}
MSH_CMD_EXPORT(see_dac, display dac output value.);
#endif

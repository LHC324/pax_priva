#include "ad9833.h"
#include "main.h"
#include <rtdbg.h>
// #include "io_signal.h"
#include "small_modbus_port.h"

#ifdef DBG_TAG
#undef DBG_TAG
#endif
#define DBG_TAG "ad9833"
#define DBG_LVL DBG_LOG
#define USING_AD9833_DEBUG 1
#define AD9833_DEBUG_R dbg_raw
#define AD9833_DEBUG_D LOG_D

/*stm32 SPI使用方法：https://blog.csdn.net/otto1230/article/details/100122559*/
extern SPI_HandleTypeDef hspi1;
extern void set_timer_sampling_freq(float cur_freq);
/*定义一个ad9833对象*/
ad9833_t ad9833_object = {
    .pspi = &hspi1,
    .cs = {
        .pGPIOx = C_S_GPIO_Port,
        .Gpio_Pin = C_S_Pin,
    },
    .scyn = {
        .pGPIOx = FSYCN_GPIO_Port,
        .Gpio_Pin = FSYCN_Pin,
    },
    .set_seampling_freq = set_timer_sampling_freq,
};

/*ad9833互斥标志*/
static bool set_ad9833_flag = false;

// static const char *ad9833_info[] = {
//     [CONF_MODE] = "Note: Enter configuration!\r\n",
//     [FREE_MODE] = "Note: Enter free mode!\r\n",
//     [UNKOWN_MODE] = "Error: Unknown mode!\r\n",
//     [USER_ESC] = "Warning: User cancel!\r\n",
//     [CONF_ERROR] = "Error: Configuration failed!\r\n",
//     [CONF_TOMEOUT] = "Error: Configuration timeout.\r\n",
//     [CONF_SUCCESS] = "Success: Configuration succeeded!\r\n",
//     [INPUT_ERROR] = "Error: Input error!\r\n",
//     [CMD_MODE] = "Note: Enter transparent mode!\r\n",
//     [CMD_SURE] = "Note: Confirm to exit the transparent transmission mode?\r\n",
//     [SET_ECHO] = "Note: Set echo?\r\n",
//     [SET_UART] = "Note: Set serial port parameters!\r\n",
//     [WORK_MODE] = "Note: Please enter the working mode?(0:TRANS/1:FP)\r\n",
//     [POWER_MODE] = "Note: Please enter the power consumption mode?(0:RUN/1:LR/2:WU/3:LSR)\r\n",
//     [SET_TIDLE] = "Note: Set idle time.\r\n",
//     [SET_TWAKEUP] = "Note: Set wake-up interval.\r\n",
//     [SPEED_GRADE] = "Note: Please enter the rate level?(1~10)\r\n",
//     [TARGET_ADDR] = "Note: Please enter the destination address?(0~65535)\r\n",
//     [CHANNEL] = "Note: Please enter the channel?(0~127)\r\n",
//     [CHECK_ERROR] = "Note: Enable forward error correction?(1:true/0:false)\r\n",
//     [TRANS_POWER] = "Note: Please input the transmission power?(10~20db)\r\n",
//     [SET_OUTTIME] = "Note: Please enter the receiving timeout?(LR/LSR mode is valid,0~15000ms)\r\n",
//     // [SET_KEY] = "Note: Please enter the data encryption word?(16bit Hex)\r\n",
//     [RESTART] = "Note: Device restart!\r\n",
//     [SIG_STREN] = "Note: Query signal strength.\r\n",
//     [EXIT_CMD] = "Note: Exit command mode!\r\n",
//     [RECOVERY] = "Note: Restore default parameters!\r\n",
//     [SELECT_NID] = "Note: Query node ID?\r\n",
//     [SELECT_VER] = "Note: Query version number?\r\n",
//     [LOW_PFLAG] = "Note: Set / query fast access low power enable flag.\r\n",
//     [LOW_PDATE] = "Note: Set / query fast access to low-power data.\r\n",
//     [FINISH_FLAG] = "Note: Set / query sending completion reply flag.\r\n",
//     [EXIT_CONF] = "Note: Please press \"ESC\" to end the configuration!\r\n",
//     [NO_CMD] = "Error: Command does not exist!\r\n",
// };

static void ad9833_dely(void)
{
    for (uint8_t i = 10; i; i--)
    {
#ifdef __ARMCC_VERSION
        __nop();
#elif defined(__ICCARM__)

#elif defined(__GNUC__)

#define __nop() __asm__ __volatile__("nop" ::)
        __nop();
#endif
    }
}

/**
 * @brief  ad983写16bit数据
 * @note
 * @param  pa ad9833句柄
 * @param  data 16bit数据
 * @retval None
 */
static void ad9833_write(ad9833_t *pa, uint16_t data)
{
#define SPI_TIMEOUT 0x100
    if (pa && pa->cs.pGPIOx && pa->scyn.pGPIOx)
    {
        /*使能器件DAC同步输出信号*/
        HAL_GPIO_WritePin((GPIO_TypeDef *)pa->scyn.pGPIOx, pa->scyn.Gpio_Pin, (GPIO_PinState)GPIO_PIN_RESET);
        ad9833_dely();
        /*调用硬件spi发送函数*/
        HAL_SPI_Transmit((SPI_HandleTypeDef *)pa->pspi, (uint8_t *)&data, sizeof(data) / sizeof(uint16_t),
                         SPI_TIMEOUT);
        /*关闭器件DAC同步输出信号*/
        HAL_GPIO_WritePin((GPIO_TypeDef *)pa->scyn.pGPIOx, pa->scyn.Gpio_Pin, (GPIO_PinState)GPIO_PIN_SET);
    }
    // #undef SPI_TIMEOUT
}

/**
 * @brief  ad983写16bit数据
 * @note   设置完幅度值时才需要cs信号
 * @param  pa ad9833句柄
 * @param  data 16bit数据
 * @retval None
 */
void ad9833_amplitude(ad9833_t *pa,
                      uint16_t data)
{
    if (pa && pa->cs.pGPIOx)
    {
        /*初始化spi为0、0模式*/
        hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
        HAL_SPI_Init(&hspi1);
        /*软件拉低CS信号*/
        HAL_GPIO_WritePin((GPIO_TypeDef *)pa->cs.pGPIOx, pa->cs.Gpio_Pin, (GPIO_PinState)GPIO_PIN_RESET);
        // HAL_GPIO_WritePin(C_S_GPIO_Port, C_S_Pin, GPIO_PIN_RESET);
        // ad9833_write(pa, data | 0x1100);
        data |= 0x1100;
        HAL_SPI_Transmit((SPI_HandleTypeDef *)pa->pspi, (uint8_t *)&data, sizeof(data) / sizeof(uint16_t),
                         SPI_TIMEOUT);
        /*软件拉高CS信号*/
        HAL_GPIO_WritePin((GPIO_TypeDef *)pa->cs.pGPIOx, pa->cs.Gpio_Pin, (GPIO_PinState)GPIO_PIN_SET);
        // HAL_GPIO_WritePin(C_S_GPIO_Port, C_S_Pin, GPIO_PIN_SET);
        // AD9833_DEBUG_R("data:%#x\n", data | 0x1100);
    }
#undef SPI_TIMEOUT
}

/**
 * @brief  ad983设置相位寄存器
 * @note   寄存器0/1
 * @param  pa ad9833句柄
 * @param  data 12bit数据
 * @retval None
 */
static uint16_t ad9833_set_phase_register(ad9833_phase_regs phase_reg,
                                          uint16_t data)
{
    if (phase_reg > phase_reg1)
        return 0;
    data |= 0xC000; /*写相位寄存器时，位 D15 和 D14 都置 1*/
    data = phase_reg < phase_reg1 ? data & 0xDFFF : data | 0x2000;
    return data;
}

/**
 * @brief  ad983绘制目标波形
 * @note
 * @param  pa ad9833句柄
 * @param  freq 频率
 * @param  freq_sfr 0/1
 * @param  wave 波形模式[TRI_WAVE(三角波);SIN_WAVE(正弦波);SQU_WAVE(方波)]
 * @param  phase 波形初相位
 * @param  phase_sfr 相位寄存器
 * @retval None
 */
void ad9833_set_wave(ad9833_t *pa,
                     float freq,
                     uint8_t freq_sfr,
                     ad9833_wave wave,
                     uint16_t phase,
                     ad9833_phase_regs phase_sfr)
{
    float fre_data = (freq * AD9833_REG_MAX_VAL) / AD9833_SOC_FREQ;
    uint16_t sfr_haead = freq_sfr ? 0x8000 : 0x4000;
    uint16_t freq_lsb = ((uint32_t)fre_data & 0x3fff) | sfr_haead;          // 把16bit的最高两位去除，16位数换去掉高位后变成了14位
    uint16_t freq_msb = (((uint32_t)fre_data >> 14U) & 0x3fff) | sfr_haead; // 高16bit去掉最高2位
    // uint16_t phase_data = phase | 0xC000;                                   //相位值;
    uint16_t phase_data = ad9833_set_phase_register(phase_sfr, phase);
    uint16_t wave_mode = wave < ad9833_tri   ? AD9833_SIN_WAVE_CMD
                         : wave < ad9833_squ ? AD9833_TRI_WAVE_CMD
                                             : AD9833_SQU_WAVE_CMD;

    uint16_t ad9833_cmd_table[] = {
        AD9833_RESET_CMD,       /*复位ad9833*/
        AD9833_WRITE_ONECE_CMD, /*选择数据一次写入，b28位和reset位为1*/
        freq_lsb,               // L14，选择频率寄存器0的低14位数据输入
        freq_msb,               // H14 频率寄存器的高14位数据输入
        phase_data,             // 设置相位
        wave_mode,              // 设置波形
    };
    static float last_freq = 0;
    // AD9833_DEBUG_R("[ad9833_info]\nf_lsb\tf_msb\tphase\twave\n");
    // AD9833_DEBUG_R("-----\t-----\t-----\t----\n%#x\t%#x\t%#x\t%#x\n",
    //                freq_lsb, freq_msb, phase_data, wave_mode);

    /*设置采样定时器*/
    if (pa->set_seampling_freq && (freq != last_freq))
    {
        pa->set_seampling_freq(freq);
        last_freq = freq;
    }
    /*初始化spi为1、0模式*/
    hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
    HAL_SPI_Init(&hspi1);
    for (uint16_t *p = ad9833_cmd_table;
         p < ad9833_cmd_table + sizeof(ad9833_cmd_table) / sizeof(ad9833_cmd_table[0]); ++p)
    {
        ad9833_write(pa, *p);
    }
}

// #include "ad9833function.h"

// Ad9833ObjectType ad9833;

// /*定义片选信号函数*/
// void AD9833CS(AD9833CSType en)
// {
//     if (AD9833CS_ENABLE == en)
//     {
//         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
//         HAL_GPIO_WritePin(FSYCN_GPIO_Port, FSYCN_Pin, GPIO_PIN_RESET);
//     }
//     else
//     {
//         HAL_GPIO_WritePin(FSYCN_GPIO_Port, FSYCN_Pin, GPIO_PIN_SET);
//         HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
//     }
// }

// /*定义发送数据函数*/
// void AD9833TransmitData(uint8_t *wData, uint16_t wSize)
// {
// extern SPI_HandleTypeDef hspi1;
//     // AD9833_DEBUG_D("data:%#x\n", *wData);
//     HAL_SPI_Transmit(&hspi1, wData, wSize, 1000);
// }

/**
 * @brief   设置ad9833输出目标波形
 * @details
 * @param   none
 * @retval  none
 */
static void set_ad9833(int argc, char **argv)
{
    // AD9833Initialization(&ad9833, 25.0, AD9833TransmitData, AD9833CS, HAL_Delay);

    if (argc > 7)
    {
        AD9833_DEBUG_D("@error:too many parameters,please input'<set_ad9833(freq)|(f_sfr)|(wave: sin/tri/squ)|(phase)|(p_sfr)|(range)>.'\n");
        return;
    }
    float freq = (float)atof(argv[1]);
    if (freq > 12000000.0F)
    {
        AD9833_DEBUG_D("@error:wrong frequency ,please input'0.1hz-12Mhz'.\n");
        return;
    }
    /*关闭程序输出频率*/
    if (freq)
        set_ad9833_flag = true;
    else
        set_ad9833_flag = false;

    uint8_t freq_sfr = (uint8_t)atoi(argv[2]);
    if (freq_sfr > 1U)
    {
        AD9833_DEBUG_D("@error:wrong frequency register,please input'0/1'.\n");
        return;
    }
    uint8_t wave_type = (uint8_t)atoi(argv[3]);
    if (wave_type > 3U)
    {
        AD9833_DEBUG_D("@error:wrong wave type,please input'0:sin;1:tri;2:squ'.\n");
        return;
    }
    uint16_t phase = (uint16_t)atoi(argv[4]);
    if (phase > 360U)
    {
        AD9833_DEBUG_D("@error:wrong phase,please input'0-360'.\n");
        return;
    }
    ad9833_phase_regs phase_sfr = (ad9833_phase_regs)atoi(argv[5]);
    if (phase_sfr > phase_reg1)
    {
        AD9833_DEBUG_D("@error:wrong phase register,please input'0/1'.\n");
        return;
    }
    uint16_t range = (uint16_t)atoi(argv[6]);
    if (range > 255U)
    {
        AD9833_DEBUG_D("@error:wrong range,please input'0-255'.\n");
        return;
    }

    AD9833_DEBUG_D("\t\t\t\t[----ad9833_info----]\nfre\t\tsfr\twave\tphase\tphase_sfr\trange\n");
    AD9833_DEBUG_D("------\t\t---\t----\t-----\t-----\n");
    AD9833_DEBUG_D("%.3f\t\t%#x\t%d\t%d\t%d\t%d\n", freq, freq_sfr, wave_type, phase, phase_sfr, range);

    ad9833_set_wave(&ad9833_object, freq, freq_sfr, (ad9833_wave)wave_type, phase, phase_sfr);
    ad9833_amplitude(&ad9833_object, range);
    AD9833_DEBUG_R("data:%#x\n", range | 0x1100);
}
MSH_CMD_EXPORT(set_ad9833, operate sample
               : set_ad9833(freq) | (f_sfr) | (wave sin / tri / squ) | (phase) | (p_sfr) | (range) >);

/**
 * @brief  ad983设置相位寄存器
 * @note   寄存器0/1
 * @param  pa ad9833句柄
 * @param  data 12bit数据
 * @retval None
 */
ad9833_coding_table ad9833_check_wave_param(ad9833_out_t *pad)
{
#define AD9833_OUT_MAX_HZ 12000000.0F
#define AD9833_MAX_PHASE 360U
#define AD9833_MAX_RANGE 255U
#define AD9833_MAX_REG_NUM 1U
#define AD9833_MAX_WAVE_NUM 3U

    if (pad == NULL)
        return ad9833_is_null;
    if (pad->frequency > AD9833_OUT_MAX_HZ)
        return ad9833_fre_err;

    if (pad->fre_sfr > AD9833_MAX_REG_NUM)
        return ad9833_fre_sfr_err;
    if (pad->phase > AD9833_MAX_PHASE)
        return ad9833_phase_err;
    if (pad->phase_sfr > AD9833_MAX_REG_NUM)
        return ad9833_phase_sfr_err;
    if (pad->range > AD9833_MAX_RANGE)
        return ad9833_range_err;
    if (pad->wave_mode > AD9833_MAX_WAVE_NUM)
        return ad9833_wave_type_err;

    return ad9833_ok;

#undef AD9833_OUT_MAX_HZ
#undef AD9833_MAX_PHASE
#undef AD9833_MAX_REG_NUM
#undef AD9833_MAX_RANGE
#undef AD9833_MAX_WAVE_NUM
}

/**
 * @brief  ad9833输出目标波形
 * @note
 * @param None
 * @retval None
 */
void ad9833_out_target_wave(void)
{
    ad9833_t *pa = &ad9833_object;
    pModbusHandle pd = Modbus_Object;
    ad9833_out_t output_wave, *pad = &output_wave;

    if ((pa == NULL) || (pd == NULL))
        return;
    /*读出保持寄存器*/
    if (!pd->Mod_Operatex(pd, HoldRegister, Read, AD9833_PARAM_START_ADDR,
                          (uint8_t *)pad, sizeof(output_wave)))
    {
#if (USING_AD9833_DEBUG)
        AD9833_DEBUG_D("@error:Hold register read failed!\r\n");
#endif
    }
    ad9833_coding_table result = ad9833_check_wave_param(pad);

    if (!set_ad9833_flag && (result == ad9833_ok))
    {
        ad9833_set_wave(pa, pad->frequency, (uint8_t)pad->fre_sfr, pad->wave_mode,
                        pad->phase, (ad9833_phase_regs)pad->phase_sfr);
        ad9833_amplitude(pa, pad->range);
    }
}

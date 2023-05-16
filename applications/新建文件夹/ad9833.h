/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-09-15     LHC       the first version
 */
#ifndef _AD9833_H_
#define _AD9833_H_
#ifdef __cplusplus
extern "C"
{
#endif
#include "tool.h"

/*ad9833的28bit最大计数值:pow(2,28)*/
#define AD9833_REG_MAX_VAL 268435456.0F
/*ad9833晶振频率:默认25MHZ*/
#define AD9833_SOC_FREQ 25000000.0F
/*复位ad9833,即reset位为1*/
#define AD9833_RESET_CMD 0x0100
/*选择数据一次写入，b28位和reset位为1*/
#define AD9833_WRITE_ONECE_CMD 0x2100
/*输出正弦波命令*/
#define AD9833_SIN_WAVE_CMD 0x2000
/*输出三角波命令*/
#define AD9833_TRI_WAVE_CMD 0x2002
/*输出方波命令*/
#define AD9833_SQU_WAVE_CMD 0x2028
#define AD9833_PARAM_START_ADDR 0x00

    typedef enum
    {
        ad9833_ok = 0,        /*全部正确*/
        ad9833_is_null,       /*使用空指针*/
        ad9833_fre_err,       /*频率错误*/
        ad9833_fre_sfr_err,   /*频率寄存器错误*/
        ad9833_wave_type_err, /*波形类型错误*/
        ad9833_phase_err,     /*相位错误*/
        ad9833_phase_sfr_err, /*相位寄存器错误*/
        ad9833_range_err,     /*幅度错误*/
    } ad9833_coding_table;

    typedef enum
    {
        ad9833_sin = 0,
        ad9833_squ,
        ad9833_tri,
        ad9833_null_signal = 0xFFFFF,
    } ad9833_wave;

    typedef enum
    {
        phase_reg0 = 0,
        phase_reg1,
    } ad9833_phase_regs;

    typedef struct
    {
        void *pspi;
        Gpiox_info cs, scyn;
        void (*set_seampling_freq)(float);
    } ad9833_t;

    typedef struct
    {
        float frequency;
        uint16_t fre_sfr;
        uint16_t phase;
        uint16_t phase_sfr;
        uint16_t range;
        ad9833_wave wave_mode;
    } ad9833_out_t;

    extern ad9833_t ad9833_object;
    extern void ad9833_set_wave(ad9833_t *pa,
                                float freq,
                                unsigned char freq_sfr,
                                ad9833_wave wave,
                                unsigned short phase,
                                ad9833_phase_regs phase_sfr);
    extern void ad9833_amplitude(ad9833_t *pa,
                                 unsigned short data);
    extern void ad9833_out_target_wave(void);

#ifdef __cplusplus
}
#endif
#endif /* _AD9833_H_ */

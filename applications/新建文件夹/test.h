/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-09-15     LHC       the first version
 */
#ifndef _TEST_H_
#define _TEST_H_
#ifdef __cplusplus
extern "C"
{
#endif
    // <<< Use Configuration Wizard in Context Menu >>>

    // <<< end of configuration section >>>

#include "ad9833.h"
#include "relay.h"
#include "tool.h"

#define TEST_MAX_NUM 7U // 测试最大段数
#define Get_Error(__std, __test) \
    (!(__std) ? 0 : (fabsf((__test) - (__std)) / (__std)) * 100.0F)

    enum system_flag_group
    {
        test_developer_signal = 0, // 开发者模式信号
        test_start_signal,         // 启动测试信号
        test_filed_signal,         // 测试失败信号
        test_finsh_signal,         // 测试完成信号
        test_overcurrent_signal,   // 过流信号
        test_zero_crossing_signal, // 过零信号
        test_first_flag,           // 首次开始标志
        test_mode,                 // 测试模式:bit0 0为自动，1为手动
        test_at_flag,              // at自由指令模式标志
        // test_order_flag,           // 测试组的顺序标志：0正序，1逆序
        test_max_flag,
    };
    enum test_stage
    {
        test_standby = 0, // 待机
        test_start,       // 测试开始
        test_under,       // 测试中
        test_auto,        // 自动测试
        test_manual,      // 手动测试
        test_developer,   // 开发者模式
        test_dc,          // dc测试
        test_ac,          // ac测试
        test_filed,       // 测试失败
        test_finish,      // 测试完成
        test_overcurrent, // 过流
    };
    enum wave_freq
    {
        low_freq,
        medium_freq,
        high_freq,
        max_freq,
        null_freq = 0xFFFF, // 适应迪文地址
    };
#define FREQ_MAX_NUM (max_freq + 1U)
    /*定时器组*/
    typedef enum
    {
        time_id_power,   /*稳定电源定时器*/
        time_id_output,  /*稳定输出定时器*/
        time_id_relay,   /*继电器操作定时器*/
        tim_id_sampling, /*采样定时器*/
        tim_id_invalid,  /*进入开发者模式超时定时器*/
        tim_id_max,
    } test_timer;
#define TEST_SOFT_TIMER_NUM tim_id_max // 测试组软件定时器数量
    typedef struct
    {
        float voltage1;       // 电压1
        float voltage2;       // 电压2
        float current;        // 电流
        float voltage_offset; // 电压偏差率
        float current_offset; // 电流偏差率
    } test_data_t;

    typedef struct
    {
        unsigned int flag;     /*定时器的溢出标志*/
        unsigned int set_flag; /*定时器被设置标志：用设置了定时器时间但是时间未到*/
        unsigned int count;    /*时间计数器*/
    } test_timer_t;

    typedef struct
    {
        char *psend;
        char *precv;
        void (*event)(void *resp, void *other);
    } at_cmd;
    typedef at_cmd *at_cmd_t;

    enum at_state
    {
        at_exe_failed = -1,
        at_standy,
        at_enter_transparent,
        at_exit_transparent,
        at_continue,
        at_complete,
        at_cli,
    };
    enum at_cmd_id
    {
        at_noll_id = -1,
        at_enter_cmd1,
        at_enter_cmd2,
        at_z,
        at_entm,
        at_uart,
        at_ntp_time,
        at_max_id,
    };

    typedef struct test_handletypedef test_t;
    typedef struct test_handletypedef *ptest_t;
    struct test_handletypedef
    {
        uint32_t flag;          // 系统标志位
        enum at_state at_state; // wifi模块at临时指令模式
        enum at_cmd_id at_id;
        // relay_power_type cur_power;     //电源类型
        pre_handle pre;            // 继电器操作组指针
        enum test_stage cur_stage; // 系统当前测试阶段
        uint16_t auto_count;       // 自动模式执行计数
        // relay_tactics_type cur_tactics;          // 当前策略
        uint16_t test_result;                    // 测试结果
        test_timer_t timer[TEST_SOFT_TIMER_NUM]; // 软件定时器组

        float freq_table[FREQ_MAX_NUM]; // 频率表
        enum wave_freq user_freq;       // 用户波形
        struct
        {
            float output_voltage; // 理论输出电压值
        } dc;                     // 系统直流测试模式
        struct
        {
            ad9833_out_t wave_param; // 波形参数
            // uint16_t *pnext_fre;              // 自动模式下一测试频率
        } ac;
        /*手动模式测试组*/
        relay_group cur_group;
        // 系统交流测试模式
        struct
        {
            float voltage_offset, current_offset; // 电压偏差率,电流偏差率
        } comm_param;

        struct
        {
            uint16_t over_current; // 过流动画
            // uint8_t retain;       // 保留字段
        } cartoon;
        // uint16_t cur_exe_count; // 记录测试执行的次数
        struct
        {
            uint16_t size;
            test_data_t *p;
        } data;
        struct
        {
            uint32_t csv_line_count; // csv当前写入行数记录
            uint32_t cur_size;       // 记录用户当前数据写入尺寸
        } file;
        void *pmodbus;
    };

    typedef struct test_eventhandletypedef test_event_t;
    typedef struct test_eventhandletypedef *ptest_event_t;
    struct test_eventhandletypedef
    {
        enum test_stage stage;
        void (*test_event)(ptest_t);
    };
    extern test_t test_object;
    extern test_timer_t *get_soft_timer_handle(test_timer_t *pe, test_timer id);
    extern void test_poll(void);
    extern void test_data_handle(test_t *pt);

    // enum test_data_type
    // {
    //     test_string,
    //     test_bool,
    //     test_uint8,
    //     test_int16,
    //     test_uint16,
    //     test_float,
    //     test_uint32,
    //     test_long,
    // };

    typedef union
    {
        char string[16];
        bool b1;
        uint8_t u8;
        int16_t i16;
        uint16_t u16;
        float f32;
        uint32_t u32;
        long l32;
    } union_data_t;
    struct ini_data_t
    {
        char section_name[16];
        char key_name[24];
        // union
        // {
        //     char string[16];
        //     bool b1;
        //     uint8_t u8;
        //     int16_t i16;
        //     uint16_t u16;
        //     float f32;
        //     uint32_t u32;
        //     long l32;
        // } def_val;
        union_data_t def_val;
        // void *user_data;
        // enum test_data_type data_type;
        // comm_val_t *pval;
        unsigned short index;
    };

#define __INIT_INI_STR_VAL(_sn, _kn, _dv, _data, _type)                                                     \
    {                                                                                                       \
        .section_name = _sn, .key_name = _kn, .def_val.string = _dv, .user_data = _data, .data_type = _type \
    }

#define __INIT_INI_BOOL_VAL(_sn, _kn, _dv, _data, _type)                                                \
    {                                                                                                   \
        .section_name = _sn, .key_name = _kn, .def_val.b1 = _dv, .user_data = _data, .data_type = _type \
    }

#define __INIT_INI_UINT8_VAL(_sn, _kn, _dv, _data, _type)                                               \
    {                                                                                                   \
        .section_name = _sn, .key_name = _kn, .def_val.u8 = _dv, .user_data = _data, .data_type = _type \
    }

#define __INIT_INI_UINT16_VAL(_sn, _kn, _dv, _data, _type)                                               \
    {                                                                                                    \
        .section_name = _sn, .key_name = _kn, .def_val.u16 = _dv, .user_data = _data, .data_type = _type \
    }

#define __INIT_INI_UINT32_VAL(_sn, _kn, _dv, _data, _type)                                               \
    {                                                                                                    \
        .section_name = _sn, .key_name = _kn, .def_val.u32 = _dv, .user_data = _data, .data_type = _type \
    }

#define __INIT_INI_FLOAT_VAL(_sn, _kn, _dv, _data, _type)                                                \
    {                                                                                                    \
        .section_name = _sn, .key_name = _kn, .def_val.f32 = _dv, .user_data = _data, .data_type = _type \
    }

#define __INIT_INI_LONG_VAL(_sn, _kn, _dv, _data, _type)                                                 \
    {                                                                                                    \
        .section_name = _sn, .key_name = _kn, .def_val.l32 = _dv, .user_data = _data, .data_type = _type \
    }

#define __INIT_INI_VAL(_sn, _kn, _v_name, _dv, _in)                   \
    {                                                                 \
        .section_name = _sn, .key_name = _kn, .def_val._v_name = _dv, \
        .index = _in                                                  \
    }
#ifdef __cplusplus
}
#endif
#endif /* _TEST_H_ */

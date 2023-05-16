/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-09-15     LHC       the first version
 */
#ifndef _RELAY_H_
#define _RELAY_H_
#ifdef __cplusplus
extern "C"
{
#endif
/*继电器动作最大组数*/
#define ACTION_MAX_GROUP_NUM 7U
/*继电器动作的下一次时间*/
#define RELAY_ACTION_NEXT_TIMES 5U // 000
    typedef enum
    {
        re_ok = 0,         /*无错误*/
        re_p_is_null,      /*空指针*/
        re_coil_over,      /*继电器池越界*/
        re_power_err,      /*电源类型错误*/
        re_act_err,        /*继电器操作类型错误*/
        re_praram_err,     /*继电器api接口参数错误*/
        re_find_null,      /*查找对象为空*/
        re_call_err,       /*函数调用返回错误*/
        re_unknown_state,  /*未知状态*/
        re_param_size_err, /*参数尺寸错误*/
        re_wait_continue,  /*继电器等待下一阶段*/
        re_other_err,      /*其他错误*/
    } relay_error_code;

    typedef enum
    {
        relay_close = 0,
        relay_open,
    } relay_state;
    typedef struct relay_handletypedef relay_handle;
    typedef struct relay_handletypedef *pre_handle;
    typedef enum
    {
        pos_sque_action = 0, /*正序动作*/
        rev_ord_action,      /*逆序动作*/
        pos_and_rev_action,  /*正序逆序交替进行*/
        custom_action,       /*自定义动作*/
        null_action,         /*不动作*/
    } relay_tactics_type;
    typedef enum
    {
        dc_out = 0, /*直流输出*/
        ac1_out,    /*交流输出:30V@50Hz*/
        ac2_out,    /*交流输出:xV@xHz*/
        null_out,
    } relay_power_type;

    typedef struct
    {
        unsigned short start;
        unsigned short end;
    } relay_group;
    typedef struct
    {
        relay_tactics_type action_type;
        void (*relay_operate)(pre_handle, relay_tactics_type);
    } relay_action_group;
    struct relay_handletypedef
    {
// unsigned int single_flag; // 标记一些参数单次设置
#pragma push
#pragma anon_unions
        union
        {
            struct
            {
                unsigned int order : 1;         /*继电器执行的顺序标志*/
                unsigned int end_site : 3;      /*结束位置*/
                unsigned int cur_exe_count : 3; /*系统当前执行次数*/
                unsigned int reserve : 25;
            };
            unsigned int val;
        } info;
#pragma pop // https://www.cnblogs.com/linux-farmer/p/15250740.html
        relay_power_type cur_power;
        relay_action_group *pag;
        unsigned short action_group_size;
        struct
        {
            unsigned char *p;
            unsigned int size;
        } coil;
        unsigned int next_time; /*继电器动作的下一次时间*/
        struct
        {
            void *p;
            unsigned int num;
        } timer;

        // unsigned char cur_group; /*自定义时使用*/
        relay_group cur_group;
        // unsigned int cur_exe_count; // 系统当前执行次数

        // void *pmodbus;
        void (*relay_delay)(unsigned int);
        // void (*relay_set_coil_interface)
        // relay_error_code (*relay_callback)(pre_handle, unsigned char);
        relay_error_code (*relay_callback)(pre_handle);
    };

    extern relay_handle relay_object;
    extern void relay_poll(pre_handle pre, relay_tactics_type cur_type);

#ifdef __cplusplus
}
#endif
#endif /* _RELAY_H_ */

/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-05-23     LHC       the first version
 */
#ifndef _INC_USER_WATER_ROOM_H_
#define _INC_USER_WATER_ROOM_H_
#ifdef __cplusplus
extern "C"
{
#endif
#include "lhc_tool.h"

//棚内c02监测的最大路数
#define POTO_CO2_MAX 8U
//百叶盒数据输入开始地址
#define LOUVER_BOX_START_ADDR 0x100

    enum water_valve
    {
        water_A0,
        water_A1,
        water_B0,
        water_B1,
        water_q4,
        water_qx_max,
    };

    enum water_f_args
    {
        /*Sensor range*/
        pipe_pre_upper,     // Upper limit of pipeline pressure
        pipe_pre_lower,     // Lower limit of pipeline pressure
        pipe_flow_upper,    // Upper limit of Pipeline flow rate
        pipe_flow_lower,    // Lower limit of Pipeline flow rate
        vat_level_upper,    // Upper limit of Fertilizer tank liquid level
        vat_level_lower,    // Lower limit of Fertilizer tank liquid level
        fer_EC_upper,       // Upper limit of Fertilizer EC
        fer_EC_lower,       // Lower limit of Fertilizer EC
        fer_PH_upper,       // Upper limit of Fertilizer PH
        fer_PH_lower,       // Lower limit of Fertilizer PH
        /*Sensor allowable adjustment range*/
        pipe_pre_max,       //Upper limit of pipeline pressure bearing
        pipe_pre_min,       //lower limit of pipeline pressure bearing
        pipe_flow_max,      //Upper limit of pipeline flow rate bearing
        pipe_flow_min,      //lower limit of pipeline flow rate bearing
        vat_level_max,      //Upper limit of Fertilizer tank liquid level bearing
        vat_level_min,      //lower limit of Fertilizer tank liquid level bearing
        fer_EC_max,         //Upper limit of Fertilizer EC bearing
        fer_EC_min,         //lower limit of Fertilizer EC bearing
        fer_PH_max,         //Upper limit of Fertilizer PH bearing
        fer_PH_min,         //lower limit of Fertilizer PH bearing
        water_f_args_max,
    };

    enum water_u_args
    {
        fer_A0,
        fer_Amax,
        fer_B0,
        fer_Bmax,
        water_u_args_max,
    };

    enum water_f_vals
    {
        pipe_pre,           //Pipeline pressure
        pipe_flow,          //Pipeline flow rate
        vat_level,          //Fertilizer tank liquid level
        fer_EC,             //Fertilizer EC
        fer_PH,             //Fertilizer EC
        water_f_vals_max,
    };

    enum water_u_vals
    {
        xu,
        water_u_vals_max,
    };

    struct water_save_arg
    {
        uint32_t start_time;     //Irrigation start time
        uint32_t end_time;       //Irrigation end time
        uint32_t cfg_time;       //Total cycle of fertilizer allocation
        uint16_t mode;           //irrigation pattern
        uint16_t count;          //irrigation count
        uint16_t fer_group;      //Fertilizer group: A or B
        struct
        {
            uint32_t mask : 10;  //Sensor error shielding flag: 0 enabled, 1 disabled
            uint32_t reserve : 22;
        }flag;
        
        float f[water_f_args_max];      // User parameters
        uint16_t u[water_u_args_max];   // User parameters
        struct resource_pool pool;
    };

    struct water_user
    {
        float f_val[water_f_vals_max]; // 用户变量
        uint16_t u_val[water_u_vals_max]; // 用户变量
    };

    struct water
    {
        struct water_save_arg arg;
        struct water_user user;
        uint32_t node_count;
        uint32_t each_time;
        struct
        {
            uint32_t debug : 1;     // 调试模式
            uint32_t code : 8;      // 系统编码：8位
            uint32_t mutex : 16;    // 系统运行时的一些滞后标志
            uint32_t next_node : 1; // 轮灌溉阀节点标志
            uint32_t reserve : 6;
        } flag;
        
        void *data;
    };

    extern struct water water_system; // 光合作用系统

#ifdef __cplusplus
}
#endif
#endif /* _INC_USER_WATER_ROOM_H_ */
	
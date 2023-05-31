/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-05-12     LHC          the first version
 */
#ifndef _INC_USER_CLIMATE_H_
#define _INC_USER_CLIMATE_H_
#ifdef __cplusplus
extern "C"
{
#endif
#include "lhc_tool.h"

    enum clim_f_args
    {
        house_out_wet_upper,          //棚外雨量上限
        house_out_wet_lower,          //棚外雨量下限

        house_in_heat_0_upper,        //0号棚内温度上限 -- 风扇
        house_in_heat_0_lower,        //0号棚内温度下限 -- 风扇

        clim_f_args_max,
    };

    enum clim_u_args
    {
        house_in_heat_00,             //0号棚内0段温度 -- 天窗
        house_in_heat_01,             //0号棚内1段温度 -- 天窗
        house_in_heat_02,             //0号棚内2段温度 -- 天窗
        house_in_heat_03,             //0号棚内3段温度 -- 天窗
        house_in_heat_0max,           //0号棚内4段温度 -- 天窗
        house_in_humidity_00,         //0号棚内0段湿度 -- 侧窗
        house_in_humidity_01,         //0号棚内1段湿度 -- 侧窗
        house_in_humidity_02,         //0号棚内2段湿度 -- 侧窗
        house_in_humidity_03,         //0号棚内3段湿度 -- 侧窗
        house_in_humidity_0max,       //0号棚内4段湿度 -- 侧窗
        house_air_window_crack_00,    //0号棚内0段天窗开合度
        house_air_window_crack_01,    //0号棚内1段天窗开合度
        house_air_window_crack_02,    //0号棚内2段天窗开合度
        house_air_window_crack_03,    //0号棚内3段天窗开合度
        house_air_window_crack_0max,  //0号棚内4段天窗开合度
        house_side_window_crack_00,   //0号棚内0段侧窗开合度
        house_side_window_crack_01,   //0号棚内1段侧窗开合度
        house_side_window_crack_02,   //0号棚内2段侧窗开合度
        house_side_window_crack_03,   //0号棚内3段侧窗开合度
        house_side_window_crack_0max,   //0号棚内4段侧窗开合度
        clim_u_args_max,
    };

    enum climate_f_vals
    {
        house_out_wet,           // 棚外雨量
        house_in_heat_0,         // 0号棚内温度
        // house_in_heat_max,       // 造成一个单元空间浪费，但便于扩展新变量
        house_in_humidity_0,     // 0号棚内湿度
        // house_in_humidity_max,

        clim_f_vals_max,
    };

    enum clim_u_vals
    {
		clim_val1,
        clim_u_vals_max,
    };

    struct clim_save_arg
    {
        // uint32_t start_time;
        // uint32_t end_time;
        uint32_t group_num;  // 风扇和窗的组数
        uint16_t air_windos_time;   //天窗总开/合时间
        uint16_t side_windos_time;  //侧窗总开/合时间
        uint16_t default_air_windos_crack;     //雨量较小时的一个默认开合度
        uint16_t default_side_windos_crack;    //雨量较小时的一个默认开合度
        struct
        {
            // uint32_t signal : 1; // 信号来源：1内部；0外部
            // uint32_t tank : 1;   // 储槽压力：1启用次位
            // uint32_t vap : 1;    // 汽化器出口压力：1启用次位
            // uint32_t gas : 1;    // 气站出口压力：1启用次位
            // uint32_t flow : 1;   // 流量计：1启用次位
            // uint32_t co2 : 8;    // co2检测量：256个棚
            // uint32_t pot : 1;    // 液体储罐类型：0卧式，1立式

            uint32_t mask : 24;  // 传感器错误屏蔽标志：0启用，1关闭
            uint32_t reserve : 8;
        } flag;

        float f[clim_f_args_max];    // 用户参数
        uint16_t u[clim_u_args_max]; // 用户参数
        struct resource_pool pool;
    };

    struct clim_user
    {
        float f_val[clim_f_vals_max]; // 用户变量
        uint16_t u_val[clim_u_vals_max]; // 用户变量
    };

    struct climate
    {
        struct clim_save_arg arg;
        struct clim_user user;
        struct
        {
            uint32_t debug : 1;  // 调试模式（手动/自动）
            uint32_t code : 8;   // 系统编码：8位
            uint32_t mutex : 16; // 系统运行时的一些滞后标志
            uint32_t reserve : 7;
        } flag;
        void *data;
    };

    extern struct climate clim_system; // 气候调节系统

#ifdef __cplusplus
}
#endif
#endif /* _INC_USER_CLIMATE_H_ */
	
	
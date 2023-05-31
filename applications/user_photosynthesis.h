/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-05-23     LHC       the first version
 */
#ifndef _INC_USER_PHOTOSYNTHESIS_H_
#define _INC_USER_PHOTOSYNTHESIS_H_
#ifdef __cplusplus
extern "C"
{
#endif
#include "lhc_tool.h"


//棚内c02监测的最大路数
#define POTO_CO2_MAX 8U
//百叶盒数据输入开始地址
#define LOUVER_BOX_START_ADDR 0x100

    enum poto_valve
    {
        poto_q0,
        poto_q1,
        poto_q2,
        poto_q3,
        poto_q4,
        poto_qx_max,
    };

    enum poto_f_args
    {
        // poto_args_min = -1,
        tank_mp_upper,     // 主储槽压力上限
        tank_mp_lower,     // 主储槽压力下限
        tank_sp_upper,     // 副储槽压力上限
        tank_sp_mlower,    // 副储槽压力下限
        vap_mp_mupper,     // 主汽化器出口压力上限
        vap_mp_lower,      // 主汽化器出口压力下限
        vap_sp_mupper,     // 副汽化器出口压力上限
        vap_sp_lower,      // 副汽化器出口压力下限
        gas_mp_upper,      // 主气站出口压力上限
        gas_mp_lower,      // 主气站出口压力下限
        gas_sp_upper,      // 副气站出口压力上限
        gas_sp_lower,      // 副气站出口压力下限
        tank_l_upper,      // 储槽液位高度上限
        tank_l_lower,      // 储槽液位高度下限
        flow_mp_upper,     // 主流量计量程上限
        flow_mp_lower,     // 主流量计量程下限
        flow_sp_upper,     // 次流量计量程上限
        flow_sp_lower,     // 次流量计量程下限
        gas_heat_limit,    // 汽化器温度临界值
        s_tank_supplement, // 启动模式：储槽补压启动值
        /*Threshold 2*/
        s_spf_start,     // 启动模式：储槽泄压启动值
        s_spf_stop,      // 启动模式：储槽泄压停止值
        s_vap_start,     // 启动模式：汽化器补压启动值
        s_vap_stop,      // 启动模式：汽化器补压停止值
        vap_back_limit,  // 汽化器回压阈值
        tank_back_limit, // 储槽回压阈值
        p_vap_start,     // 停机模式：汽化器泄压启动值
        p_vap_stop,      // 停机模式：汽化器泄压停止值
        p_spf_start,     // 停机模式储槽泄压启动值
        p_spf_stop,      // 停机模式储槽泄压停止值
        tank_p_limit,    // 安全策略：储槽压力阈值
        tank_i_limit,    // 安全策略：储槽液位阈值
        /*卧式储罐*/
        tank_h,          // 储罐参数：储槽封头内高度
        tank_r,          // 储罐参数：储槽圆柱半径
        tank_l,          // 储罐参数：储槽圆体长度
        tank_d,          // 储槽参数：储槽液体密度
        /*立式储罐*/
   
        poto_f_args_max,
    };

    enum poto_u_args
    {
        co2_1_upper,    //1#co2上限
        co2_1_lower,    //1#co2下限
        co2_2_upper,    //2#co2上限
        co2_2_lower,    //2#co2下限
        co2_3_upper,    //3#co2上限
        co2_3_lower,    //3#co2下限
        co2_4_upper,    //4#co2上限
        co2_4_lower,    //4#co2下限
        co2_5_upper,    //5#co2上限
        co2_5_lower,    //5#co2下限
        co2_6_upper,    //6#co2上限
        co2_6_lower,    //6#co2下限
        co2_7_upper,    //7#co2上限
        co2_7_lower,    //7#co2下限
        co2_8_upper,    //8#co2上限
        co2_8_lower,    //8#co2下限
        // co2_9_upper,    //9#co2上限
        // co2_9_lower,    //9#co2下限
        poto_u_args_max,
    };

    enum poto_f_vals
    {
        m_tank_pre, //主储槽压力
        s_tank_pre, //次储槽压力
        m_vap_pre,  //主汽化器压力
        s_vap_pre,  //副汽化器压力
        m_gas_pre,  //主气站压力
        s_gas_pre,  //副气站压力
        m_flow,     //主流量计
        s_flow,     //副流量计
        tank_level, //储槽液位
        gas_heat,   //汽化器温度
        
        poto_f_vals_max,
    };

    enum poto_u_vals
    {
        co2_out,    //室外co2浓度
        co2_1,      //1号大棚co2浓度
        co2_2,      //2号大棚co2浓度
        co2_3,      //3号大棚co2浓度
        co2_4,      //4号大棚co2浓度
        co2_5,      //5号大棚co2浓度
        co2_6,      //6号大棚co2浓度
        co2_7,      //7号大棚co2浓度
        co2_8,      //8号大棚co2浓度
        poto_u_vals_max,
    };

    // #define POTOS_USER_PARAM_MAX poto_args_max

    struct potos_save_arg
    {
        uint32_t start_time;
        uint32_t end_time;
        uint32_t support_max; //最大支持的用户棚数
        struct
        {
            uint32_t signal : 1; // 信号来源：1内部；0外部
            uint32_t tank : 1;   //储槽压力：1启用次位
            uint32_t vap : 1;    //汽化器出口压力：1启用次位
            uint32_t gas : 1;    //气站出口压力：1启用次位
            uint32_t flow : 1;   //流量计：1启用次位
            uint32_t co2 : 8;    //co2检测量：256个棚
            uint32_t pot : 1;    //液体储罐类型：0卧式，1立式
            uint32_t mask : 10;  //传感器错误屏蔽标志：0启用，1关闭
            uint32_t reserve : 8;
        }flag;
        
        float f[poto_f_args_max]; // 用户参数
        uint16_t u[poto_u_args_max]; // 用户参数
        struct resource_pool pool;
    };

    struct potos_user
    {
        float f_val[poto_f_vals_max]; // 用户变量
        uint16_t u_val[poto_u_vals_max]; // 用户变量
    };

    struct potos
    {
        struct potos_save_arg arg;
        struct potos_user user;
        struct
        {
            uint32_t debug : 1; // 调试模式
            uint32_t code : 8;  // 系统编码：8位
            uint32_t mutex : 16; // 系统运行时的一些滞后标志
            uint32_t reserve : 7;
        } flag;
        void *data;
    };

    extern struct potos poto_system; // 光合作用系统

#ifdef __cplusplus
}
#endif
#endif /* _INC_USER_PHOTOSYNTHESIS_H_ */

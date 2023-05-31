#include "user_climate.h"
#include "lhc_modbus_port.h"
#include <rtdevice.h>
#include <qled.h>

/*
* note : 使用quick led组件时，目标pin必须带上 INDIRC_COIL 标识
* keil中char类型特殊说明：https://mp.weixin.qq.com/s/grf8MBXUj7amMu6JCPGaFQ
*/

struct climate clim_system; // 气候调节系统
static uint8_t clim_coil[8U];
static uint16_t clim_air_window_last_state[(house_in_heat_0max - house_in_heat_00) + 1];
static uint16_t clim_side_window_last_state[(house_in_humidity_0max - house_in_humidity_00) + 1];

static char *clim_air_pin_name[] = {
    "PB.12",
    "PE.15",
};

static char *clim_side_pin_name[] = {
    "PE.14",
    "PE.13",
};

/**
 * @brief	气候系统获取内部变量
 * @details
 * @param	None
 * @retval  None
 */
static void clim_get_vals(struct climate *clim,
                          pModbusHandle pd)
{
    float *data, *arg;

    //数据的分步获取：雨量片区域或者一个棚一个，温度和湿度间隔一定地址
    pd->Mod_Operatex(pd, InputRegister, lhc_modbus_read, clim->arg.pool.ai_des.base_addr, //offset
                     (uint8_t *)clim->user.f_val, sizeof(clim->user.f_val));

    for (data = clim->user.f_val, arg = clim->arg.f;
         data < clim->user.f_val + clim_f_vals_max;
         ++data, arg += 2U)
    {
        *data = Get_Target(*data, *arg, *(arg + 1U));

        if (*data < 0.0f)
            *data = 0;
    }
}

/**
 * @brief	气候系统检测错误
 * @details
 * @param	None
 * @retval  0系统无错误 1系统存在错误
 */
static uint8_t clim_check_error(struct climate *clim)
{
#define CLIM_ERROR_BASE_CODE 0x02
    uint8_t site;
    float *data = clim->user.f_val;

    for (site = 0; site < clim_f_vals_max; ++site, ++data)
    {
        if (__GET_FLAG(clim->arg.flag.mask, site)) //检查目标传感器是否被屏蔽
            continue;
        clim->flag.code = *data <= 0.0F ? (3U * site + CLIM_ERROR_BASE_CODE)
                                        : (*data < (CURRENT_LOWER - 0.5F) /*Disconnection detection offset 0.5mA*/
                                               ? (3U * site + CLIM_ERROR_BASE_CODE + 1U)
                                               : (*data > (CURRENT_LOWER + CURRENT_UPPER + 1.0F)
                                                      ? (3U * site + CLIM_ERROR_BASE_CODE + 2U)
                                                      : 0));
        if (clim->flag.code) //按顺序报告错误码
            return 1;
    }
#undef CLIM_ERROR_BASE_CODE
    return 0;
}


/**
 * @brief	气候窗初始化
 * @details
 * @param	None
 * @retval  none
 */
static void clim_windows_init(void)
{
    rt_int32_t pin_num;
    uint8_t i;
    
    for (i = 0; i < sizeof(clim_air_pin_name) / sizeof(clim_air_pin_name[0]); ++i)
    {
        pin_num = rt_pin_get(clim_air_pin_name[i]);
        qled_add(pin_num, GPIO_PIN_SET);
    }

    for (i = 0; i < sizeof(clim_side_pin_name) / sizeof(clim_side_pin_name[0]); ++i)
    {
        pin_num = rt_pin_get(clim_side_pin_name[i]);
        qled_add(pin_num, GPIO_PIN_SET);
    }
}

/**
 * @brief	气候窗移除
 * @details
 * @param	None
 * @retval  none
 */
static void clim_windows_remove(void)
{
    rt_int32_t pin_num;
    uint8_t i;
    
    for (i = 0; i < sizeof(clim_air_pin_name) / sizeof(clim_air_pin_name[0]); ++i)
    {
        pin_num = rt_pin_get(clim_air_pin_name[i]);
        qled_remove(pin_num);
    }

    for (i = 0; i < sizeof(clim_side_pin_name) / sizeof(clim_side_pin_name[0]); ++i)
    {
        pin_num = rt_pin_get(clim_side_pin_name[i]);
        qled_remove(pin_num);
    }
}

/**
 * @brief	气候窗关闭
 * @details
 * @param	None
 * @retval  none
 */
static void clim_windows_off(void)
{
    rt_int32_t pin_num;
    uint8_t i;
    
    for (i = 0; i < sizeof(clim_air_pin_name) / sizeof(clim_air_pin_name[0]); ++i)
    {
        pin_num = rt_pin_get(clim_air_pin_name[i]);
        qled_set_off(pin_num);
    }

    for (i = 0; i < sizeof(clim_side_pin_name) / sizeof(clim_side_pin_name[0]); ++i)
    {
        pin_num = rt_pin_get(clim_side_pin_name[i]);
        qled_set_off(pin_num);
    }
}

/**
 * @brief	xx统检查状态是否发生改变
 * @details
 * @param	cur   current state
 * @param   last  Last Status
 * @retval  0 / +,-xx
 */
int xx_system_state_check(int cur, int last)
{
    int state = cur - last;
    return (cur > last ? state : cur < last ? -state
                                           : 0);
}

/**
 * @brief	气候系统设置窗子属性
 * @details
 * @param	None
 * @retval  0系统无错误 1系统存在错误
 */
static void clim_set_window_nature(struct climate *clim,
                                   uint8_t val_start,
                                   uint8_t val_end,
                                   uint8_t arg_start,
                                   uint8_t arg_end,
                                   uint8_t crack_start,
                                   uint8_t crack_end,
                                   uint16_t *last_state,
                                   char **pin_name,
                                   uint16_t crack)
{
#define clim_get_site(_cur, _next, _max) ((_cur) + (_next) * ((_max) + 1U))
#define clim_val_gap(_next, _max) ((_next) * ((_max) + 1U))

    rt_int32_t pin_num;
    uint8_t i = 0, val_site;
    uint8_t gap, *pname;
    uint16_t rem_crack;
    int rem_state;
    uint16_t open_times;
    uint16_t window_crack;
    float *user_val = &clim->user.f_val[val_start];
    uint16_t *user_arg = &clim->arg.u[arg_start];
    // float per;

    for (val_site = val_start; val_site < val_end; ++val_site)
    {
        i = val_site - val_start;
        gap = clim_val_gap(i, (arg_end - arg_start));

        if (!crack)
        {
            window_crack =
                user_val[val_site] > user_arg[arg_start + gap]
                    ? user_arg[crack_start + gap]
                : user_val[val_site] > user_arg[arg_start + 1U + gap] ? user_arg[crack_start + 1U + gap]
                : user_val[val_site] > user_arg[arg_start + 2U + gap] ? user_arg[crack_start + 2U + gap]
                : user_val[val_site] > user_arg[arg_start + 3U + gap] ? user_arg[crack_start + 3U + gap]
                : user_val[val_site] > user_arg[arg_end + gap]        ? user_arg[crack_end + gap]
                                                                      : 0U;
        }
        else
            window_crack = crack;

        // if (window_crack > last_state[i])
        // {
        //     __RESET_FLAG(clim->flag.mutex, i);
        //     pname = pin_name[i * 2];
        //     rem_crack = window_crack - last_state[i];
        // }
        // else if (window_crack < last_state[i])
        // {
        //     __RESET_FLAG(clim->flag.mutex, i);
        //     pname = pin_name[1 + i * 2];
        //     rem_crack = last_state[i] - window_crack;
        // }
        // else
        //     __SET_FLAG(clim->flag.mutex, i);

        // /*数据传递到底层*/
        // if (!__GET_FLAG(clim->flag.mutex, i))
        // {
        //     pin_num = rt_pin_get(pname);
        //     per = (float)rem_crack / 100.0f;
        //     open_times = (uint16_t)(per * (float)clim->arg.air_windos_time);
        //     qled_set_blink(pin_num, open_times, clim->arg.air_windos_time - open_times);
        //     last_state[i] = window_crack;
        // }

        rem_state = xx_system_state_check(window_crack, last_state[i]);

        if (rem_state)
        {
            pname = rem_state > 0 ? pin_name[i * 2] : pin_name[1 + i * 2];
            pin_num = rt_pin_get(pname);
            rem_crack = abs(rem_state);
            open_times = (uint16_t)((float)rem_crack / 100.0f *
                                    (float)clim->arg.air_windos_time);
            qled_set_blink(pin_num, open_times, clim->arg.air_windos_time - open_times);
            last_state[i] = window_crack;
        }
    }

#undef clim_get_site
#undef clim_val_gap
}

/**
 * @brief	气候系统中天窗、侧窗控制
 * @details
 *           a、天窗控制基于棚内温度、侧窗控制基于棚内湿度，分别设置5个段，每段包含1个湿度值和1个具体的开合度。
 *            （温度大于30摄氏度时，天窗开合度为100%，湿度大于60时，侧窗开合度为100%）。
 *           b、天窗、侧窗控制还要基于棚外雨量传感器，雨大时，全部关闭，雨偏小时，看温度情况决定。
 *           c、风扇、天窗、侧窗开合都需要支持自动模式和手动模式。
 *           d、风扇、天窗、侧窗开合可能会影响到CO2浓度，导致CO2控制系统无法正常工作，大量气体浪费。
 * @param	None
 * @retval  none
 */
static int clim_windows_control(struct climate *clim, uint8_t *coil)
{

    // uint8_t i = 0, he, hu;
    // uint16_t rem_crack;
    // uint16_t open_times;
    // uint8_t gap, *pname;

    uint16_t air_window_crack = 0, side_window_crack = 0;

    if (RT_NULL == coil)
        return 0;

    //当前雨量较大，不允许开棚
    if (clim->user.f_val[house_out_wet] > clim->arg.f[house_out_wet_upper])
    {
        clim_windows_off();
        return 1;
    }
        
    //当前雨量较小，允许固定开合20%
    if (clim->user.f_val[house_out_wet] < clim->arg.f[house_out_wet_lower])
    {
        air_window_crack = clim->arg.default_air_windos_crack;
        side_window_crack = clim->arg.default_side_windos_crack;
    }
       
    //设置天窗开合度
    clim_set_window_nature(clim, house_in_heat_0, house_in_humidity_0, house_in_heat_00, house_in_heat_0max,
                           house_air_window_crack_00, house_air_window_crack_0max, clim_air_window_last_state,
                           clim_air_pin_name, air_window_crack);

    // for (he = house_in_heat_0, i = 0; he < house_in_humidity_0; ++he, ++i)
    // {
    //     gap = clim_val_gap(i, (house_in_heat_0max - house_in_heat_00));

    //     air_window_crack =
    //         clim->user.f_val[he] > clim->arg.u[house_in_heat_00 + gap]
    //             ? clim->arg.u[house_air_window_crack_00 + gap]
    //         : clim->user.f_val[he] > clim->arg.u[house_in_heat_01 + gap]   ? clim->arg.u[house_air_window_crack_01 + gap]
    //         : clim->user.f_val[he] > clim->arg.u[house_in_heat_02 + gap]   ? clim->arg.u[house_air_window_crack_02 + gap]
    //         : clim->user.f_val[he] > clim->arg.u[house_in_heat_03 + gap]   ? clim->arg.u[house_air_window_crack_03 + gap]
    //         : clim->user.f_val[he] > clim->arg.u[house_in_heat_0max + gap] ? clim->arg.u[house_air_window_crack_0max + gap]
    //                                                                        : 0U;
       
    //     //上述设置应用到底层硬件

    //     if (air_window_crack > clim_air_window_last_state[i])
    //     {
    //         __RESET_FLAG(clim->flag.mutex, (he - house_in_heat_0));
    //         pname = clim_pin_name[i * 2];
    //         rem_crack = air_window_crack - clim_air_window_last_state[i];
    //     }
    //     else if (air_window_crack < clim_air_window_last_state[i])
    //     {
    //         __RESET_FLAG(clim->flag.mutex, (he - house_in_heat_0));
    //         pname = clim_pin_name[1 + i * 2];
    //         rem_crack = clim_air_window_last_state[i] - air_window_crack;
    //     }
    //     else
    //         __SET_FLAG(clim->flag.mutex, (he - house_in_heat_0));
         
        
    //     if (!__GET_FLAG(clim->flag.mutex, (he - house_in_heat_0)))
    //     {
    //         pin_num = rt_pin_get(pname);
    //         per =  (float)rem_crack / 100.0f;
    //         open_times = (uint16_t)(per * (float)clim->arg.air_windos_time);
    //         qled_set_blink(pin_num, open_times, clim->arg.air_windos_time - open_times);
    //     }
            
    //     clim_air_window_last_state[i] = air_window_crack;
    // }

    // 设置侧窗开合度
    clim_set_window_nature(clim, house_in_humidity_0, clim_f_vals_max, house_in_humidity_01, house_in_humidity_0max,
                           house_side_window_crack_00, house_side_window_crack_0max, clim_side_window_last_state,
                           clim_side_pin_name, side_window_crack);

    // for (hu = house_in_humidity_0, i = 0; hu < clim_f_vals_max; ++hu, ++i)
    // {
    //     gap = clim_val_gap(i, (house_in_humidity_0max - house_in_humidity_00));

    //     side_window_crack =
    //         clim->user.f_val[hu] > clim->arg.u[house_in_humidity_00 + gap]
    //             ? clim->arg.u[house_side_window_crack_00 + gap]
    //         : clim->user.f_val[hu] > clim->arg.u[house_in_humidity_01 + gap]   ? clim->arg.u[house_side_window_crack_01 + gap]
    //         : clim->user.f_val[hu] > clim->arg.u[house_in_humidity_02 + gap]   ? clim->arg.u[house_side_window_crack_02 + gap]
    //         : clim->user.f_val[hu] > clim->arg.u[house_in_humidity_03 + gap]   ? clim->arg.u[house_side_window_crack_03 + gap]
    //         : clim->user.f_val[hu] > clim->arg.u[house_in_humidity_0max + gap] ? clim->arg.u[house_side_window_crack_0max + gap]
    //                                                                            : 0U;

    //     // 上述设置应用到底层硬件
    // }

    return 1;
}

/**
 * @brief	气候系统中风扇控制
 * @details
 * @param	None
 * @retval  none
 */
static int clim_fans_control(struct climate *clim, uint8_t *coil)
{
    uint8_t site, i = 0;

    if (RT_NULL == coil)
        return 0;

    for (site = house_in_heat_0; site < house_in_humidity_0; ++site, ++i)
    {
        // xx棚内温度高于当前xx温度上限，开风扇
        if(clim->user.f_val[site] > clim->arg.f[house_in_heat_0_upper + i])
            coil[i] = 1;

        // xx棚内温度低于当前xx温度下限，关风扇
        if(clim->user.f_val[site] < clim->arg.f[house_in_heat_0_lower + i])
            coil[i] = 0;
    }

    return 1;
}

/**
 * @brief	气候控制
 * @details
 * @param	None
 * @retval  none
 */
static void clim_control(void)
{
    struct climate *clim = &clim_system;
    pModbusHandle pd = share_lhc_modbus;
    static uint8_t debug_flag = 0;

    RT_ASSERT(pd);

    memset(clim_coil, DIRC_COIL, sizeof(clim_coil));

    if (clim->flag.debug) // 处于调试模式，不执行系统逻辑
    {
        debug_flag = 1;
        clim_windows_remove();
        return;
    }
        
    if (debug_flag)
    {
        clim_windows_init();
        debug_flag = 0;
    }

    clim_get_vals(clim, pd);    // 信号转换
    if (clim_check_error(clim)) // 错误检测
    {
        clim_windows_off();
        goto __no_action;
    }

    rt_memset(&clim_coil[0], INDIRC_COIL, 4U); //屏蔽pin的直接控制功能，启用quick led

    clim_fans_control(clim, &clim_coil[0]);
    clim_windows_control(clim, &clim_coil[house_in_heat_0max - house_in_heat_00]);

__no_action:
    pd->Mod_Operatex(pd, Coil, lhc_modbus_write, clim->arg.pool.do_des.base_addr,
                     clim_coil, clim->arg.pool.do_des.num);
}

/**
 * @brief   气候系统线程
 * @details
 * @param	parameter:线程初始参数
 * @retval  None
 */
void climate_thread_entry(void *parameter)
{
    clim_windows_init();
    for (;;)
    {
        clim_control();
        rt_thread_mdelay(1000);
    }
}

/**
 * @brief	气候控制系统初始化
 * @details
 * @param	None
 * @retval  none
 */
static int clim_init(void)
{
    rt_thread_t tid = rt_thread_create(
        "climate",
        climate_thread_entry,
        RT_NULL,
        1024, 0x11, 20);

    RT_ASSERT(tid != RT_NULL);

    rt_thread_startup(tid);

    rt_kprintf("climate system init.\r\n");

    return (RT_EOK);
}
// INIT_APP_EXPORT(clim_init);

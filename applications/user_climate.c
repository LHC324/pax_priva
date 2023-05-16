#include "user_climate.h"
#include "lhc_modbus_port.h"

struct climate clim_system; // 气候调节系统
static uint8_t clim_coil[8U];


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
            *data = 0.0f;
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
static char clim_windows_control(struct climate *clim, uint8_t *coil)
{
#define clim_get_site(_cur, _next, _max) ((_cur) + (_next) * ((_max) + 1U))
#define clim_val_gap(_next, _max) ((_next) * ((_max) + 1U))

    uint8_t i = 0, he, hu;
    uint8_t air_window_crack, side_window_crack;
    uint8_t gap;

    if (RT_NULL == coil)
        return 0;

    //当前雨量较大，不允许开棚
    if (clim->user.f_val[house_out_wet] > clim->arg.f[house_out_wet_upper])
        return 1;

    //当前雨量较小，允许固定开合20%
    if (clim->user.f_val[house_out_wet] < clim->arg.f[house_out_wet_lower])
    {
        air_window_crack = 20;
        side_window_crack = 20;
    }
       

    for (he = house_in_heat_0, hu = house_in_humidity_0;
         hu < clim_f_vals_max;
         ++he, ++hu, ++i)
    {
        // // xx棚内温度高于当前xx温度上限，开风扇
        // if (clim->user.f_val[he] > clim->arg.f[house_in_heat_00 + i * (house_in_heat_04 + 1U)])
        //     coil[i] = 1;

        // // xx棚内温度低于当前xx温度下限，关风扇
        // if (clim->user.f_val[site] < clim->arg.f[house_in_humidity_0_lower + i])
        //     coil[i] = 0;

        gap = clim_val_gap(i, (house_in_heat_0max - house_in_heat_00));

        //设置天窗开合度
        air_window_crack =
            clim->user.f_val[he] > clim->arg.u[house_in_heat_00 + gap]
                ? clim->arg.u[house_air_window_crack_00 + gap]
            : clim->user.f_val[he] > clim->arg.u[house_in_heat_01 + gap]   ? clim->arg.u[house_air_window_crack_01 + gap]
            : clim->user.f_val[he] > clim->arg.u[house_in_heat_02 + gap]   ? clim->arg.u[house_air_window_crack_02 + gap]
            : clim->user.f_val[he] > clim->arg.u[house_in_heat_03 + gap]   ? clim->arg.u[house_air_window_crack_03 + gap]
            : clim->user.f_val[he] > clim->arg.u[house_in_heat_0max + gap] ? clim->arg.u[house_air_window_crack_0max + gap]
                                                                           : 0U;
        // gap = clim_val_gap(i, house_in_heat_0max);

        // 设置侧窗开合度
        side_window_crack =
            clim->user.f_val[he] > clim->arg.u[house_in_humidity_00 + gap]
                ? clim->arg.u[house_side_window_crack_00 + gap]
            : clim->user.f_val[he] > clim->arg.u[house_in_humidity_01 + gap]   ? clim->arg.u[house_side_window_crack_01 + gap]
            : clim->user.f_val[he] > clim->arg.u[house_in_humidity_02 + gap]   ? clim->arg.u[house_side_window_crack_02 + gap]
            : clim->user.f_val[he] > clim->arg.u[house_in_humidity_03 + gap]   ? clim->arg.u[house_side_window_crack_03 + gap]
            : clim->user.f_val[he] > clim->arg.u[house_in_humidity_0max + gap] ? clim->arg.u[house_side_window_crack_0max + gap]
                                                                               : 0U;

        //上述设置应用到底层硬件                                                                               
    }

    return 1;
#undef clim_get_site
#undef clim_val_gap
}

/**
 * @brief	气候系统中风扇控制
 * @details
 * @param	None
 * @retval  none
 */
static char clim_fans_control(struct climate *clim, uint8_t *coil)
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
void clim_control(void)
{
    struct climate *clim = &clim_system;
    pModbusHandle pd = Lte_Modbus_Object;
    // float tank_pre = poto_get_pre(poto, poto->arg.flag.tank, m_tank_pre, s_tank_pre);
    // float vap_pre = poto_get_pre(poto, poto->arg.flag.vap, m_vap_pre, s_vap_pre);

    RT_ASSERT(pd);

    memset(clim_coil, 0x00, sizeof(clim_coil));

    if (clim->flag.debug) // 处于调试模式，不执行系统逻辑
        // goto __no_action;
        return;

    clim_get_vals(clim, pd);    // 信号转换
    if (clim_check_error(clim)) // 错误检测
        goto __no_action;

//     if (poto_safe_mode(poto, tank_pre, poto_coil))
//     {
//         goto __no_action;
//     }

//     if (poto_get_boot_signal(poto, pd, poto_coil, 12U)) // 最大支持12个用户棚
//     {
//         poto_start_mode(poto, tank_pre, vap_pre, poto_coil);
//     }
//     else
//         poto_stop_mode(poto, tank_pre, vap_pre, poto_coil);

    clim_fans_control(clim, &clim_coil[0]);
    clim_windows_control(clim, &clim_coil[house_in_heat_0max - house_in_heat_00]);

__no_action:
    pd->Mod_Operatex(pd, Coil, lhc_modbus_write, clim->arg.pool.do_des.base_addr,
                     clim_coil, sizeof(clim_coil));
}

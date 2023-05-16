#include <rtthread.h>
#include <rtdevice.h>
#include <sys/time.h>
#include "lhc_modbus_port.h"
#include "user_photosynthesis.h"

/**
 * @brief priva 系统介绍资料
 * Priva Compass http://www.sheshiyuanyi.com/technology-wenshizhongzhizhinanzhepriva_compass.html
 *
 **/

/*阀门说明：
 *     Q0            Q1          Q2          Q3          Q4-Q15                  合计
 *     汽象电磁阀      液相电磁阀    泄压电磁阀   出口电磁阀    用户电磁阀+预留电磁阀      16路（2块DO）
 *     DI0-DI7
 *     启动信号（外部模式时）                                                        8路（1块DI)
 */

struct potos poto_system; // 光合作用系统
static uint8_t poto_coil[16U];

#define poto_get_pre(_p, _flag, _m, _s) \
    (_flag) ? (_p)->user.f_val[_s] : (_p)->user.f_val[_m]

/**
 * @brief	光合作用系统安全模式
 * @details
 * @param	None
 * @retval  none
 */
static uint8_t poto_safe_mode(struct potos *poto,
                              float tank_pre,
                              uint8_t *coil)
{
    /*检测到系统错误*/
    if (poto->flag.code)
        return 1;
    /*储槽压力低于储槽压力阈值*/
    if (tank_pre < poto->arg.f[tank_p_limit])
        return 1;
    /*储槽液位低于储槽液位阈值*/
    if (poto->user.f_val[tank_level] < poto->arg.f[tank_i_limit])
        return 1;

    return 0;
}

/**
 * @brief	光合作用系统启动模式
 * @details
 * @param	None
 * @retval  none
 */
static void poto_start_mode(struct potos *poto,
                            float tank_pre,
                            float vap_pre,
                            uint8_t *coil)
{
    /*只要汽化器出口压力 > 汽化器补压启动值，出口阀打开*/
    if (vap_pre > poto->arg.f[s_vap_start])
    {
        coil[poto_q3] = 1;
    }

    /*储槽压力 > 储槽泄压启动值:优先使用汽象阀供气*/
    tank_pre > poto->arg.f[s_spf_start] ? __SET_FLAG(poto->flag.mutex, 0)
                                        : false;
    /*储槽压力 < 储槽泄压停止值*/
    tank_pre > poto->arg.f[s_spf_stop] ? __RESET_FLAG(poto->flag.mutex, 0)
                                       : false;
    /*储槽压力 > 储槽泄压启动值 && 储槽压力 还没有 < 储槽泄压停止值*/
    if (__GET_FLAG(poto->flag.mutex, 0))
    {
        coil[poto_q0] = 1;
        return;
    }

    /*汽化器出口压力 > 汽化器补压停止值: 开液相阀产气*/
    if (vap_pre > poto->arg.f[s_vap_stop])
    {
        coil[poto_q1] = 1;
    } // 汽化器出口压力 < 汽化器补压停止值/汽化器补压启动值，并且储槽压力大于储槽泄压启动值：开气象阀补压
    else if (tank_pre > poto->arg.f[s_tank_supplement])
    {
        coil[poto_q0] = 1;
    }

    /*Ensure that after the startup mode is switched to the shutdown mode*/
    __GET_FLAG(poto->flag.mutex, 1) ? __RESET_FLAG(poto->flag.mutex, 1)
                                    : false;
    __GET_FLAG(poto->flag.mutex, 2) ? __RESET_FLAG(poto->flag.mutex, 2)
                                    : false;
}

/**
 * @brief	光合作用系统停机模式
 * @details
 * @param	None
 * @retval  none
 */
static void poto_stop_mode(struct potos *poto,
                           float tank_pre,
                           float vap_pre,
                           uint8_t *coil)
{
    /*Check whether the pressure relief operation is on*/
    __GET_FLAG(poto->flag.mutex, 0) ? __RESET_FLAG(poto->flag.mutex, 0)
                                    : false;

    // 储槽压力 < 储槽压力回压阈值 && 汽化器出口压力 > 汽化器回压阈值，进行回压
    if (tank_pre < poto->arg.f[tank_back_limit] &&
        vap_pre > poto->arg.f[vap_back_limit])
    {
        coil[poto_q0] = 1;
    }

    /*储槽压力 > 储槽泄压启动值:*/
    tank_pre > poto->arg.f[p_spf_start] ? __SET_FLAG(poto->flag.mutex, 1)
                                        : false;
    /*储槽压力 < 储槽泄压停止值*/
    tank_pre < poto->arg.f[p_spf_stop] ? __RESET_FLAG(poto->flag.mutex, 1)
                                       : false;
    // 储槽压力 > 储槽泄压开始值 && 还没有 < 储槽泄压停止值
    if (__GET_FLAG(poto->flag.mutex, 1))
    {
        coil[poto_q2] = 1;
    }

    /*汽化器出口压力 > 汽化器泄压启动值:*/
    vap_pre > poto->arg.f[p_vap_start] ? __SET_FLAG(poto->flag.mutex, 2)
                                       : false;
    /*汽化器出口压力 < 汽化器泄压停止值*/
    vap_pre < poto->arg.f[p_vap_stop] ? __RESET_FLAG(poto->flag.mutex, 2)
                                      : false;
    // 汽化器压力 > 汽化器泄压开始值 && 还没有 < 汽化器泄压停止值
    if (__GET_FLAG(poto->flag.mutex, 2))
    {
        coil[poto_q3] = 1;
        coil[poto_q4] = 1; // 默认泄压至1号大棚
    }
}

/**
 * @brief	判断当前时间是否在目标时间段内
 * @details
 * @param	cur 当前时间
 * @param   start 开始时间
 * @param   end   结束时间
 * @retval  0:不在时间段内；1：在时间段内
 */
uint8_t distingulsh_times_slot(uint32_t cur, uint32_t start, uint32_t end)
{
    if (start == end) // 开始点和结束点相同:全天候工作
        return 1;

    if (start < end) // 没有跨越天
        return (cur >= start && cur <= end ? 1 : 0);

    return (cur > end && cur < start ? 0 : 1);
}

/**
 * @brief	根据co2浓度给出启动信号
 * @details
 * @note    co2控制器: https://www.docin.com/p-87568477.html
 * @param	None
 * @retval  none
 */
void by_co2_set_boot_signal(struct potos *poto, pModbusHandle pd)
{
#define POTO_CO2_OFFSET_PDU 8U
#define POTO_CO2_SITE 3U
    // uint16_t addr = LOUVER_BOX_START_ADDR + POTO_CO2_SITE;
    // uint16_t *prange = &poto->arg.u[co2_1_lower];
    uint16_t *pdata = &poto->user.u_val[co2_1];
    uint8_t boot[POTO_CO2_MAX];

    for (uint8_t i = 0; i < POTO_CO2_MAX; ++i)
    {
        pd->Mod_Operatex(pd, InputRegister, lhc_modbus_read,
                         poto->arg.pool.ai_des.offset_addr + i * POTO_CO2_OFFSET_PDU,
                         (uint8_t *)(pdata + i), sizeof(poto->user.u_val[co2_1]));
        // 前3位用于启动模式、停止模式
        *pdata > poto->arg.u[co2_1_upper + i * 2U] ? __RESET_FLAG(poto->flag.mutex, 3 + i) : false;
        *pdata < poto->arg.u[co2_1_lower + i * 2U] ? __SET_FLAG(poto->flag.mutex, 3 + i) : false;

        if (__GET_FLAG(poto->flag.mutex, 3 + i))
            boot[i] = 1;
        else
            boot[i] = 0;
    }
    // 写回启动信号
    pd->Mod_Operatex(pd, InputCoil, lhc_modbus_write, poto->arg.pool.di_des.base_addr,
                     boot, sizeof(boot));

#undef POTO_CO2_OFFSET_PDU
#undef POTO_CO2_SITE
}

/**
 * @brief	光合作用系统获取内部变量
 * @details
 * @param	None
 * @retval  None
 */
static void poto_get_vals(struct potos *poto,
                          pModbusHandle pd)
{
    float *data, *arg;

    pd->Mod_Operatex(pd, InputRegister, lhc_modbus_read, poto->arg.pool.ai_des.base_addr, //0x00
                     (uint8_t *)poto->user.f_val, sizeof(poto->user.f_val));

    for (data = poto->user.f_val, arg = poto->arg.f;
         data < poto->user.f_val + poto_f_vals_max;
         ++data, arg += 2U)
    {
        if (data == (poto->user.f_val + tank_level))
        {
            if (poto->arg.flag.pot) // 根据储罐类型选择公式
            {
                *data = Get_HZtank_Level(Get_Target(*data, *arg, *(arg + 1U)),
                                         poto->arg.f[tank_h], poto->arg.f[tank_r], poto->arg.f[tank_l]);
            }
            else
                *data = Get_VCtank_Level(poto->arg.f[tank_r], Get_Target(*data, *arg, *(arg + 1U)),
                                         poto->arg.f[tank_h]);
        }
        *data = Get_Target(*data, *arg, *(arg + 1U));
        if (*data < 0.0f)
            *data = 0.0f;
    }
}

/**
 * @brief	光合作用系统检测错误
 * @details
 * @param	None
 * @retval  0系统无错误 1系统存在错误
 */
static uint8_t poto_check_error(struct potos *poto)
{
#define POTO_ERROR_BASE_CODE 0x02
    uint8_t site;
    float *data = poto->user.f_val;

    for (site = 0; site < poto_f_vals_max; ++site, ++data)
    {
        if (__GET_FLAG(poto->arg.flag.mask, site)) //检查目标传感器是否被屏蔽
            continue;
        poto->flag.code = *data <= 0.0F ? (3U * site + POTO_ERROR_BASE_CODE)
                                        : (*data < (CURRENT_LOWER - 0.5F) /*Disconnection detection offset 0.5mA*/
                                               ? (3U * site + POTO_ERROR_BASE_CODE + 1U)
                                               : (*data > (CURRENT_LOWER + CURRENT_UPPER + 1.0F)
                                                      ? (3U * site + POTO_ERROR_BASE_CODE + 2U)
                                                      : 0));
        if (poto->flag.code) //按顺序报告错误码
            return 1;
    }
#undef POTO_ERROR_BASE_CODE
    return 0;
}

/**
 * @brief	光合作用系统获取启动信号
 * @details
 * @param	None
 * @retval  1；系统启动 0：系统停止
 */
uint8_t poto_get_boot_signal(struct potos *poto,
                             pModbusHandle pd,
                             uint8_t *coil,
                             uint8_t size)
{
    struct tm start_time = {0};
    struct tm end_time = {0};
    struct tm now_time = {0};
    time_t now;
    uint16_t sbit = 0;

    //汽化器温度小于临界值
    if (__GET_FLAG(poto->arg.flag.mask, gas_heat) &&
        poto->user.f_val[gas_heat] < poto->arg.f[gas_heat_limit])
        return 0;

    // get_timestamp(&now);
    time(&now);

    gmtime_r(&now, &now_time);
    gmtime_r((time_t *)&poto->arg.start_time, &start_time);
    gmtime_r((time_t *)&poto->arg.end_time, &end_time);

    // 按时间段启动
    now_time.tm_hour = now_time.tm_hour << 8U | now_time.tm_min;
    start_time.tm_hour = start_time.tm_hour << 8U | start_time.tm_min;
    end_time.tm_hour = end_time.tm_hour << 8U | end_time.tm_min;

    // 不在允许启动时间段
    if (!distingulsh_times_slot(now_time.tm_hour, start_time.tm_hour, end_time.tm_hour))
        return 0;

    if (poto->arg.flag.signal) // 信号由内部系统产生
    {
        by_co2_set_boot_signal(poto, pd);
    }

    /*启动信号读取不分内外，仅检测指定地址数据*/
    pd->Mod_Operatex(pd, InputCoil, lhc_modbus_read, poto->arg.pool.di_des.base_addr, //0x00
                     &coil[poto_q4], size);

    for (uint8_t i = 0; i < size; ++i)
    {
        sbit += coil[poto_q4 + i];
    }
    return sbit ? 1 : 0;
}

/**
 * @brief	光合作用控制
 * @details
 * @param	None
 * @retval  none
 */
void poto_control(void)
{
    struct potos *poto = &poto_system;
    pModbusHandle pd = Lte_Modbus_Object;
    float tank_pre = poto_get_pre(poto, poto->arg.flag.tank, m_tank_pre, s_tank_pre);
    float vap_pre = poto_get_pre(poto, poto->arg.flag.vap, m_vap_pre, s_vap_pre);

    RT_ASSERT(pd);

    memset(poto_coil, 0x00, sizeof(poto_coil));

    if (poto->flag.debug) // 处于调试模式，不执行系统逻辑
        // goto __no_action;
        return;

    poto_get_vals(poto, pd);    // 信号转换
    if (poto_check_error(poto)) // 错误检测
        goto __no_action;

    if (poto_safe_mode(poto, tank_pre, poto_coil))
    {
        goto __no_action;
    }

    if (poto_get_boot_signal(poto, pd, poto_coil, 12U)) // 最大支持12个用户棚
    {
        poto_start_mode(poto, tank_pre, vap_pre, poto_coil);
    }
    else
        poto_stop_mode(poto, tank_pre, vap_pre, poto_coil);

__no_action:
    pd->Mod_Operatex(pd, Coil, lhc_modbus_write, poto->arg.pool.do_des.base_addr,
                     poto_coil, sizeof(poto_coil));
}

#include "relay.h"
#include "main.h"
#include <rtthread.h>
#include <rtdevice.h>
#include "io_signal.h"
#include "small_modbus_port.h"
#include "test.h"

#ifdef DBG_TAG
#undef DBG_TAG
#endif
#define DBG_TAG "relay"
#define DBG_LVL DBG_LOG
#define USING_RELAY_DEBUG 1
#define RELAY_DEBUG_R dbg_raw
#define RELAY_DEBUG_D LOG_D

static void relay_any_tactics_action(pre_handle pre, relay_tactics_type cur_type);
// static void relay_manual_tactics_action(pre_handle pre, relay_tactics_type cur_type);
static relay_error_code relay_callback(pre_handle pre);
/*继电器动作策略组*/
relay_action_group rela_act_group[] = {
    {.action_type = pos_sque_action, .relay_operate = relay_any_tactics_action},
    {.action_type = rev_ord_action, .relay_operate = relay_any_tactics_action},
    {.action_type = pos_and_rev_action, .relay_operate = NULL},
    {.action_type = custom_action, .relay_operate = NULL},
};
/*继电器池*/
// uint8_t relay_coil[SIGNAL_IO_DO_MAX];
relay_handle relay_object = {
    .pag = rela_act_group,
    .action_group_size = sizeof(rela_act_group) / sizeof(rela_act_group[0]),
    .coil.size = SIGNAL_IO_DO_MAX,
    .next_time = RELAY_ACTION_NEXT_TIMES,
    .relay_delay = (void (*)(unsigned int))rt_thread_mdelay,
    .relay_callback = relay_callback,
    // .cur_exe_count = 0,
    .info.cur_exe_count = 0,
};

/**
 * @brief   继电器组初始化
 * @details
 * @param   None
 * @retval  None
 */
// int rt_relay_init(void)
// {
//     relay_object.pmodbus = Modbus_Object;
// #if (USING_RELAY_DEBUG)
//     RELAY_DEBUG_D("@note:relay_object = 0x%p\r\n", &relay_object);
// #endif
//     return 0;
// }
// /*在内核对象中初始化:https://blog.csdn.net/yang1111111112/article/details/93982354*/
// INIT_DEVICE_EXPORT(rt_relay_init);

/**
 * @brief  EXTI line detection callback.
 * @param  GPIO_Pin: Specifies the port pin connected to corresponding EXTI line.
 * @retval None
 */
// void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
// {
//     if (GPIO_Pin == GPIO_PIN_5)
//     {
//     }
// }

/**
 * @brief   继电器策略执行回调函数
 * @details
 * @param   pre 继电器组对象
 * @param   cur_site 当前点
 * @param
 * @retval  None
 */
relay_error_code relay_callback(pre_handle pre)
{
#define ZERO_CORSSING_OVERTIMES 3000U // 信号量等待3s超时，测量失败退出
    extern rt_sem_t trig_semaphore;
    extern rt_sem_t continue_semaphore;
    relay_error_code result = re_ok;
    ptest_t pt = &test_object;
    rt_err_t rt_result;
    uint16_t cur_site = pre->info.cur_exe_count;
    bool finsh_flag = false;

    if (pt->data.p == NULL || pre == NULL)
    {
        result = re_p_is_null;
#if (USING_RELAY_DEBUG)
        RELAY_DEBUG_D("@error: exe 'relay_callback', illegal operation null pointer[pre].\n");
#endif
        goto __exit;
    }

    switch (pt->pre->cur_power)
    {
        /*dc信号不存在过零点：手动触发采样*/
    case dc_out:
        /*触发adc同步采样当前dc信号*/
        release_semaphore(trig_semaphore);
        /*不确定直流期间是否会误触发过流信号：关闭过零检测*/
        __SET_FLAG(pt->flag, test_zero_crossing_signal);
        break;
    case ac1_out:
    case ac2_out:
        /*ac信号：过零自动触发*/
        // pe->count = ZERO_CORSSING_OVERTIMES;
        /*开启下一次过零检测*/
        __RESET_FLAG(pt->flag, test_zero_crossing_signal);
        break;
    default:
        break;
    }
    rt_result = rt_sem_take(continue_semaphore, ZERO_CORSSING_OVERTIMES);
    /* 永久方式等待信号量，加上一个超时定时器：数据处理完成后获取*/
    if (rt_result != RT_EOK)
    {
        result = re_other_err;
#if (USING_RELAY_DEBUG)
        RELAY_DEBUG_D("@error: exe 'relay_callback', waiting for semaphore error: %ld.\n", rt_result);
#endif
        goto __exit;
    }

    /*与允许误差比较：不在范围内的不满足标准*/
    if (pt->data.p[cur_site].voltage_offset > pt->comm_param.voltage_offset ||
        pt->data.p[cur_site].current_offset > pt->comm_param.current_offset)
    {
        /*设置结果*/
        __SET_FLAG(pt->test_result, cur_site);
        // pt->cur_stage = test_filed;
#if (USING_RELAY_DEBUG)
        RELAY_DEBUG_D("@error: exe 'relay_callback', error generation.\
        \n\rcur_site\tv_offset\ti_offset\r\n--------\t--------\t--------\r\n%d\t\t%f\t%f\r\n",
                      cur_site, pt->data.p[cur_site].voltage_offset, pt->data.p[cur_site].current_offset);
#endif
        // result = re_other_err;
        // goto __exit;
    }

    if (cur_site == pre->info.end_site)
    {
        if (__GET_FLAG(pt->flag, test_mode)) // 手动模式：单次结束
        {
            finsh_flag = true;
        }
        else
        {
            relay_group temp_group = {
                .start = 1U,
                .end = TEST_MAX_NUM,
            };
            pre->cur_group = temp_group; // 初始化下一次组开启顺序
            if (++pt->auto_count >= max_freq)
            {
                pt->auto_count = 0;
                finsh_flag = true;
            }
        }
        if (finsh_flag)
        {
            __SET_FLAG(pt->flag, test_finsh_signal);
        }
    }

    return result;

__exit:
    __SET_FLAG(pt->flag, test_filed_signal); // 发生错误：主动结束
    return result;
#undef ZERO_CORSSING_OVERTIMES
}

/**
 * @brief   继电器策略执行
 * @details
 * @param   pre 继电器组对象
 * @param   cur_type 当前动作类型
 * @retval  None
 */
void relay_poll(pre_handle pre, relay_tactics_type cur_type)
{
    if (NULL == pre || NULL == pre->pag)
        return;
    for (relay_action_group *p = pre->pag;
         p < pre->pag + pre->action_group_size; ++p)
    {
        if (p->action_type == cur_type)
        {
            if (p->relay_operate)
            {
                p->relay_operate(pre, cur_type);
                break;
            }
        }
    }
}

/**
 * @brief   通过继电器输出目标电压类型
 * @details
 * @param   pre 继电器组对象
 * @param   site 线圈位置
 * @param   state 目标状态
 * @retval  执行结果
 */
static relay_error_code set_relay_coil(pre_handle pre,
                                       uint8_t site,
                                       relay_state state)
{
    if (NULL == pre || NULL == pre->coil.p)
        return re_p_is_null;

    if (site > pre->coil.size)
        return re_coil_over;

    if (state > relay_open)
        return re_unknown_state;
    /*数据写回输出寄存器池*/
    *(pre->coil.p + site) = (uint8_t)state;

    return re_ok;
}

/**
 * @brief   操作继电器组
 * @details
 * @param   pre 继电器组对象
 * @param   start_site 开始位置
 * @param   end_site 结束位置
 * @retval  执行结果
 */
static relay_error_code operate_relay_coil_group(pre_handle pre,
                                                 uint8_t start_site,
                                                 uint8_t end_site,
                                                 relay_state state)
{
    relay_error_code result = re_ok;

    if (NULL == pre)
    {
        result = re_p_is_null;
        goto __exit;
    }

    result = set_relay_coil(pre, start_site, state);
    result = set_relay_coil(pre, end_site, state);

__exit:
    return result;
}

/**
 * @brief   通过继电器输出目标电压类型
 * @details
 * @param   pre 继电器组对象
 * @retval  执行结果
 */
static relay_error_code set_relay_target_power(pre_handle pre,
                                               relay_power_type power,
                                               relay_state state)
{
    /*电源类型*/
#define K19_DC 9U
#define K29_AC 23U
#define K39_AC 11U
/*电流挡位*/
#define K1A_AC_50HZ 10U
#define K1A_AC_xHZ 24U

    relay_error_code result = re_ok;
    result = power < ac1_out
                 ? set_relay_coil(pre, K19_DC, state)
                 : power < ac2_out
                 ? set_relay_coil(pre, K29_AC, state),
    set_relay_coil(pre, K1A_AC_50HZ, state) /*选择目标电流挡:默认在直流挡*/
        : power < null_out
        ? set_relay_coil(pre, K39_AC, state),
    set_relay_coil(pre, K1A_AC_xHZ, state)
        : re_power_err;

    return result;
#undef K19_DC
#undef K29_AC
#undef K39_AC
#undef K1A_AC_50HZ
#undef K1A_AC_xHZ
}

/**
 * @brief   设置继电器下一个动作周期
 * @details
 * @param   pre 继电器组对象
 * @param   times 目标时间
 * @retval  执行结果
 */
static relay_error_code set_coil_next_action_cycle(pre_handle pre,
                                                   test_timer timer_id,
                                                   uint32_t times)
{
    if (NULL == pre->relay_delay || NULL == pre->timer.p)
        return re_p_is_null;
    // if (times)
    //     pre->relay_delay(times);

    if (timer_id > pre->timer.num)
        return re_param_size_err;

    test_timer_t *ptimer = (test_timer_t *)pre->timer.p;
    ptimer = &ptimer[timer_id];

    if (NULL == ptimer)
        return re_p_is_null;

    if (!ptimer->set_flag)
    {
        ptimer->flag = false;
        ptimer->count = times;
        ptimer->set_flag = true;
    }

    if (!ptimer->flag)
        return re_wait_continue;

    ptimer->flag = false; // 本阶段定时结束
    ptimer->set_flag = false;

    return re_ok;
}

struct relay_tactics
{
    uint8_t tactics[2U][ACTION_MAX_GROUP_NUM * 2U];
    // uint8_t tactics_count;
};
struct relay_matrix
{
    struct relay_tactics tactics;
    uint8_t matrix_size;
    relay_error_code result;
};
/*继电器策略组*/
const struct relay_tactics relay_matrix_group[] = {
    {
        .tactics = {
            {0, 1, 2, 3, 4, 5, 6, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
            {16, 17, 18, 19, 20, 21, 22, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
        },
        // .tactics_count = ACTION_MAX_GROUP_NUM,
    }, /*正向递进vs1->vs2*/
    // {
    //     .tactics = {
    //         {1, 2, 3, 4, 5, 6, 7, 8, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
    //         {0, 1, 2, 3, 4, 5, 6, 7, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff},
    //     },
    //     .tactics_count = ACTION_MAX_GROUP_NUM,
    // }, /*反向递进vs2->vs1*/
    // {
    //     .tactics = {
    //         {0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8},
    //         {1, 0, 2, 1, 3, 2, 4, 3, 5, 4, 6, 5, 7, 6, 8, 7},
    //     },
    //     .tactics_count = ACTION_MAX_GROUP_NUM * 2U,
    // }, /*先正向后逆向*/
    // {
    //     .tactics = {
    //         {1, 0, 2, 1, 3, 2, 4, 3, 5, 4, 6, 5, 7, 6, 8, 7},
    //         {0, 1, 1, 2, 2, 3, 3, 4, 4, 5, 5, 6, 6, 7, 7, 8},
    //     },
    //     .tactics_count = ACTION_MAX_GROUP_NUM * 2U,
    // }, /*先逆向后正向*/
};

// struct relay_matrix matrix_tactics = {
//     .ptactics = relay_matrix_group,
//     .matrix_size = sizeof(relay_matrix_group) / sizeof(relay_matrix_group[0]),
// };

#define __RESET_RELAY(__pre, __size)                                      \
    do                                                                    \
    {                                                                     \
        memset((__pre)->pcoil, 0x00, (__size));                           \
        memset(&((__pre)->pcoil[SIGNAL_IO_DO_MAX / 2U]), 0x00, (__size)); \
    } while (0)

/**
 * @brief   cs组信号输出
 * @details
 * @param   p_gpio_info   gpio 信息表指针
 * @param   p_state_table 状态表指针
 * @param   start_pin       cs 开始输出管脚
 * @param   end_pin         cs 结束输出管脚
 * @retval  执行结果
 */
static relay_error_code relay_cs_signal_output(Gpiox_info *const p_gpio_info,
                                               uint8_t *const p_state_table,
                                               uint8_t start_pin,
                                               uint8_t end_pin)
{
    if (NULL == p_gpio_info || NULL == p_gpio_info->pGPIOx ||
        NULL == p_state_table)
        return re_p_is_null;

    if (start_pin > end_pin)
        return re_praram_err;

    GPIO_TypeDef *pgpiox = (GPIO_TypeDef *)p_gpio_info->pGPIOx;
    uint32_t pin = pgpiox->ODR; // 保留当前引脚状态

    for (uint8_t i = start_pin; i < end_pin; ++i)
    {
        switch (p_state_table[i]) // 此处输出与实际电平相反
        {
        case 0:
            pin |= p_gpio_info[i].Gpio_Pin;
            break;
        case 1:
            pin &= ~p_gpio_info[i].Gpio_Pin;
            break;
        default:
            break;
        }
    }

    pgpiox->BSRR = (uint32_t)(pin & 0xffff) | ((~pin) << 16U);

    return re_ok;
}

/**
 * @brief   获取cs信息配置表
 * @details
 * @param   None
 * @retval  gpio 配置表指针
 */
static Gpiox_info *get_cs_gpio_info(void)
{
    static Gpiox_info cs_gpio[] = {
        // {.pGPIOx = CS00_GPIO_Port, .Gpio_Pin = CS00_Pin}, // cs_00
        // {.pGPIOx = CS01_GPIO_Port, .Gpio_Pin = CS01_Pin}, // cs_01
        // {.pGPIOx = CS02_GPIO_Port, .Gpio_Pin = CS02_Pin}, // cs_02
        // {.pGPIOx = CS10_GPIO_Port, .Gpio_Pin = CS10_Pin}, // cs_10
        // {.pGPIOx = CS11_GPIO_Port, .Gpio_Pin = CS11_Pin}, // cs_11
        // {.pGPIOx = CS12_GPIO_Port, .Gpio_Pin = CS12_Pin}, // cs_12

        {.pGPIOx = CS12_GPIO_Port, .Gpio_Pin = CS12_Pin}, // cs_12
        {.pGPIOx = CS11_GPIO_Port, .Gpio_Pin = CS11_Pin}, // cs_11
        {.pGPIOx = CS10_GPIO_Port, .Gpio_Pin = CS10_Pin}, // cs_10
        {.pGPIOx = CS02_GPIO_Port, .Gpio_Pin = CS02_Pin}, // cs_02
        {.pGPIOx = CS01_GPIO_Port, .Gpio_Pin = CS01_Pin}, // cs_01
        {.pGPIOx = CS00_GPIO_Port, .Gpio_Pin = CS00_Pin}, // cs_00
    };

    return cs_gpio;
}
/**
 * @brief   选通对应采样通道
 * @details
 * @param   pre 继电器组对象
 * @param   size 继电器数量
 * @retval  执行结果
 */
static relay_error_code relay_set_cs_signal(uint8_t cs_site)
{
    static uint8_t cs_order[][6] = {
        // {0, 0, 0, 1, 0, 0},
        // {1, 0, 0, 0, 1, 0},
        // {0, 1, 0, 1, 1, 0},
        // {1, 1, 0, 0, 0, 1},
        // {0, 0, 1, 1, 0, 1},
        // {1, 0, 1, 0, 1, 1},
        // {0, 1, 1, 1, 1, 1},

        {0, 0, 1, 0, 0, 0},
        {0, 1, 0, 0, 0, 1},
        {0, 1, 1, 0, 1, 0},
        {1, 0, 0, 0, 1, 1},
        {1, 0, 1, 1, 0, 0},
        {1, 1, 0, 1, 0, 1},
        {1, 1, 1, 1, 1, 0},

        // {0, 0, 1, 0, 0, 1},
        // {0, 1, 0, 0, 1, 0},
        // {0, 1, 1, 0, 1, 1},
        // {1, 0, 0, 1, 0, 0},
        // {1, 0, 1, 1, 0, 1},
        // {1, 1, 0, 1, 1, 0},
        // {1, 1, 1, 1, 1, 1},
    };

    if (cs_site > sizeof(cs_order) / sizeof(cs_order[0]))
        return re_param_size_err;

#if (USING_RELAY_DEBUG)
        // RELAY_DEBUG_D("@note: exe 'relay_set_cs_signal', cs site: %d.\n", cs_site);
#endif
    return relay_cs_signal_output(get_cs_gpio_info(), cs_order[cs_site], 0, sizeof(cs_order[0]));
}

/**
 * @brief   通过继电器模式获得对应策略
 * @details
 * @param   pre 继电器组对象
 * @param   relay_mode 当前动作模式组
 * @retval  执行结果
 */
static struct relay_matrix get_tactics_by_relay_mode(pre_handle pre,
                                                     relay_tactics_type relay_mode)
{
    struct relay_matrix target_matrix = {
        .matrix_size = sizeof(relay_matrix_group) / sizeof(relay_matrix_group[0]),
        .result = re_ok,
    };

    if (NULL == pre || relay_mode > target_matrix.matrix_size)
    {
        target_matrix.result = re_praram_err;
        goto __exit;
    }

    if (!pre->cur_group.start || !pre->cur_group.end)
    {
        target_matrix.result = re_other_err;
        goto __exit;
    }

    target_matrix.tactics = relay_matrix_group[(uint8_t)relay_mode]; /*访问flash*/

__exit:
    return target_matrix;
}

/**
 * @brief   继电器动作执行器
 * @details
 * @param   pre 继电器组对象
 * @param   relay_mode 当前动作模式组
 * @retval  执行结果
 */
static relay_error_code set_coil_action_exe(pre_handle pre,
                                            relay_tactics_type relay_mode)
{
    struct relay_matrix cur_matrix = get_tactics_by_relay_mode(pre, relay_mode);
    relay_error_code result = re_ok;

    if (NULL == pre || cur_matrix.result != re_ok)
    {
        result = re_call_err;
        goto __exit;
    }

    result = relay_set_cs_signal(pre->info.cur_exe_count); // 保证cs信号仅仅单次选通
    if (result != re_ok)
    {
#if (USING_RELAY_DEBUG)
        RELAY_DEBUG_D("@error: failed to operate cs signal, cs group:[%d].\n", pre->info.cur_exe_count);
#endif
        goto __exit;
    }

    /*根据电源类型打开目标电源*/
    set_relay_target_power(pre, pre->cur_power, relay_open);
    result = operate_relay_coil_group(pre, cur_matrix.tactics.tactics[0][pre->info.cur_exe_count],
                                      cur_matrix.tactics.tactics[1][pre->info.cur_exe_count], relay_open);
    if (result != re_ok)
        goto __exit;

    result = set_coil_next_action_cycle(pre, time_id_relay, pre->next_time);
    if (result != re_ok)
    {
        if (result == re_wait_continue)
            return result;
        else
            goto __exit;
    }

#if (USING_RELAY_DEBUG)
    RELAY_DEBUG_R("\r\ncs\tstart\tend\tk1x\tk2x\r\n---\t-----\t---\t---\t---\r\n%d\t%d\t%d\t%d\t%d\r\n",
                  pre->info.cur_exe_count, pre->cur_group.start - 1U, pre->info.end_site,
                  cur_matrix.tactics.tactics[0][pre->info.cur_exe_count], cur_matrix.tactics.tactics[1][pre->info.cur_exe_count]);
#endif

    /*回调函数采集数据、计算偏差*/
    if (pre->relay_callback) /*回调函数中传入一些继电器策略*/
        if (pre->relay_callback(pre) != re_ok)
        {
            goto __exit;
        }

    /*复位当前操作组*/
    result = operate_relay_coil_group(pre, cur_matrix.tactics.tactics[0][pre->info.cur_exe_count],
                                      cur_matrix.tactics.tactics[1][pre->info.cur_exe_count], relay_close);
    if (result != re_ok)
        goto __exit;

    // if (++pre->.info.cur_exe_count >= pre->cur_group.end)
    // {
    //     pre->.info.cur_exe_count = 0;
    //     /*测试完毕后关闭电源*/
    //     set_relay_target_power(pre, pre->cur_power, relay_close);
    // }
    /*根据执行顺序，得到有效结束条件*/
    if (!pre->info.order)
    {
        if (++pre->info.cur_exe_count >= pre->cur_group.end)
            goto __close_power;
    }
    else
    {
        if (--pre->info.cur_exe_count > pre->cur_group.start)
            goto __close_power;
    }

    return result;

__exit:

    // pre->.info.cur_exe_count = 0;
    /*意外错误或者测试完毕，关闭所有继电器（包括电源）*/
    // for (uint8_t i = 0; i < ACTION_MAX_GROUP_NUM; ++i)
    // {
    //     operate_relay_coil_group(pre, cur_matrix.tactics.tactics[0][i],
    //                              cur_matrix.tactics.tactics[1][i], relay_close);
    // }
#if (USING_RELAY_DEBUG)
    RELAY_DEBUG_D("@error: exe 'set_coil_action_exe', an error occurred.\n");
#endif

__close_power:
    /*测试完毕后关闭电源*/
    set_relay_target_power(pre, pre->cur_power, relay_close);
    pre->info.cur_exe_count = 0;

    return result;
}

/**
 * @brief   继电器策略执行
 * @details
 * @param   pre 继电器组对象
 * @param   cur_type 当前动作类型
 * @retval  None
 */
static void relay_any_tactics_action(pre_handle pre, relay_tactics_type cur_type)
{
    /*根据电源类型打开目标电源*/
    // set_relay_target_power(pre, pre->cur_power, relay_open);
    if (set_coil_action_exe(pre, cur_type) != re_ok)
    {
        //        __GET_FLAG(pt->flag, test_filed_signal);
    }
    /*测试完毕后关闭电源*/
    // set_relay_target_power(pre, pre->cur_power, relay_close);
}

/**
 * @brief   继电器手动模式时策略执行
 * @details
 * @param   pre 继电器组对象
 * @param   cur_type 当前动作类型
 * @retval  None
 */
// static void relay_manual_tactics_action(pre_handle pre, relay_tactics_type cur_type)
// {
//     /*根据电源类型打开目标电源*/
//     set_relay_target_power(pre, pre->cur_power, relay_open);
// }

#ifdef RT_USING_FINSH
#include <finsh.h>

/**
 * @brief   设置cs信号
 * @details
 * @param   none
 * @retval  none
 */
static void set_cs(int argc, char **argv)
{
    if (argc < 7)
    {
        RELAY_DEBUG_R("@error: Please input'set_cs <(x x x x x x)> e.g 0 0 0 0 0 1'.\n");
        return;
    }
    if (argc > 7)
    {
        RELAY_DEBUG_R("@error: parameter is too long,please input'set_cs <(x x x x x x)> e.g 0 0 0 0 0 1'.\n");
        return;
    }

    uint8_t cs_state_table[6];

    for (uint8_t i = 0; i < sizeof(cs_state_table) / sizeof(cs_state_table[0]); ++i)
    {
        uint8_t state = (uint8_t)atoi(argv[1 + i]);
        if (state > 1U)
        {
            RELAY_DEBUG_R("@error: parameter[%d]: %d error,please input'0/1'.\n", i + 1, state);
            return;
        }
        cs_state_table[i] = state;
    }
    if (relay_cs_signal_output(get_cs_gpio_info(), cs_state_table, 0, sizeof(cs_state_table)) == re_ok)
    {
        RELAY_DEBUG_R("@note: set cs signal success.\r\n");
    }
    else
        RELAY_DEBUG_R("@error: set cs signal failed.\r\n");
}
MSH_CMD_EXPORT(set_cs, set_cs<(x x x x x x)> e.g 0 0 0 0 0 1.);
#endif

#include "user_water_room.h"
#include "lhc_modbus_port.h"
#include <rtdevice.h>
#include <qled.h>

/*
* note : 使用quick led组件时，目标pin必须带上 INDIRC_COIL 标识
*/

static rt_timer_t timer = RT_NULL;

struct water water_system;
static uint8_t water_coil[16U];
static uint16_t water_ferA_last_state[(fer_Amax - fer_A0) + 1];
static uint16_t water_ferB_last_state[(fer_Bmax - fer_B0) + 1];

static char *water_ferA_pin_name[] = {
    "PC.8",
    "PC.7",
};

static char *water_ferB_pin_name[] = {
    "PC.6",
    "PD.15",
};


/**
 * @brief	水系统获取内部变量
 * @details
 * @param	None
 * @retval  None
 */
static void water_get_vals(struct water *water,
                          pModbusHandle pd)
{
    float *data, *arg;

    //数据的分步获取：雨量片区域或者一个棚一个，温度和湿度间隔一定地址
    pd->Mod_Operatex(pd, InputRegister, lhc_modbus_read, water->arg.pool.ai_des.base_addr, //offset
                     (uint8_t *)water->user.f_val, sizeof(water->user.f_val));

    for (data = water->user.f_val, arg = water->arg.f;
         data < water->user.f_val + water_f_vals_max;
         ++data, arg += 2U)
    {
        *data = Get_Target(*data, *arg, *(arg + 1U));

        if (*data < 0.0f)
            *data = 0;
    }
}

/**
 * @brief	水系统检测错误
 * @details
 * @param	None
 * @retval  0系统无错误 1系统存在错误
 */
static uint8_t water_check_error(struct water *water)
{
#define WATER_ERROR_BASE_CODE 0x02
    uint8_t site;
    float *data = water->user.f_val;

    for (site = 0; site < water_f_vals_max; ++site, ++data)
    {
        if (__GET_FLAG(water->arg.flag.mask, site)) //检查目标传感器是否被屏蔽
            continue;
        water->flag.code = *data <= 0.0F ? (3U * site + WATER_ERROR_BASE_CODE)
                                        : (*data < (CURRENT_LOWER - 0.5F) /*Disconnection detection offset 0.5mA*/
                                               ? (3U * site + WATER_ERROR_BASE_CODE + 1U)
                                               : (*data > (CURRENT_LOWER + CURRENT_UPPER + 1.0F)
                                                      ? (3U * site + WATER_ERROR_BASE_CODE + 2U)
                                                      : 0));
        if (water->flag.code) //按顺序报告错误码
            return 1;
    }
#undef WATER_ERROR_BASE_CODE
    return 0;
}

/**
 * @brief	水系统配肥开关初始化
 * @details
 * @param	None
 * @retval  none
 */
static void water_fer_switch_init(void)
{
    rt_int32_t pin_num;
    uint8_t i;
    
    for (i = 0; i < sizeof(water_ferA_pin_name) / sizeof(water_ferA_pin_name[0]); ++i)
    {
        pin_num = rt_pin_get(water_ferA_pin_name[i]);
        qled_add(pin_num, GPIO_PIN_SET);
    }

    for (i = 0; i < sizeof(water_ferB_pin_name) / sizeof(water_ferB_pin_name[0]); ++i)
    {
        pin_num = rt_pin_get(water_ferB_pin_name[i]);
        qled_add(pin_num, GPIO_PIN_SET);
    }
}

/**
 * @brief	水系统配肥开关移除
 * @details
 * @param	None
 * @retval  none
 */
static void water_fer_switch_remove(void)
{
    rt_int32_t pin_num;
    uint8_t i;
    
    for (i = 0; i < sizeof(water_ferA_pin_name) / sizeof(water_ferA_pin_name[0]); ++i)
    {
        pin_num = rt_pin_get(water_ferA_pin_name[i]);
        qled_remove(pin_num);
    }

    for (i = 0; i < sizeof(water_ferB_pin_name) / sizeof(water_ferB_pin_name[0]); ++i)
    {
        pin_num = rt_pin_get(water_ferB_pin_name[i]);
        qled_remove(pin_num);
    }
}

/**
 * @brief	水系统配肥开关关闭
 * @details
 * @param	None
 * @retval  none
 */
static void water_fer_switch_off(void)
{
    rt_int32_t pin_num;
    uint8_t i;
    
    for (i = 0; i < sizeof(water_ferA_pin_name) / sizeof(water_ferA_pin_name[0]); ++i)
    {
        pin_num = rt_pin_get(water_ferA_pin_name[i]);
        qled_set_off(pin_num);
    }

    for (i = 0; i < sizeof(water_ferB_pin_name) / sizeof(water_ferB_pin_name[0]); ++i)
    {
        pin_num = rt_pin_get(water_ferB_pin_name[i]);
        qled_set_off(pin_num);
    }
}

static void water_ferx_switch_cb(void *arg)
{
    struct water *water = &water_system;
    uint8_t flag_site = *(uint8_t *)arg;

    __RESET_FLAG(water->flag.mutex, flag_site);
}

/**
 * @brief	水系统设置肥料开关属性
 * @details
 * @param	None
 * @retval  None
 */
static void water_set_fer_switch_nature(struct water *water,
                                        uint16_t *cur_sate,
                                        uint8_t arg_start,
                                        uint8_t arg_end,
                                        char **pin_name,
                                        uint16_t *last_state)
{
    rt_int32_t pin_num;
    uint8_t i = 0, arg_site;
    uint16_t open_times;
    uint16_t rem_sate;
    float per;
    uint16_t switch_data[] = {0, 0};

    for (arg_site = arg_start; cur_sate && arg_site < arg_end; ++arg_site)
    {
        i = arg_site - arg_start;

         if (!__GET_FLAG(water->flag.mutex, i)) 
         {
            __SET_FLAG(water->flag.mutex, i);
            open_times = (uint16_t)((float)cur_sate[i] / 100.0f *
                                    (float)water->arg.cfg_time);
            switch_data[0] = open_times;
            switch_data[0] = water->arg.cfg_time - open_times; // + 1000 Interval of 1 second
            qled_set_special(pin_num, switch_data, sizeof(switch_data) / sizeof(switch_data[0]),
                             water_ferx_switch_cb, &i);
         }
    }
}

/**
 * @brief	水系统肥料配置
 * @details
 * @param	None
 * @retval  None
 */
static int water_fertilizer_config(struct water *water)
{
    if (!water->arg.fer_group)
    {
         water_set_fer_switch_nature(water, &water->arg.u[fer_A0], fer_A0, fer_B0,
                                     water_ferA_pin_name, water_ferA_last_state);
    }
    else
         water_set_fer_switch_nature(water, &water->arg.u[fer_B0], fer_B0, water_u_args_max,
                                     water_ferB_pin_name, water_ferB_last_state);

    return 0;
}

/**
 * @brief	水系统缸液位检查
 * @details
 * @param	None
 * @retval  None
 */
static int water_vat_level_check(struct water *water)
{
    if (water->user.f_val[vat_level] < water->arg.f[vat_level_min])
        return -1;

    if (water->user.f_val[vat_level] >= water->arg.f[vat_level_max])
        return 0;                                                                                                                                
}

/**
 * @brief	水系统缸EC检查
 * @details
 * @param	None
 * @retval  0 / -1
 */
static int water_ec_check(struct water *water)
{
    if (water->user.f_val[fer_EC] < water->arg.f[fer_EC_min] ||
        water->user.f_val[fer_EC] > water->arg.f[fer_EC_max])
        return -1;

    return 0;                                                                                                                               
}

/**
 * @brief	水系统缸PH检查
 * @details
 * @param	None
 * @retval  0 / -1
 */
static int water_ph_check(struct water *water)
{
    if (water->user.f_val[fer_PH] < water->arg.f[fer_PH_min] ||
        water->user.f_val[fer_PH] > water->arg.f[fer_PH_max])
        return -1;
        
    return 0;                                                                                                                                
}

/**
 * @brief	水系统管道压力检查
 * @details
 * @param	None
 * @retval  0 / -1
 */
static int water_pipe_presure_check(struct water *water)
{
    if (water->user.f_val[pipe_pre] > water->arg.f[pipe_pre_max])
        return -1;
        
    return 0;                                                                                                                                
}

/**
 * @brief	水系统管道流量检查
 * @details
 * @param	None
 * @retval  0 / -1
 */
static int water_pipe_flow_check(struct water *water)
{
    if (water->user.f_val[pipe_flow] < water->arg.f[pipe_flow_min] ||
        water->user.f_val[pipe_flow] > water->arg.f[pipe_flow_max])
        return -1;

    return 0;                                                                                                                                
}

/**
 * @brief	水系统启动水泵
 * @details
 * @param	None
 * @retval  0 / -1
 */
static int water_pump_start(struct water *water, uint8_t *coil)
{
    coil[0] = 1;

    return 0;                                                                                                                                
}

/**
 * @brief	水系统停止水泵
 * @details
 * @param	None
 * @retval  0 / -1
 */
static int water_pump_stop(struct water *water, uint8_t *coil)
{
    coil[0] = 0;


    return 0;                                                                                                                                
}

/**
 * @brief	水肥系统灌溉控制
 * @details
 * @param	None
 * @retval  none
 */
static void water_irrigate_control(struct water *water, uint8_t *coil, uint8_t num)
{
    // uint32_t each_time;
    struct tm start_time = {0};
    struct tm end_time = {0};
    struct tm now_time = {0};
    time_t now;

    water->node_count %= num;
    coil[water->node_count] = 1;

    if (water->flag.next_node)
        return;

    time(&now);
    gmtime_r(&now, &now_time);
    gmtime_r((time_t *)&water->arg.start_time, &start_time);
    gmtime_r((time_t *)&water->arg.end_time, &end_time);

    water->each_time = (end_time.tm_hour * 60 + end_time.tm_min -
                        (start_time.tm_hour * 60 + start_time.tm_min)) /
                       num;
    // water->each_time *= 60;
    water->flag.next_node = 1;
    water->node_count++;
}

/**
 * @brief	水肥系统
 * @details
 * @param	None
 * @retval  none
 */
static void water_control(void)
{
    struct water *water = &water_system;
    pModbusHandle pd = share_lhc_modbus;
    static uint8_t debug_flag = 0;

    RT_ASSERT(pd);

    memset(water_coil, DIRC_COIL, sizeof(water_coil));

    if (water->flag.debug) // 处于调试模式，不执行系统逻辑
    {
        debug_flag = 1;
        // water_windows_remove();
        return;
    }
        
    if (debug_flag)
    {
        // water_windows_init();
        debug_flag = 0;
    }

    water_get_vals(water, pd);    // 信号转换
    if (water_check_error(water)) // 错误检测
    {
        // water_windows_off();
        goto __no_action;
    }

    if (water_vat_level_check(water))
    {
        water_coil[0] =  1; //加水开关
        goto __no_action;
    }

    extern char xx_system_boot_time_check(uint32_t s_time, uint32_t e_time);
    if (xx_system_boot_time_check(water->arg.start_time, water->arg.end_time))
    {
        goto __no_action;
    }

    //按比列配置肥料
    water_fertilizer_config(water);

    if (water_ec_check(water)) //ec检查
    {
        water->flag.code = 0x00;
        goto __no_action;
    }

    if (water_ph_check(water)) //ph检查
    {
        water->flag.code = 0x00;
        goto __no_action;
    }
    /*
    *@note : 系统存在2个模拟量presure、flow，但是控制目标只有一个，难点在于怎么平衡二者
    */
    water_pump_start(water, &water_coil[1]); //水泵启动开关

    if (water_pipe_presure_check(water)) // pipe_presure检查
    {
        water->flag.code = 0x00;
        goto __no_action;
    }

    if (water_pipe_flow_check(water)) // pipe_flow检查
    {
        water->flag.code = 0x00;
        goto __no_action;
    }
    /*后端灌溉阀控制*/
    water_irrigate_control(water, water_coil + 2, 5);

__no_action:
    pd->Mod_Operatex(pd, Coil, lhc_modbus_write, water->arg.pool.do_des.base_addr,
                     water_coil, water->arg.pool.do_des.num);
}

/**
 * @brief   水肥系统线程
 * @details
 * @param	parameter:线程初始参数
 * @retval  None
 */
void water_thread_entry(void *parameter)
{
    for (;;)
    {
        water_control();
        rt_thread_mdelay(1000);
    }
}

/**
 * @brief	rt_thread 软件定时器回调函数
 * @details
 * @param	parameter
 * @retval  None
 */
void timer_callback(void *parameter)
{
    struct water *water = (struct water *)parameter;

    if (!water->flag.next_node)
        return;

    water->each_time--;
    if (!water->each_time)
        water->flag.next_node = 0;
}

/**
 * @brief	水肥系统初始化
 * @details
 * @param	None
 * @retval  none
 */
static int water_init(void)
{
    rt_thread_t tid = rt_thread_create(
        "warter",
        water_thread_entry,
        RT_NULL,
        1024, 0x11, 20);

    RT_ASSERT(tid != RT_NULL);

    rt_thread_startup(tid);

    /* 创建定时器周期定时器 */
    timer = rt_timer_create("water_timer",
                            timer_callback,
                            &water_system,
                            1000 * 60,
                            RT_TIMER_FLAG_PERIODIC);
    /* 启动timer定时器 */
    if (timer != RT_NULL)
        rt_timer_start(timer);

    rt_kprintf("warter system init.\r\n");

    return (RT_EOK);
}
// INIT_APP_EXPORT(water_init);

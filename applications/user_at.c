#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
#include "main.h"
#include <fal.h>
#include <dfs_fs.h>
#include <at.h>

#ifdef DBG_TAG
#undef DBG_TAG
#endif
#define DBG_TAG "user_at"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

#define AT_MOUDLE_MAX 2

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

typedef struct
{
    char dev_name[8];     // 设备名
    at_client_t client;   // 当前at客户端
    enum at_state state;  // 当前at执行状态
    int event_id;         // at事件id
    rt_size_t recv_bufsz; // 接收缓冲区尺寸
    struct
    {
        at_cmd_t table;
        uint8_t size;
    } cmd; // 目标at模块命令表指针
    // at_cmd_t cmd_table;   //目标at模块命令表指针
    char *resp;                                                  // at模块响应指针
    rt_err_t (*at_temp_rx_ind)(rt_device_t dev, rt_size_t size); // at临时中断转移指针
} at_module;
typedef at_module *at_module_t;


// static at_cmd at_table[] = {
//     {.psend = "+++", .precv = "a"},
//     {.psend = "a", .precv = "+OK"},
//     {.psend = "AT+Z", .event = at_restart},
//     {.psend = "AT+ENTM", .event = at_restart},
//     {.psend = "AT+UART", .event = at_get_uart_param},
//     {.psend = "AT+NTPTM", .event = at_get_ntp_time},
// };

static void at_restart(void *resp, void *other);
static void at_get_uart_param(void *resp, void *other);
static void at_get_ntp_time(void *resp, void *other);

static at_cmd _4G_table[] = {
    {.psend = "AT+Z", .event = at_restart},
    {.psend = "AT+ENTM", .event = at_restart},
    {.psend = "AT+UART", .event = at_get_uart_param},
    {.psend = "AT+NTPTM", .event = at_get_ntp_time},
};

static at_cmd _wifi_table[] = {
    {.psend = "AT+Z", .event = at_restart},
    {.psend = "AT+ENTM", .event = at_restart},
    {.psend = "AT+UART", .event = at_get_uart_param},
    {.psend = "AT+NTPTM", .event = at_get_ntp_time},
};

at_module at_group[AT_MOUDLE_MAX] = {
    {
        .cmd = {
            .table = _4G_table,
            .size = sizeof(_4G_table) / sizeof(_4G_table[0]),
        },
        .dev_name = "uart2",
        .recv_bufsz = 128U,
    }, // 4G
    {
        .cmd = {
            .table = _wifi_table,
            .size = sizeof(_wifi_table) / sizeof(_wifi_table[0]),
        },
        .dev_name = "uart5",
        .recv_bufsz = 128U,
    }, // wifi
};

/**
 * @brief  at模块重启
 * @details
 * @param	resp at模块响应帧
 * @param   other 其他信息
 * @retval  None
 */
static void at_restart(void *resp, void *other)
{
    if (RT_NULL == resp || RT_NULL == other)
        return;

    at_module_t at = (at_module_t)other;

    rt_kprintf("@note: at module enter transport mode or restart....\r\n");

    at->event_id = at_noll_id;
    at->state = at_exit_transparent;
}

/**
 * @brief  获取at模块串口参数
 * @details
 * @note    可变参数宏使用：https://www.cnblogs.com/gogly/articles/2416833.html
 * @param	resp at模块响应帧
 * @param   other 其他信息
 * @retval  None
 */
static void at_get_uart_param(void *resp, void *other)
{
    uint32_t baudrate, databits, stopbits;
    char parity[8], control[8];
    if (RT_NULL == resp || RT_NULL == other)
        return;

    at_module_t at = (at_module_t)other;

    sscanf((char *)resp, "%*[^=]=%d,%d,%d,%s,%s", &baudrate, &databits,
           &stopbits, parity, control);
    rt_kprintf("baudrate: %d, databits: %d, stopbits: %d, parity: %d, control: %d\r\n",
               baudrate, databits, stopbits, parity, control);

    /*可以连续处理多个at事件*/
    at->event_id = at_entm;
    // pt->at_state = at_exit_transparent;
}

/**
 * @brief  通过at模块获取网络时间
 * @details
 * @note    sscsnf使用正则表达式：https://www.cnblogs.com/lanjianhappy/p/7171341.html
 * @param	resp at模块响应帧
 * @param   other 其他信息
 * @retval  None
 */
static void at_get_ntp_time(void *resp, void *other)
{
    if (RT_NULL == resp || RT_NULL == other)
        return;

    at_module_t at = (at_module_t)other;
    char buf[8];
    struct tm tm_new, *p = &tm_new;

    sscanf((char *)resp, "%*[^=]=%d-%d-%d  %d:%d:%d  %s",
           &p->tm_year, &p->tm_mon, &p->tm_mday, &p->tm_hour, &p->tm_min, &p->tm_sec, buf);
    rt_kprintf("%d-%d-%d  %d:%d:%d  %s\r\n",
               p->tm_year, p->tm_mon, p->tm_mday, p->tm_hour, p->tm_min, p->tm_sec, buf);

    at->event_id = at_entm;
}

/**
 * @brief   根据当前id
 * @details
 * @param	id at事件id
 * @retval  NULL/target cmd handle
 */
static at_cmd_t get_target_at_event(at_module_t at, enum at_cmd_id id)
{
    RT_ASSERT(at);
    if (at->cmd.table && id < at->cmd.size)
        return &at->cmd.table[id];

    return NULL;
}

/**
 * @brief   at进入临时指令模式
 * @details
 * @param client 目标客户端句柄
 * @param cmd_table 客户端指令表
 * @retval  操作的结果
 */
static rt_err_t at_enter_transparent_mode(at_client_t client)
{
    rt_err_t ret = RT_EOK;
    char at_recv_buf[32U];

    at_cmd enter_tans_cmd[] = {
        {.psend = "+++", .precv = "a"},
        {.psend = "a", .precv = "+OK"},
    };

    for (at_cmd_t cmd = enter_tans_cmd;
         client && (cmd < enter_tans_cmd + sizeof(enter_tans_cmd) / sizeof(at_cmd));
         ++cmd)
    {
        rt_device_read(client->device, 0, at_recv_buf, sizeof(at_recv_buf)); //清空上一次fifo缓冲区
        at_client_obj_send(client, cmd->psend, rt_strlen(cmd->psend));
        rt_memset(at_recv_buf, 0x00, sizeof(at_recv_buf));
        at_client_obj_recv(client, at_recv_buf, sizeof(at_recv_buf), 500);
        if (RT_NULL == strstr(at_recv_buf, cmd->precv))
        {
            if (RT_NULL == strstr(at_recv_buf, "+ok")) // 不同模块可能存在大小写问题
            {
                rt_kprintf("cur %s, string: %s not found. ^_^\r\n", at_recv_buf, cmd->precv);
                ret = RT_ERROR;
                break;
            }
        }
    }

    return ret;
}

/**
 * @brief  at模块初始化
 * @details
 * @param	None
 * @retval  None
 */
void at_module_init(void)
{
    for (at_module_t p = at_group; p < at_group + AT_MOUDLE_MAX; ++p)
    {
        RT_ASSERT(p->dev_name);
        RT_ASSERT(p->recv_bufsz);
        at_client_init(p->dev_name, p->recv_bufsz); // 初始化client
        p->client = at_client_get(p->dev_name);
        RT_ASSERT(p->client);
        if (p->client && p->client->parser) // rt-thread的一个线程无法挂起另外一个线程，用不到urc数据解析
        {
            p->resp = p->client->recv_line_buf;
            rt_thread_delete(p->client->parser);
        }
        p->state = at_standy;
    }
    // LOG_E("At module initialization completed.");
}

/**
 * @brief  at模块初始化
 * @details
 * @param	None
 * @retval  None
 */
static at_module_t get_at_module_object(char *module_name)
{
    for (at_module_t p = at_group; p < at_group + AT_MOUDLE_MAX; ++p)
    {
        if (rt_strncmp(module_name, p->dev_name, RT_NAME_MAX) == 0)
            return p;
    }

    return NULL;
}

/**
 * @brief  at模块进入命令模式
 * @details
 * @param	None
 * @retval  None
 */
static rt_err_t at_enter_cmd(at_module_t at)
{
    rt_err_t ret;

    /* backup modbus device RX indicate */
    at->at_temp_rx_ind = at->client->device->rx_indicate;
    if (at->client)
    {
        extern void set_at_client_rx(at_client_t client);
        set_at_client_rx(at->client);
        ret = at_enter_transparent_mode(at->client);
    }
    if (ret != RT_EOK)
    {
        LOG_RAW("@error: an error occurred while entering command mode. ^_^\r\n");
    }

    return ret;
}

/**
 * @brief   at状态机
 * @details
 * @param at 目at模块句柄
 * @param uart_dev 目标at模块串口
 * @retval  None
 */
void at_fsm(char *at_name)
{
    rt_err_t ret;
    rt_base_t level;

    RT_ASSERT(at_name);
    at_module_t at = get_at_module_object(at_name);
    rt_device_t dev = at->client->device; 
    // rt_device_find(at->dev_name);

    RT_ASSERT(at);
    RT_ASSERT(dev);

    switch (at->state)
    {
    case at_enter_transparent:
    {
        ret = at_enter_cmd(at);

        at->state = (RT_EOK == ret) ? at_continue : at_exe_failed;
    }
    break;
    case at_exit_transparent:
    {
        /* restore modbus device RX indicate */
        {
            level = rt_hw_interrupt_disable();
            rt_device_set_rx_indicate(dev, at->at_temp_rx_ind);
            rt_hw_interrupt_enable(level);
        }

        at->state = at_standy;
    }
    break;
    case at_continue:
    {
        RT_ASSERT(at->resp);
        at_cmd_t cmd = get_target_at_event(at, (enum at_cmd_id)at->event_id);
        if (RT_NULL == cmd || RT_NULL == cmd->event ||
            RT_NULL == cmd->psend)
            goto __exit;

        /* 发送数据到服务器，并接收响应数据存放在 resp 结构体中 */
        at_obj_exec_cmd(at->client, RT_NULL, cmd->psend);
        rt_memset(at->resp, 0x00, at->client->recv_bufsz);
        at_client_obj_recv(at->client, at->resp, at->client->recv_bufsz, 500);
        cmd->event(at->resp, at);
    }
    break;
    case at_exe_failed:
    case at_complete:
    {
    __exit:
        at->state = at_exit_transparent;
    }
    break;
    case at_cli:
    {
        ret = at_enter_cmd(at);
        at->state = at_exit_transparent;
    }
    break;
    case at_standy:
    default:
    {
        rt_thread_mdelay(100);
    }
    break;
    }
}

#ifdef FINSH_USING_MSH
#include <finsh.h>

/**
 * @brief  设置at进入配置模式/退出at模式
 * @param  
 * @retval None
 */
void set_at(int argc, char **argv)
{
#ifdef COMMM_TITLE
#undef COMMM_TITLE
#endif
#define COMMM_TITLE "Please input'set_at<(uartx)[(eg. uart1)]>'"

    if (argc < 2)
    {
        LOG_RAW("@error: " COMMM_TITLE ".\n");
        return;
    }
    if (argc > 2)
    {
        LOG_RAW("@error: parameter is too long," COMMM_TITLE ".\n");
        return;
    }


    for (at_module_t p = at_group; p < at_group + sizeof(at_group) / sizeof(at_group[0]); ++p)
    {
        if (!strcmp(argv[1], p->dev_name))
        {
            if (!strcmp(rt_console_get_device()->parent.name, p->dev_name))
            {
                rt_kprintf("The operating device[%s] is running the terminal.\r\n",
                           p->dev_name);
                return;
            }

            p->state = at_cli;
            return;
        }
    }

    LOG_RAW("At device: %s does not exist.\n", argv[1]);

#undef COMMM_TITLE
}
MSH_CMD_EXPORT(set_at, at module enters transparent mode.);
#endif


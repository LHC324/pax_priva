/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-09-15     LHC       the first version
 *  @verbatim
 *  使用： 1、用户需要完善"rt_small_modbus_init/MX_ModbusInit"、"Modbus_Send"函数
 *         2、用户需要定义"ModbusPools"寄存器池
 *         3、"rt_small_modbus_init/MX_ModbusInit"初始化时需要明确指定"UartHandle"参数
 *         4、"Modbus_CallBack"函数用户按需编写
 *         5、按需配置"small_modbus_cfg.h"
 */
#include "lhc_modbus_port.h"
#include <rtdevice.h>

#define __init_modbus(__name, __type, __master_id, __slave_id,                    \
                      __callback, __lock, __unlock, __ota_update, __error_handle, \
                      __transmit, __uart, __pools, __user_handle)                 \
    MdbusHandle __name##_lhc_modbus = {                                           \
        .type = __type,                                                           \
        .Master.id = __master_id,                                                 \
        .Slave.id = __slave_id,                                                   \
        .Mod_CallBack = __callback,                                               \
        .Mod_Lock = __lock,                                                       \
        .Mod_Unlock = __unlock,                                                   \
        .Mod_Ota = __ota_update,                                                  \
        .Mod_Error = __error_handle,                                              \
        .Mod_Transmit = __transmit,                                               \
        .Uart = __uart,                                                           \
        .pPools = &__pools,                                                       \
        .Slave.pHandle = __user_handle,                                           \
    };

/*定义Modbus对象*/
// pModbusHandle Cpu_Modbus_Object;
pModbusHandle Lte_Modbus_Object;
pModbusHandle Wifi_Modbus_Object;
pModbusHandle modbus_console_handle = RT_NULL;
static ModbusPools Spool;
/* 指向互斥量的指针 */
static rt_mutex_t modbus_mutex = RT_NULL;
rt_sem_t  console_sem = RT_NULL;

static void Modbus_CallBack(pModbusHandle pd, Function_Code code);
static void lhc_console_transfer(pModbusHandle pd);
static void Modbus_ErrorHadle(pModbusHandle pd, Lhc_Modbus_State_Code error_code);
#if (LHC_MODBUS_RTOS)
static void Modbus_Lock(void);
static void Modbus_UnLock(void);
#endif
static void Modbus_Send(pModbusHandle pd, enum Using_Crc crc);

#if (LHC_MODBUS_RTOS == 2)
#if (LHC_MODBUS_USING_MALLOC)
extern DMA_HandleTypeDef hdma_usart1_rx;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern UART_HandleTypeDef huart2;
// extern DMA_HandleTypeDef hdma_uart3_rx;
// extern UART_HandleTypeDef huart3;

/**
 * @brief  初始化Modbus协议库
 * @retval None
 */
int rt_lhc_modbus_init(void)
{
    /* 创建一个动态互斥量 */
    modbus_mutex = rt_mutex_create("modbus_mutex", RT_IPC_FLAG_PRIO);
    /* 创建控制信号量 */
    console_sem = rt_sem_create("console_sem", 0, RT_IPC_FLAG_FIFO);//RT_IPC_FLAG_PRIO
/*-------------------------------------------------------------------*/
    UartHandle lte_lhc_modbus_uart = {
        .huart = &huart2,
        .phdma = &hdma_usart2_rx,
#if (LHC_MODBUS_RTOS)
        .semaphore = NULL,
#else
        .recive_finish_flag = false,
#endif
        .tx = {
            .wmode = uart_using_it,
            .size = LHC_MODBUS_TX_BUF_SIZE,
            .count = 0,
            .pbuf = NULL,
        },
        .rx = {
            .wmode = uart_using_dma,
            .size = LHC_MODBUS_RX_BUF_SIZE,
            .count = 0,
            .pbuf = NULL,
        },
    };
    __init_modbus(lte, lhc_modbus_Slave, LHC_MODBUS_MASTER_ADDR, 0x02,
                  Modbus_CallBack, Modbus_Lock, Modbus_UnLock, lhc_console_transfer, Modbus_ErrorHadle,
                  Modbus_Send, lte_lhc_modbus_uart, Spool, NULL);
#if (LHC_MODBUS_RTOS == 2U)
    lte_lhc_modbus.dev = NULL;
    lte_lhc_modbus.old_console = NULL;
#endif
    create_lhc_modbus(&Lte_Modbus_Object, &lte_lhc_modbus);

/*-------------------------------------------------------------------*/
    UartHandle wifi_lhc_modbus_uart = {
        .huart = &huart2,
        .phdma = &hdma_usart2_rx,
#if (LHC_MODBUS_RTOS)
        .semaphore = NULL,
#else
        .recive_finish_flag = false,
#endif
        .tx = {
            .wmode = uart_using_it,
            .size = LHC_MODBUS_TX_BUF_SIZE,
            .count = 0,
            .pbuf = NULL,
        },
        .rx = {
            .wmode = uart_using_dma,
            .size = LHC_MODBUS_RX_BUF_SIZE,
            .count = 0,
            .pbuf = NULL,
        },
    };
    __init_modbus(wifi, lhc_modbus_Slave, LHC_MODBUS_MASTER_ADDR, 0x03,
                  Modbus_CallBack, Modbus_Lock, Modbus_UnLock, lhc_console_transfer, Modbus_ErrorHadle,
                  Modbus_Send, wifi_lhc_modbus_uart, Spool, NULL);
#if (LHC_MODBUS_RTOS == 2U)
    wifi_lhc_modbus.dev = NULL;
    wifi_lhc_modbus.old_console = NULL;
#endif
    create_lhc_modbus(&Wifi_Modbus_Object, &wifi_lhc_modbus);

    return 0;
}
/*在内核对象中初始化:https://blog.csdn.net/yang1111111112/article/details/93982354*/
INIT_DEVICE_EXPORT(rt_lhc_modbus_init);

/*主机使用example*/
/*void small_modbus_master_request(void)
{
    pModbusHandle pd = Modbus_Object;
    Request_HandleTypeDef request = {
        .code = ReadHoldReg,
        .reg_start_addr = 0x0000,
        .reg_len = 0x01,
    };

    if (pd)
    {
        lhc_tool_memcpy(&pd->Master.request_data, &request, sizeof(request));
        pd->Mod_Request(pd);
    }
}*/
#else
int rt_small_modbus_init(void)
{
    MdbusHandle temp_small_modbus = {
        /*User init info*/
    };
    create_lhc_modbus(&Modbus_Object, &temp_small_modbus);
    return 0;
}
#endif

#else
void MX_ModbusInit(void)
{
}

#endif

/**
 * @brief  modbus协议栈外部回调函数
 * @param  pd 需要初始化对象指针
 * @param  code 功能码
 * @retval None
 */
static void Modbus_CallBack(pModbusHandle pd, Function_Code code)
{
}

#if (LHC_MODBUS_RTOS)
/**
 * @brief  modbus协议栈加锁函数
 * @param  pd 需要初始化对象指针
 * @param  code 功能码
 * @retval None
 */
static void Modbus_Lock(void)
{
    rt_mutex_take(modbus_mutex, RT_WAITING_FOREVER);
}

/**
 * @brief  modbus协议栈解锁函数
 * @param  pd 需要初始化对象指针
 * @param  code 功能码
 * @retval None
 */
static void Modbus_UnLock(void)
{
    rt_mutex_release(modbus_mutex);
}
#endif

/**
 * @brief  modbus协议栈进行控制台转移
 * @param  pd modbus协议站句柄
 * @retval None
 */
static void lhc_console_transfer(pModbusHandle pd)
{
    extern void finsh_set_device(const char *device_name);
    rt_err_t ret = RT_EOK;
    if (NULL == pd || NULL == pd->Uart.huart ||
        NULL == pd->dev)
        return;
    /*@note 
            基于稳定性考虑，底板cpu暂时不支持finsh
            finsh设备底层适配RS485手动切换“DIR”不方便，需要修改源码
    */
    // if(Cpu_Modbus_Object == pd)
    // {
    //     LHC_MODBUS_DEBUG_R("The main CPU modbus does not support finsh transfer.");
    //     return;
    // }

    pd->old_rx_indicate = pd->dev->rx_indicate; //备份接收完成通知(串口关闭前)
    ret = rt_device_close(pd->dev);
    if ((RT_EOK == ret) && (RT_EOK == rt_sem_trytake(console_sem)))
    {
        modbus_console_handle = pd; //记录当前正在使用控制台的modbus句柄
        // rt_device_set_rx_indicate(pd->dev, RT_NULL); // 置空当前串口回调函数
        pd->old_console = rt_console_set_device(pd->dev->parent.name);
        finsh_set_device(pd->dev->parent.name);
        rt_kprintf("@note: enter finsh mode.\r\n");
    }
    else
        rt_kprintf("@error: finsh already locked.\r\n");
    // 没有挂起的必要：中断函数转移后将会永久挂起
}

/**
 * @brief  modbus协议栈接收帧错误处理
 * @param  pd 需要初始化对象指针
 * @param  error_code 错误码
 * @retval None
 */
static void Modbus_ErrorHadle(pModbusHandle pd, Lhc_Modbus_State_Code error_code)
{
    /*@note 主cpu切换一次finsh后，modbus接收的size会丢失，怀疑是rtt底层驱动有问题
    */
    if (pd && pd->dev) //解决数据帧错位后，连续错位问题
        rt_device_read(pd->dev, 0, pd->Uart.rx.pbuf, pd->Uart.rx.size);

    pd->old_rx_indicate = pd->dev->rx_indicate; //备份接收完成通知(串口关闭前)
    if(RT_EOK == rt_device_close(pd->dev))
    {
         /*挂接目标接收中断函数*/
        rt_device_set_rx_indicate(pd->dev, (rt_err_t(*)(rt_device_t, rt_size_t))pd->old_rx_indicate);
        rt_device_open(pd->dev, RT_DEVICE_FLAG_TX_BLOCKING | RT_DEVICE_FLAG_RX_NON_BLOCKING);
        LHC_MODBUS_DEBUG_R("device[%s] restarted successfully.", pd->dev->parent.name);
    }
    
}

/**
 * @brief  Modbus协议发送
 * @param  pd 需要初始化对象指针
 * @retval None
 */
static void Modbus_Send(pModbusHandle pd, enum Using_Crc crc)
{
#if (LHC_MODBUS_RTOS == 2U)
    if (NULL == pd || NULL == pd->dev)
#else
    if (NULL == pd || NULL == pd->Uart.huart)
#endif
        return;

    if (crc == lhc_used_crc)
    {
        uint16_t crc16 = get_crc16(lhc_modbus_tx_buf, lhc_modbus_tx_count(pd), 0xffff);

        lhc_tool_memcpy(&lhc_modbus_tx_buf[lhc_modbus_tx_count(pd)], (uint8_t *)&crc16, sizeof(crc16));
        lhc_modbus_tx_count(pd) += sizeof(crc16);
    }
    if ((pd->Uart.tx.wmode & uart_using_rs485) && pd->Uart.rs485->port)
        HAL_GPIO_WritePin(pd->Uart.rs485->port, pd->Uart.rs485->pin, GPIO_PIN_SET);
#if (LHC_MODBUS_RTOS == 2U)
    rt_device_write(pd->dev, 0, lhc_modbus_tx_buf, lhc_modbus_tx_count(pd));
#else
    switch (pd->Uart.tx.wmode)
    {
    case uart_using_it:
    {
        HAL_UART_Transmit((UART_HandleTypeDef *)pd->Uart.huart, lhc_modbus_tx_buf, lhc_modbus_tx_count(pd), 0x100);
    }
    break;
    case uart_using_dma:
    {
        HAL_UART_Transmit_DMA((UART_HandleTypeDef *)pd->Uart.huart, lhc_modbus_tx_buf, lhc_modbus_tx_count(pd));
        while (__HAL_UART_GET_FLAG((UART_HandleTypeDef *)pd->Uart.huart, UART_FLAG_TC) == RESET)
        {
        }
    }
    break;
    default:
        break;
    }
#endif
    if ((pd->Uart.tx.wmode & uart_using_rs485 )&& pd->Uart.rs485->port)
        HAL_GPIO_WritePin(pd->Uart.rs485->port, pd->Uart.rs485->pin, GPIO_PIN_RESET);
}

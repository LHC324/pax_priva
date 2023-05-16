/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-09-15     LHC       the first version
 */
#ifndef _INC_LHC_MODBUS_PORT_H_
#define _INC_LHC_MODBUS_PORT_H_
#ifdef __cplusplus
extern "C"
{
#endif
#include "lhc_modbus.h"

    // extern pModbusHandle Cpu_Modbus_Object;
    extern pModbusHandle Lte_Modbus_Object;
    extern pModbusHandle Wifi_Modbus_Object;
    extern pModbusHandle modbus_console_handle;

#if (LHC_MODBUS_RTOS == 2)
    extern int rt_small_modbus_init(void);
#else
extern void MX_ModbusInit(void);
#endif

#ifdef __cplusplus
}
#endif
#endif /* _INC_LHC_MODBUS_PORT_H_ */

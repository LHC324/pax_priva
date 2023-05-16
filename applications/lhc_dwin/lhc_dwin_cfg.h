/*
 * Copyright (c) 2006-2022, LHC Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-09-15     LHC       the first version
 */
#ifndef _INC_LHC_DWIN_CFG_H_
#define _INC_LHC_DWIN_CFG_H_
#ifdef __cplusplus
extern "C"
{
#endif
/*****************************************************功能配置区*******************************************/
/*Configuration Wizard：https://blog.csdn.net/qq_15647227/article/details/89297207*/
// <<< Use Configuration Wizard in Context Menu >>>

// <o>Selsect dwin thread Running Environment
//  <i>Default: 2
//  <0=> Bare pager
//  <1=> Freertos
//  <2=> rt_thread
/*迪文屏幕使用RTOS[0:不使用RTOS;1:Freertos;2:rt_thread]*/
#define LHC_DWIN_USING_RTOS 2 

// <o>Set Dwin Dynamic Memory allocation mode
//  <i>Default: 1
//  <0=> Unused
//  <1=> Used
/*迪文屏幕动态内存分配[0:不使用；1：使用]*/
#define LHC_DWIN_USING_MALLOC 1

// <o>Set Dwin Dma peripheral
//  <i>Default: 1
//  <0=> Unused
//  <1=> Used
/*迪文屏幕DMA选项*/
#define LHC_DWIN_USING_DMA 1

// <o>Set Dwin crc algorithm
//  <i>Default: 1
//  <0=> Unused
//  <1=> Used
/*迪文屏幕CRC校验*/
#define LHC_DWIN_USING_CRC 1

// <o>Selsect Dwin Debug Options
//  <i>Default: 2
//  <0=> Unused debug
//  <1=> leeter shell
//  <2=> finish shell
/*迪文屏幕调试选项[0:不使用调试终端；1：leeter shell; 2:finish shell]*/
#define LHC_DWIN_USING_DEBUG 2

// <o>View dwin receive buffer data
//  <i>Default: 0
//  <0=> not print
//  <1=> print
/*迪文屏幕查看接收缓冲区*/
#define LHC_DWIN_SEE_RX_BUFF 0
/*迪文屏幕调试输出终端选择:[0:不使用调试终端；1：leeter shell; 2:finish shell]*/
//#define DWIN_USING_SHELL 2

// <o>Set the buffer size of Dwin receiving thread
//  <i>Default: 128 (Unit: byte)
//  <0-1024>
/*迪文屏幕数据缓冲区尺寸*/
#define LHC_DWIN_RX_BUF_SIZE 128

// <o>Set the buffer size of Dwin sending thread
//  <i>Default: 512 (Unit: byte)
//  <0-1024>
#define LHC_DWIN_TX_BUF_SIZE 512
// <<< end of configuration section >>>

#if (LHC_DWIN_USING_MALLOC == 1)
#define lhc_dwin_malloc rt_malloc
#define lhc_dwin_free rt_free
#endif

#if (LHC_DWIN_USING_DEBUG == 1)
/*迪文屏幕调试接口*/
#define LHC_DWIN_DEBUG (fmt, ...) shellPrint(Shell_Object, fmt, ##__VA_ARGS__)
#elif (LHC_DWIN_USING_DEBUG == 2)
#undef DBG_TAG
#define DBG_TAG "lhc_dwin"
#define DBG_LVL DBG_LOG
/*必须位于DBG_SECTION_NAME之后*/
#include <rtdbg.h>
#define LHC_DWIN_DEBUG_R LOG_RAW
#define LHC_DWIN_DEBUG LOG_D
#define LHC_DWIN_HEX LOG_HEX
#else
#define LHC_DWIN_DEBUG
#endif

#ifdef __cplusplus
}
#endif
#endif /* _INC_LHC_DWIN_CFG_H_ */

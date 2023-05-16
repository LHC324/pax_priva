/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-09-15     LHC       the first version
 */
#ifndef _INC_PCF8563_H_
#define _INC_PCF8563_H_
#ifdef __cplusplus
extern "C"
{
#endif

#include <rthw.h>
#include <rtthread.h>
#include <rtdevice.h>

/* slave address */	
#define 	PCF8563_ADDR			0x51
//#define PCF8563_WRITE_ARR   0xA2
//#define PCF8563_ERAD_ARRD	  0xA3

/* register */
#define		REG_PCF8563_STATE1		0x00
#define		REG_PCF8563_STATE2		0x01
#define		REG_PCF8563_SEC			0x02
#define		REG_PCF8563_MIN			0x03
#define		REG_PCF8563_HOUR		0x04
#define		REG_PCF8563_DAY			0x05
#define		REG_PCF8563_WEEK		0x06
#define		REG_PCF8563_MON			0x07
#define		REG_PCF8563_YEAR		0x08
#define		REG_PCF8563_MIN_ALARM	0x09
#define		REG_PCF8563_HOUR_ALARM	0x0A
#define		REG_PCF8563_DAY_ALARM	0x0B
#define		REG_PCF8563_WEEK_ALARM	0x0C
#define		REG_PCF8563_CLKOUT		0x0D
#define		REG_PCF8563_TIMER		0x0E
#define		REG_PCF8563_CDOWN		0x0F

/* offset */
#define 	SHIELD_PCF8563_STATE1   (unsigned char)0xa8
#define 	SHIELD_PCF8563_STATE2   (unsigned char)0x1f
#define 	SHIELD_PCF8563_SEC      (unsigned char)0x7f
#define 	SHIELD_PCF8563_MIN      (unsigned char)0x7f
#define 	SHIELD_PCF8563_HOUR     (unsigned char)0x3f
#define 	SHIELD_PCF8563_DAY      (unsigned char)0x3f
#define 	SHIELD_PCF8563_WEEK     (unsigned char)0x07
#define 	SHIELD_PCF8563_MON      (unsigned char)0x1f
#define 	SHIELD_PCF8563_YEAR     (unsigned char)0xff

extern int rt_hw_pcf8563_init(void);


#ifdef __cplusplus
}
#endif
#endif /* _INC_PCF8563_H_ */

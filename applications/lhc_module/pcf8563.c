/*
 * File      : drv_pcf8563.c
 * This file is part of RT-Thread RTOS
 * COPYRIGHT (C) 2006 - 2018, RT-Thread Development Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Change Logs:
 * Date           Author       Notes
 * 2023-04-23     lhc          the first version
 */

#include <board.h>
#include <rtdevice.h>
#include "pcf8563.h"

#if RT_VER_NUM >= 0x40100
#include <sys/time.h>
#endif /*RT_VER_NUM >= 0x40100*/

#define DBG_TAG "drv.pcf8563"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>

// #define RT_USING_PCF8563

#ifdef RT_USING_PCF8563

#define PCF8563_I2C_BUS "i2c3"    /* i2c linked */
#define PCF8563_DEVICE_NAME "rtc" /* register device name */

struct pcf8563_device
{
    struct rt_device rtc_parent;
    struct rt_i2c_bus_device *i2c_device;
};
static struct pcf8563_device pcf8563_dev;

/* bcd to hex */
static unsigned char bcd_to_hex(unsigned char data)
{
    unsigned char temp;

    temp = ((data >> 4) * 10 + (data & 0x0f));
    return temp;
}

/* hex_to_bcd */
static unsigned char hex_to_bcd(unsigned char data)
{
    unsigned char temp;

    temp = (((data / 10) << 4) + (data % 10));
    return temp;
}

/* pcf8563 read register */
rt_err_t pcf8563_read_reg(rt_uint8_t reg, rt_uint8_t *data, rt_uint8_t data_size)
{
    struct rt_i2c_msg msg[2];

    msg[0].addr = PCF8563_ADDR;
    msg[0].flags = RT_I2C_WR;
    msg[0].len = 1;
    msg[0].buf = &reg;
    msg[1].addr = PCF8563_ADDR;
    msg[1].flags = RT_I2C_RD;
    msg[1].len = data_size;
    msg[1].buf = data;

    if (rt_i2c_transfer(pcf8563_dev.i2c_device, msg, 2) == 2)
    {
        return RT_EOK;
    }
    else
    {
        rt_kprintf("i2c bus read failed!\r\n");
        return -RT_ERROR;
    }
}

/* pcf8563 write register */
rt_err_t pcf8563_write_reg(rt_uint8_t reg, rt_uint8_t *data, rt_uint8_t data_size)
{
#ifndef BSP_USING_HARD_I2C
    struct rt_i2c_msg msg[2];

    msg[0].addr = PCF8563_ADDR;
    msg[0].flags = RT_I2C_WR;
    msg[0].len = 1;
    msg[0].buf = &reg;
    msg[1].addr = PCF8563_ADDR;
    msg[1].flags = RT_I2C_WR | RT_I2C_NO_START;
    msg[1].len = data_size;
    msg[1].buf = data;

    if (rt_i2c_transfer(pcf8563_dev.i2c_device, msg, 2) == 2)
#else
    struct rt_i2c_msg msg[1];
    rt_uint8_t *buf = (rt_uint8_t *)rt_malloc(data_size + 1U);

    if (buf == RT_NULL)
    {
        rt_free(buf);
        return -RT_ERROR;
    }

    buf[0] = reg;
    rt_memcpy(&buf[1], data, data_size);
    msg[0].addr = PCF8563_ADDR;
    msg[0].flags = RT_I2C_WR;
    msg[0].len = data_size + 1;
    msg[0].buf = buf;
    if (rt_i2c_transfer(pcf8563_dev.i2c_device, msg, 1) == 1)
#endif
    {
#ifdef BSP_USING_HARD_I2C
        rt_free(buf);
#endif
        return RT_EOK;
    }
    else
    {
        rt_kprintf("i2c bus write failed!\r\n");
        return -RT_ERROR;
    }
}

static rt_err_t rt_pcf8563_open(rt_device_t dev, rt_uint16_t flag)
{
    if (dev->rx_indicate != RT_NULL)
    {
        /* open interrupt */
    }

    return RT_EOK;
}

static rt_ssize_t rt_pcf8563_read(rt_device_t dev, rt_off_t pos, void *buffer, rt_size_t size)
{
    return RT_EOK;
}

static rt_err_t rt_pcf8563_control(rt_device_t dev, int cmd, void *args)
{
    rt_err_t ret = RT_EOK;
    time_t *time;
    struct tm time_temp;
    rt_uint8_t buff[7];

    RT_ASSERT(dev != RT_NULL);
    rt_memset(&time_temp, 0, sizeof(struct tm));

    switch (cmd)
    {
    case RT_DEVICE_CTRL_RTC_GET_TIME:
        time = (time_t *)args;
        ret = pcf8563_read_reg(REG_PCF8563_SEC, buff, 7);

        if (ret == RT_EOK)
        {
            time_temp.tm_year = bcd_to_hex(buff[6] & SHIELD_PCF8563_YEAR) + 2000 - 1900;
            time_temp.tm_mon = bcd_to_hex(buff[5] & SHIELD_PCF8563_MON) - 1;
            time_temp.tm_mday = bcd_to_hex(buff[3] & SHIELD_PCF8563_DAY);
            time_temp.tm_hour = bcd_to_hex(buff[2] & SHIELD_PCF8563_HOUR);
            time_temp.tm_min = bcd_to_hex(buff[1] & SHIELD_PCF8563_MIN);
            time_temp.tm_sec = bcd_to_hex(buff[0] & SHIELD_PCF8563_SEC);

            *time = mktime(&time_temp);
        }
        break;

    case RT_DEVICE_CTRL_RTC_SET_TIME:
    {
        struct tm *time_new;

        time = (time_t *)args;
        time_new = localtime(time);
        buff[6] = hex_to_bcd(time_new->tm_year + 1900 - 2000);
        buff[5] = hex_to_bcd(time_new->tm_mon + 1);
        buff[3] = hex_to_bcd(time_new->tm_mday);
        buff[4] = hex_to_bcd(time_new->tm_wday + 1);
        buff[2] = hex_to_bcd(time_new->tm_hour);
        buff[1] = hex_to_bcd(time_new->tm_min);
        buff[0] = hex_to_bcd(time_new->tm_sec);
        // LOG_HEX(DBG_TAG, 7, buff, 7);
        ret = pcf8563_write_reg(REG_PCF8563_SEC, buff, 7);
    }
    break;
#ifdef RT_USING_ALARM
    /* get alarm time */
    case RT_DEVICE_CTRL_RTC_GET_ALARM:
    {
        struct rt_rtc_wkalarm *alm_time;

        ret = pcf8563_read_reg(dev, REG_PCF8563_MIN_ALARM, buff, 4);
        if (ret == RT_EOK)
        {
            alm_time = (struct rt_rtc_wkalarm *)args;
            
            alm_time->tm_wday = bcd_to_hex(buff[3]);
            alm_time->tm_mday = bcd_to_hex(buff[2]);
            alm_time->tm_hour = bcd_to_hex(buff[1]);
            alm_time->tm_min = bcd_to_hex(buff[0]);
            alm_time->tm_sec = 0x00; //pcf8563不支持sec
        }
    }
    break;

    /* set alarm time */
    case RT_DEVICE_CTRL_RTC_SET_ALARM:
    {
        struct rt_rtc_wkalarm *alm_time;

        alm_time = (struct rt_rtc_wkalarm *)args;
        buff[3] = hex_to_bcd(alm_time->tm_wday);
        buff[2] = hex_to_bcd(alm_time->tm_mday);
        buff[1] = hex_to_bcd(alm_time->tm_hour);
        buff[0] = hex_to_bcd(alm_time->tm_min);
        ret = pcf8563_write_reg(dev, REG_PCF8563_MIN_ALARM, buff, 4);
    }
    break;
#endif
    default:
        break;
    }
    return RT_EOK;
}

/* pcf8563 device int  */
int rt_hw_pcf8563_init(void)
{
    struct rt_i2c_bus_device *i2c_device;
    uint8_t data;

    i2c_device = rt_i2c_bus_device_find(PCF8563_I2C_BUS);
    if (i2c_device == RT_NULL)
    {
#ifdef RT_USE_FINSH_DEBUG
        rt_kprintf("i2c bus device %s not found!\r\n", PCF8563_I2C_BUS);
#endif
        return 1;
    }
    pcf8563_dev.i2c_device = i2c_device;
    /* register rtc device */
    pcf8563_dev.rtc_parent.type = RT_Device_Class_RTC;
    pcf8563_dev.rtc_parent.init = RT_NULL;
    pcf8563_dev.rtc_parent.open = rt_pcf8563_open;
    pcf8563_dev.rtc_parent.close = RT_NULL;
    pcf8563_dev.rtc_parent.read = rt_pcf8563_read;
    pcf8563_dev.rtc_parent.write = RT_NULL;
    pcf8563_dev.rtc_parent.control = rt_pcf8563_control;
    pcf8563_dev.rtc_parent.user_data = RT_NULL; /* no private */
    rt_device_register(&pcf8563_dev.rtc_parent, PCF8563_DEVICE_NAME, RT_DEVICE_FLAG_RDWR);

    if (pcf8563_read_reg(REG_PCF8563_STATE1, &data, 1) != RT_EOK)
        return -RT_ERROR;

    rt_kprintf("REG_PCF8563_STATE1:%#x.\r\n", data);

    if (data & 0x20)
    {
        data = 0x08;
        if (pcf8563_write_reg(REG_PCF8563_STATE1, &data, 1) != RT_EOK) // runn mode
            return -RT_ERROR;
    }

    /* init pcf8563 */
    data = 0x7f; /* close clock out */
    if (pcf8563_write_reg(REG_PCF8563_CLKOUT, &data, 1) != RT_EOK)
    {
        return -RT_ERROR;
    }

    return RT_EOK;
}
INIT_DEVICE_EXPORT(rt_hw_pcf8563_init);
#endif

#ifdef FINSH_USING_MSH
#include <finsh.h>

/**
 * @brief  pcf8563设置内部寄存器
 * @param  
 * @retval None
 */
static void pcf8563(int argc, char **argv)
{
#define PCF8563_CMD_DESC "pcf8563\r\n\t-clk [0(close), 1, 32, 1024, 32768]\r\n"

    int state;
    if(argc < 3)
    {
        rt_kprintf("param error."PCF8563_CMD_DESC);
    }

    state = rt_strcmp("-clk", argv[1]) ? -1 : 0;

    if (state < 0)
    {
        rt_kprintf("[%s]not find.eg."PCF8563_CMD_DESC, argv[1]);
        return;
    }

    switch (state)
    {
    case 0:
    {
        int clk = atoi(argv[2]);

        uint8_t val = !clk             ? 0x7F
                      : (clk == 1)     ? 0xFC + 3
                      : (clk == 32)    ? 0xFC + 2
                      : (clk == 1024)  ? 0xFC + 1
                      : (clk == 32768) ? 0xFC + 0
                                       : 0xCC, rval = 0;

        if (0xCC == val)
        {
            rt_kprintf("freq[%d] absent." PCF8563_CMD_DESC, clk);
            return;
        }

        if (pcf8563_write_reg(REG_PCF8563_CLKOUT, (rt_uint8_t *)&val, 1) == RT_EOK)
        {
            pcf8563_read_reg(REG_PCF8563_CLKOUT, (rt_uint8_t *)&rval, 1);
            rt_kprintf("successfully set.\r\nset\tcur\r\n%#x\t%#x\r\n", val, rval);
            
        }
    }
    break;
    default:
    rt_kprintf("[%d] absent."PCF8563_CMD_DESC, state);
        break;
    }
}
MSH_CMD_EXPORT(pcf8563, eg. pcf8563 -clk [0(close) 1 32 1024 32768]);
#endif

/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-12-8      zylx         first version
 */

#ifndef _FAL_CFG_H_
#define _FAL_CFG_H_

#include <rtthread.h>
#include <board.h>

#define RT_APP_PART_ADDR (0x08008000)

extern const struct fal_flash_dev stm32_onchip_flash;

/* flash device table */
#define FAL_FLASH_DEV_TABLE  \
    {                        \
        &stm32_onchip_flash, \
    }
/* ====================== Partition Configuration ========================== */
#ifdef FAL_PART_HAS_TABLE_CFG

/* partition table */
#define FAL_PART_TABLE                                                                                   \
    {                                                                                                    \
        {FAL_PART_MAGIC_WROD, "bootloader", "onchip_flash", 0, 32 * 1024, 0},                            \
            {FAL_PART_MAGIC_WROD, "app", "onchip_flash", 32 * 1024, 192 * 1024, 0},                      \
            {FAL_PART_MAGIC_WROD, "download", "onchip_flash", (32 + 192) * 1024, 128 * 1024, 0},         \
            {FAL_PART_MAGIC_WROD, "filesystem", "onchip_flash", (32 + 192 + 128) * 1024, 64 * 1024, 0},  \
            {FAL_PART_MAGIC_WROD, "retain", "onchip_flash", (32 + 192 + 128 + 64) * 1024, 96 * 1024, 0}, \
    }
#endif /* FAL_PART_HAS_TABLE_CFG */
#endif /* _FAL_CFG_H_ */

/*
 * Copyright (c) 2006-2022,LHC Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-09-15     LHC       the first version
 */
#ifndef _INC_LHC_DWIN_PORT_H_
#define _INC_LHC_DWIN_PORT_H_
#ifdef __cplusplus
extern "C"
{
#endif
#include "lhc_dwin.h"

    extern pDwinHandle dwin_object;
#if (LHC_DWIN_USING_RTOS == 2)
    extern int rt_dwin_init(void);
#else
extern void MX_DwinInit(void);
#endif

/*迪文屏幕区别不同类型数据*/
#define DWIN_MUST_CHEANGE 0x80
#define DWIN_NOT_CHEANGE 0x00
/*迪文屏幕下发数据：带一位小数*/
#define DWIN_PARAM_OFFSET_SIT 10.0F

/*迪文屏幕页面*/
#define MAIN_PAGE 0x03
#define DIGITAL_INPUT_PAGE 0x04
#define DIFITAL_OUTPUT_PAGE 0x05
#define ANALOG_INPUT_PAGE 0x06
#define ANALOG_OUTPUT_PAGE 0x07
#define NONE_PAGE 0x08
#define COMMUNICATION_PAGE 0x0F
#define ERROR_PAGE 0x10
#define RESET_POEWR_NOTE_PAGE 28U
#define NEXT_DELAT_TIMES 50U

#define DWIN_SYSTEM_READ_RTC_ADDR 0x0010 // 迪文屏幕系统区读取rtc时间
#define DWIN_DI_OUTPUT_ADDR 0x1000       // 数字输出开始地址
#define DWIN_TEST_RESULT_ADDR 0x1002     // 测试结果呈现地址
#define DWIN_OVERCURRENT_CARTOON 0x1003  // 过流动画地址
#define DWIN_SET_FRE_REG_ADDR 0x2016     // 设置频率寄存器
#define DWIN_SET_PHASE_ADDR 0x2017       // 设置相位
#define DWIN_SET_PHASE_REG_ADDR 0x2018   // 设置相位寄存器
#define DWIN_SET_RANGE_ADDR 0x2019       // 设置幅度
#define DWIN_SET_START_GROUP_ADDR 0x201A // 设置开始组号
#define DWIN_SET_END_GROUP_ADDR 0x201B   // 设置结束组号

#define DWIN_SET_DEVE_FRE_ADDR 0x101E         // 设置开发者频率
#define DWIN_SET_VOLTAGE_SHIF 0x1020          // 电压偏差率
#define DWIN_SET_CURRENT_SHIF 0x1022          // 电流偏差率
#define DWIN_SAMPLING_CURRENT_CH0_ADDR 0x1024 // 0通道电流采样地址

#define DWIN_OPERATE_SHIFT_ADDR 0x2000      // 迪文手动/自动模式区分地址
#define DWIN_START_TEST_ADDR 0x2001         // 迪文自动/手动开始测试地址
#define DWIN_SET_USER_FRE_ADDR 0x2002       // 用户频率设置地址
#define DWIN_SET_DEVE_WAVE_ADDR 0x2003      // 开发者波形模式选择地址
#define DWIN_WIFI_SWITCH_ADDR 0x2004        // WiFi模块开关地址
#define DWIN_WIFI_REFACTORY_ADDR 0x2005     // WIFI恢复出厂设置地址
#define DWIN_WIFI_WORK_MODE_ADDR 0x2006     // WIFI工作模式地址
#define DWIN_WIFI_SET_BAUD_ADDR 0x2007      // WIFI串口波特率设置地址
#define DWIN_WIFI_SET_DATA_BIT_ADDR 0x2008  // WIFI串口数据位设置地址
#define DWIN_WIFI_SET_STOP_BIT_ADDR 0x2009  // WIFI串口停止位设置地址
#define DWIN_WIFI_SET_CHECK_BIT_ADDR 0x200A // WIFI串口校验位设置地址
#define DWIN_WIFI_DATA_EXPORT_ADDR 0x200B   // WIFI串口数据导出地址
#define DWIN_USER_DATA_CLEAN_ADDR 0x200C    // 用户数据清除地址
#define DWIN_SAVE_DATA_ADDR 0x200D          // 迪文保存数据地址
#define DWIN_SAVE_PARAM_ADDR 0x200E         // 迪文保存参数地址
#define DWIN_GET_NET_TIMES_ADDR 0x200F      // 网络时间获取地址
#define DWIN_REQUEST_TIMES_ADDR 0x2010      // 迪文屏幕时间下发地址:4bit 16bit地址
#define DWIN_DEVELOPER_MODE_ADDR 0x2015     // 迪文屏幕开发者模式
#define DWIN_OVERCURRNRT_RESET_ADDR 0x201C  // 迪文屏幕过流复位按钮

#define DWIN_USER_NAME_ADDR 0x3000     // 用户名
#define DWIN_USER_PASSWORD_ADDR 0x3001 // 用户密码
#define DWIN_LOGIN_OPERATE_ADDR 0x3002 // 登录界面操作地址（登录/注销）
#define DWIN_ERROR_NOTE_ADDR 0x3003    // 错误图标提示地址

#define DWIN_SURE_CODE 0x00F1   // 设置确认键值
#define DWIN_CANCEL_CODE 0x00F0 // 注销键值

#ifdef __cplusplus
}
#endif
#endif /* _INC_LHC_DWIN_PORT_H_ */

/*USING_MALLOCUSING_MALLOC
 * Dwin.h
 *
 *  Created on: Nov 19, 2020
 *      Author: play
 */

#ifndef _INC_LHC_DWIN_H_
#define _INC_LHC_DWIN_H_
#ifdef __cplusplus
extern "C"
{
#endif

#include "lhc_dwin_cfg.h"
#include "lhc_tool.h"
#if (LHC_DWIN_USING_RTOS == 1)
// #include "cmsis_os.h"
// #include "shell_port.h"
#else
// #include <rtthread.h>
// #include <rtdbg.h>
#endif

/*****************************************************迪文屏幕特殊指令区*******************************************/
/*迪文屏幕写指令*/
#define DWIN_WRITE_CMD 0x82
/*迪文屏幕读指令*/
#define DWIN_READ_CMD 0x83
/*迪文屏幕页面切换指令*/
#define DWIN_PAGE_CHANGE_CMD 0x84
/*迪文屏幕触摸指令*/
#define DWIN_TOUCH_CMD 0xD4

	typedef enum
	{
		err_frame_head = 0,	 /*数据帧头错误*/
		err_check_code,		 /*校验码错误*/
		err_event,			 /*事件错误*/
		err_max_upper_limit, /*超出数据上限*/
		err_min_lower_limit, /*低于数据下限*/
		err_data_len,		 /*数据长度错误*/
		err_data_type,		 /*数据类型错误*/
		err_other,			 /*其他错误*/
		dwin_ok,
	} dwin_result;

	// typedef enum
	// {
	// 	dw_uint8_t,
	// 	dw_int8_t,
	// 	dw_uint16_t,
	// 	dw_int16_t,
	// 	dw_uint32_t,
	// 	dw_int32_t,
	// 	dw_float,
	// 	dw_long,
	// 	dw_ulong,
	// 	dw_type_max,
	// } dwin_data_type;

	typedef struct
	{
		uint16_t addr;
		uint16_t index;
		// void *val;
		// dwin_data_type type;
		// comm_val_t *pval;
		uint8_t site; // 数据开始位置
		float ratio;  // 小数位数
	} dwin_val_glue_t;

	typedef struct
	{
		union
		{
			unsigned char uc8;
			char c8;
			unsigned short us16;
			short s16;
			unsigned int ui32;
			int i32;
			float f32;
			long l32;
		} data;
		uint16_t size;
		dwin_result result;
	} dwin_data_t; // adapter

	typedef struct Dwin_HandleTypeDef *pDwinHandle;
	typedef struct Dwin_HandleTypeDef DwinHandle;

	struct Dwin_HandleTypeDef
	{
		uint8_t Id;
#if (LHC_DWIN_USING_RTOS == 2U)
		/*对于rt thread版本，增加一个设备描述指针*/
		rt_device_t dev, old_console;
		void *old_rx_indicate;
#endif
		void (*Dw_Transmit)(pDwinHandle);
#if (LHC_DWIN_USING_RTOS == 1U)
		void (*Dw_Recive)(void *);
#endif
		void (*Dw_Write)(pDwinHandle, uint16_t, uint8_t *, uint16_t);
		void (*Dw_Read)(pDwinHandle, uint16_t, uint8_t);
		void (*Dw_Page)(pDwinHandle, uint16_t);
		void (*Dw_Poll)(pDwinHandle);
		short int (*Dw_GetSignedData)(pDwinHandle, unsigned short int);
		void (*Dw_Error)(pDwinHandle, dwin_result, uint8_t, void *);
		void (*Dw_Delay)(uint32_t);
		struct dwin
		{
			dwin_val_glue_t *ptable;
			uint16_t num;
		} user_val;

		struct
		{
			void *pHandle;
		} Master;
		struct
		{
			/*预留外部数据结构接口*/
			void *pHandle;
			Event_Map *pMap;
			uint16_t Events_Size;
		} Slave;
		UartHandle Uart;
	} __attribute__((aligned(4)));

/*带上(pd)解决多级指针解引用问题：(*pd)、(**pd)*/
#define lhc_dwin_tx_size(pd) ((pd)->Uart.tx.size)
#define lhc_dwin_rx_size(pd) ((pd)->Uart.rx.size)
#define lhc_dwin_tx_count(pd) ((pd)->Uart.tx.count)
#define lhc_dwin_rx_count(pd) ((pd)->Uart.rx.count)
#define lhc_dwin_tx_buf ((pd)->Uart.tx.pbuf)
#define lhc_dwin_rx_buf ((pd)->Uart.rx.pbuf)

/*83指令返回数据以一个字为基础*/
#define LHC_DWIN_WORD 1U
#define LHC_DWIN_DWORD 2U
/*获取迪文屏幕数据*/
#define lhc_get_dwin_data(__pbuf, __s, __size)                                   \
	((__size) < 2U ? ((__pbuf[__s] << 8U) |                                  \
					  (__pbuf[__s + 1U]))                                    \
				   : ((__pbuf[__s] << 24U) |                                 \
					  (__pbuf[__s + 1U] << 16U) | (__pbuf[__s + 2U] << 8U) | \
					  (__pbuf[__s + 3U])))

#if (LHC_DWIN_USING_MALLOC == 0)
#define init_dwin(id) (static DwinHandle Connect_Str(Dwin_Object, id))
#define get_dwin(id) (&Connect_Str(Dwin_Object, id))
#else
#define lhc_dwin_handler(obj) (obj->Dw_Poll(obj))
extern void create_lhc_dwin(pDwinHandle *pd, pDwinHandle ps);
extern void free_lhc_dwin(pDwinHandle *pd);
#endif
#ifdef __cplusplus
}
#endif
#endif /* _INC_LHC_DWIN_H_ */

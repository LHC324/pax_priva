/*
 * Dwin.c
 *
 *  Created on: 2022年3月25日
 *      Author: LHC
 */

#include "lhc_dwin.h"

static void Dwin_Write(pDwinHandle pd, uint16_t start_addr, uint8_t *dat, uint16_t len);
static void Dwin_Read(pDwinHandle pd, uint16_t start_addr, uint8_t words);
static void Dwin_PageChange(pDwinHandle pd, uint16_t page);
static short int Dwin_GetSignedData(pDwinHandle pd, uint16_t udata);
static void Dwin_Poll(pDwinHandle pd);

/**
 * @brief  创建迪文屏幕对象(动态创建)
 * @param  pd 需要初始化对象指针
 * @param  ps 初始化数据指针
 * @retval None
 */
void create_lhc_dwin(pDwinHandle *pd, pDwinHandle ps)
{
	if (!ps)
		return;
#if (LHC_DWIN_USING_MALLOC)
	(*pd) = (pDwinHandle)lhc_dwin_malloc(sizeof(DwinHandle));
	if (!(*pd))
	{
		lhc_dwin_free(*pd);
		return;
	}
	uint8_t *pTxbuf = (uint8_t *)lhc_dwin_malloc(lhc_dwin_tx_size(ps));
	if (!pTxbuf)
	{
		lhc_dwin_free(pTxbuf);
		return;
	}
	uint8_t *pRxbuf = (uint8_t *)lhc_dwin_malloc(lhc_dwin_rx_size(ps));
	if (!pRxbuf)
	{
		lhc_dwin_free(pRxbuf);
		return;
	}
#else
	static uint8_t pTxbuf[lhc_dwin_tx_size(ps)];
	static uint8_t pRxbuf[lhc_dwin_rx_size(ps)];
#endif
	lhc_tool_memset(pTxbuf, 0x00, lhc_dwin_tx_size(ps));
	lhc_tool_memset(pRxbuf, 0x00, lhc_dwin_rx_size(ps));
#if (LHC_DWIN_USING_DEBUG)
	LHC_DWIN_DEBUG("@note:DwinObject[%d] = 0x%p", ps->Id, *pd);
#endif

	(*pd)->Id = ps->Id;
	(*pd)->Dw_Transmit = ps->Dw_Transmit;
#if (LHC_DWIN_USING_DMA)
#if (TOOL_USING_STM32HAL)
	(*pd)->Dw_Recive = (void (*)(void *))uartx_recive_handle;
#else
	(*pd)->Dw_Recive = Dw_Recive;
#endif
#endif
	(*pd)->Dw_Delay = ps->Dw_Delay;
	(*pd)->Dw_Error = ps->Dw_Error;
	(*pd)->Dw_Write = Dwin_Write;
	(*pd)->Dw_Read = Dwin_Read;
	(*pd)->Dw_Page = Dwin_PageChange;
	(*pd)->Dw_GetSignedData = Dwin_GetSignedData;
	(*pd)->Dw_Poll = Dwin_Poll;

	if (!ps->Uart.tx.pbuf)
	{
		ps->Uart.tx.pbuf = pTxbuf;
	}
	if (!ps->Uart.rx.pbuf)
	{
		ps->Uart.rx.pbuf = pRxbuf;
	}
	lhc_tool_memcpy(&(*pd)->Uart, &ps->Uart, sizeof(UartHandle));
	lhc_tool_memcpy(&(*pd)->user_val, &ps->user_val, sizeof(ps->user_val));
	lhc_tool_memcpy(&(*pd)->Master, &ps->Master, sizeof(ps->Master));
	lhc_tool_memcpy(&(*pd)->Slave, &ps->Slave, sizeof(ps->Slave));
}

/**
 * @brief  销毁迪文屏幕对象
 * @param  pd 需要初始化对象指针
 * @retval None
 */
void free_lhc_dwin(pDwinHandle *pd)
{
	if (*pd)
	{
		lhc_dwin_free((*pd)->Uart.tx.pbuf);
		lhc_dwin_free((*pd)->Uart.rx.pbuf);
		lhc_dwin_free((*pd));
	}
}

/**
 * @brief  写数据变量到指定地址并显示
 * @param  pd 迪文屏幕对象句柄
 * @param  start_addr 开始地址
 * @param  dat 指向数据的指针
 * @param  length 数据长度
 * @retval None
 */
static void Dwin_Write(pDwinHandle pd, uint16_t start_addr, uint8_t *pdat, uint16_t len)
{
#if (LHC_DWIN_USING_CRC)
	uint8_t buf[] = {
		0x5A, 0xA5, len + 3U + 2U, DWIN_WRITE_CMD, start_addr >> 8U,
		start_addr};
#else
	uint8_t buf[] = {
		0x5A, 0xA5, len + 3U, DWIN_WRITE_CMD, start_addr >> 8U,
		start_addr};
#endif
	lhc_dwin_tx_count(pd) = 0U;
	lhc_tool_memcpy(lhc_dwin_tx_buf, buf, sizeof(buf));
	lhc_dwin_tx_count(pd) += sizeof(buf);
	lhc_tool_memcpy(&lhc_dwin_tx_buf[lhc_dwin_tx_count(pd)], pdat, len);
	lhc_dwin_tx_count(pd) += len;
#if (LHC_DWIN_USING_DEBUG)
	// shellPrint(Shell_Object, "pd = %p, lhc_dwin_tx_count(pd) = %d.\r\n", pd, lhc_dwin_tx_count(pd));
	// shellPrint(Shell_Object, "lhc_dwin_tx_buf = %s.\r\n", lhc_dwin_tx_buf);
#endif

	pd->Dw_Transmit(pd);
}

/**
 * @brief  读出指定地址指定长度数据
 * @param  pd 迪文屏幕对象句柄
 * @param  start_addr 开始地址
 * @param  dat 指向数据的指针
 * @param  length 数据长度
 * @retval None
 */
static void Dwin_Read(pDwinHandle pd, uint16_t start_addr, uint8_t words)
{
#if (LHC_DWIN_USING_CRC)
	uint8_t buf[] = {
		0x5A, 0xA5, 0x04 + 2U, DWIN_READ_CMD, start_addr >> 8U,
		start_addr, words};
#else
	uint8_t buf[] = {
		0x5A, 0xA5, 0x04, DWIN_READ_CMD, start_addr >> 8U,
		start_addr, words};
#endif
	lhc_dwin_tx_count(pd) = 0U;
	lhc_tool_memcpy(lhc_dwin_tx_buf, buf, sizeof(buf));
	lhc_dwin_tx_count(pd) += sizeof(buf);

	// Dwin_Send(pd);
	pd->Dw_Transmit(pd);
}

/**
 * @brief  迪文屏幕指定页面切换
 * @param  pd 迪文屏幕对象句柄
 * @param  page 目标页面
 * @retval None
 */
static void Dwin_PageChange(pDwinHandle pd, uint16_t page)
{
#if (LHC_DWIN_USING_CRC)
	uint8_t buf[] = {
		0x5A, 0xA5, 0x07 + 2U, DWIN_WRITE_CMD, 0x00, 0x84, 0x5A, 0x01,
		page >> 8U, page};
#else
	uint8_t buf[] = {
		0x5A, 0xA5, 0x07, DWIN_WRITE_CMD, 0x00, 0x84, 0x5A, 0x01,
		page >> 8U, page};
#endif
	lhc_dwin_tx_count(pd) = 0U;
	lhc_tool_memcpy(lhc_dwin_tx_buf, buf, sizeof(buf));
	lhc_dwin_tx_count(pd) += sizeof(buf);

	pd->Dw_Transmit(pd);
}

/**
 * @brief  迪文屏幕识别16bit数据的正负
 * @param  pd 迪文屏幕对象句柄
 * @param  page 目标页面
 * @retval None
 */
static short int Dwin_GetSignedData(pDwinHandle pd, uint16_t udata)
{
	(void)pd;

	short int sdata = 0;
	/*识别负数*/
	if (udata & 0x1000)
		sdata = -1 * (short int)(~udata + 1U);
	else
		sdata = (short int)udata;

	return sdata;
}

/**
 * @brief  Dwin接收数据帧合法性检查
 * @param  pd modbus对象句柄
 * @retval dwin_result 检验结果
 */
static dwin_result Dwin_Recive_Check(pDwinHandle pd)
{
#define DWIN_MIN_FRAME_LEN 5U // 3个前导码+2个crc16
#if (LHC_DWIN_USING_DEBUG && LHC_DWIN_SEE_RX_BUFF)
	// LHC_DWIN_DEBUG("\r\ndwin_rx_buf[%d]:", lhc_dwin_rx_count(pd));
	// for (uint8_t i = 0; i < lhc_dwin_rx_count(pd); i++)
	// {
	// 	LHC_DWIN_DEBUG_R("%02X ", lhc_dwin_rx_buf[i]);
	// }
	// LHC_DWIN_DEBUG_R("\r\n\r\n");
	LHC_DWIN_HEX(DBG_TAG, 16, lhc_dwin_rx_buf, lhc_dwin_rx_count(pd));
	
#endif
	/*检查帧头*/
	if ((lhc_dwin_rx_buf[0] != 0x5A) || (lhc_dwin_rx_buf[1] != 0xA5))
	{
#if (LHC_DWIN_USING_DEBUG)
		LHC_DWIN_DEBUG("@error:Protocol frame header error.\r\n");
#endif
#if (!LHC_DWIN_SEE_RX_BUFF)
		// LHC_DWIN_DEBUG_R("lhc_dwin_rx_buf[%d]:", lhc_dwin_rx_count(pd));
		// for (uint8_t i = 0; i < lhc_dwin_rx_count(pd); i++)
		// {
		// 	LHC_DWIN_DEBUG_R("%02X ", lhc_dwin_rx_buf[i]);
		// }
		// LHC_DWIN_DEBUG_R("\r\n\r\n");
		LHC_DWIN_HEX(DBG_TAG, 16, lhc_dwin_rx_buf, lhc_dwin_rx_count(pd));
#endif
		return err_frame_head;
	}
	/*检查接收数据的尺寸*/
	if ((lhc_dwin_rx_count(pd) >= pd->Uart.rx.size) ||
		(lhc_dwin_rx_count(pd) < DWIN_MIN_FRAME_LEN))
	{
#if (LHC_DWIN_USING_DEBUG)
		LHC_DWIN_DEBUG("@error:Data length error,cur_len: %d.\r\n", lhc_dwin_rx_count(pd));
#endif
		return err_data_len;
	}
	/*检查crc校验码*/
	uint16_t crc16 = get_crc16(&lhc_dwin_rx_buf[3U], lhc_dwin_rx_count(pd) - 5U, 0xFFFF);
	if (crc16 == lhc_get_dwin_data(lhc_dwin_rx_buf, lhc_dwin_rx_count(pd) - 2U, LHC_DWIN_WORD))
	{
#if (LHC_DWIN_USING_DEBUG)
		LHC_DWIN_DEBUG("@error:crc check code error.\r\n");
		LHC_DWIN_DEBUG("dwin_rxcount = %d,crc16 = 0x%X.\r\n", lhc_dwin_rx_count(pd),
				   (uint16_t)((crc16 >> 8U) | (crc16 << 8U)));
#endif
		return err_check_code;
	}

	return dwin_ok;
#undef DWIN_MIN_FRAME_LEN
}

/**
 * @brief  迪文屏幕接收数据解析
 * @param  pd 迪文屏幕对象句柄
 * @retval None
 */
static void Dwin_Poll(pDwinHandle pd)
{
	if (pd == NULL)
		return;
	dwin_result dwin_code = Dwin_Recive_Check(pd);
	if (dwin_code != dwin_ok)
		return;
	uint16_t addr = lhc_get_dwin_data(lhc_dwin_rx_buf, 4U, LHC_DWIN_WORD);

	for (uint8_t i = 0; i < pd->Slave.Events_Size; i++)
	{
		if (pd->Slave.pMap[i].addr == addr)
		{
			if (pd->Slave.pMap[i].event)
				pd->Slave.pMap[i].event(pd, i, addr);
			break;
		}
	}

	lhc_tool_memset(lhc_dwin_rx_buf, 0x00, lhc_dwin_rx_count(pd));
	lhc_dwin_rx_count(pd) = 0U;
}

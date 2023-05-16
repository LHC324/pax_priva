/*
 * ModbusSlave.h
 *
 *  Created on: 2022年04月08日
 *      Author: LHC
 */

#ifndef INC_LHC_MODBUS_H_
#define INC_LHC_MODBUS_H_
#ifdef __cplusplus
extern "C"
{
#endif

#include "lhc_modbus_cfg.h"
#include "lhc_tool.h"

	/*错误码状态*/
	typedef enum
	{
		lhc_mod_ok = 0,		/*正常*/
		lhc_mod_err_config, /*配置错误*/
		lhc_mod_err_len,	/*数据帧长错误*/
		lhc_mod_err_id,		/*地址号错误*/
		lhc_mod_err_crc,	/*校验码错误*/
		lhc_mod_err_cmd,	/*不支持的功能码*/
		lhc_mod_err_addr,	/*寄存器地址错误*/
		lhc_mod_err_value,	/*数据值域错误*/
		lhc_mod_err_write,	/*写入失败*/
	} Lhc_Modbus_State_Code;

	typedef enum
	{
		lhc_modbus_Master = 0x00,
		lhc_modbus_Slave,
	} lhc_modbus_type;
	typedef enum
	{
		Coil = 0x00,
		InputCoil,
		InputRegister,
		HoldRegister,
		NullRegister,
	} Regsiter_Type __attribute__((aligned(4)));

	typedef enum
	{
		ReadCoil = 0x01,
		ReadInputCoil = 0x02,
		ReadHoldReg = 0x03,
		ReadInputReg = 0x04,
		WriteCoil = 0x05,
		WriteHoldReg = 0x06,
		WriteCoils = 0x0F,
		WriteHoldRegs = 0x10,
		ReportSeverId = 0x11,

	} Function_Code __attribute__((aligned(4)));

	typedef enum
	{
		lhc_modbus_read = 0x00,
		lhc_modbus_write,
	} Regsiter_Operate __attribute__((aligned(4)));

	enum Using_Crc
	{
		lhc_used_crc,
		lhc_not_used_crc
	};

	typedef struct Modbus_HandleTypeDef *pModbusHandle;
	typedef struct Modbus_HandleTypeDef MdbusHandle;
#if defined(LHC_MODBUS_USING_MASTER)
	typedef struct
	{
		Function_Code code;
		uint16_t reg_len;
		uint16_t reg_start_addr;
	} Request_HandleTypeDef;
#endif
	typedef struct
	{
#if defined(LHC_MODBUS_USING_COIL)
		uint8_t Coils[LHC_MODBUS_REG_POOL_SIZE];
#endif
#if defined(LHC_MODBUS_USING_INPUT_COIL)
		uint8_t InputCoils[LHC_MODBUS_REG_POOL_SIZE];
#endif
#if defined(LHC_MODBUS_USING_INPUT_REGISTER)
		uint16_t InputRegister[LHC_MODBUS_REG_POOL_SIZE * 2U];
#endif
#if defined(LHC_MODBUS_USING_HOLD_REGISTER)
		uint16_t HoldRegister[LHC_MODBUS_REG_POOL_SIZE * 2U];
#endif
	} ModbusPools __attribute__((aligned(4)));

	typedef struct
	{
		Regsiter_Type type;
		void *registers;
	} Pool __attribute__((aligned(4)));

	struct Modbus_HandleTypeDef
	{
#if (LHC_MODBUS_RTOS == 2U)
		/*对于rt thread版本，增加一个设备描述指针*/
		// char dev_name[32U];
		rt_device_t dev, old_console;
		void *old_rx_indicate;
#endif
		uint8_t code;
		lhc_modbus_type type;
		void (*Mod_CallBack)(pModbusHandle, Function_Code);
		void (*Mod_Recive)(void *);
		void (*Mod_Poll)(pModbusHandle);
		void (*Mod_Transmit)(pModbusHandle, enum Using_Crc);
#if defined(LHC_MODBUS_USING_MASTER)
		void (*Mod_Code46H)(pModbusHandle, uint16_t, uint8_t *, uint8_t);
		void (*Mod_Request)(pModbusHandle);
#endif
		bool (*Mod_Operatex)(pModbusHandle, Regsiter_Type, Regsiter_Operate, uint16_t, uint8_t *, uint8_t);
#if (LHC_MODBUS_RTOS)
		void (*Mod_Lock)(void);
		void (*Mod_Unlock)(void);
#endif
		// void (*Mod_ReportSeverId)(pModbusHandle);
		void (*Mod_Error)(pModbusHandle, Lhc_Modbus_State_Code);
		void (*Mod_Ota)(pModbusHandle);
#if defined(LHC_MODBUS_USING_MASTER)
		struct
		{
			/*主机id：可变*/
			uint8_t id;
			Request_HandleTypeDef request_data;
			void *pHandle;
		} Master;
#endif
		struct
		{
			/*从机id*/
			uint8_t id;
			/*预留外部数据结构接口*/
			void *pHandle;
		} Slave;
		ModbusPools *pPools;
		UartHandle Uart;
	} __attribute__((aligned(4)));

#define LHC_MODBUS_POOL_SIZE sizeof(ModbusPools)

/*带上(pd)解决多级指针解引用问题：(*pd)、(**pd)*/
#define lhc_modbus_tx_size(pd) ((pd)->Uart.tx.size)
#define lhc_modbus_rx_size(pd) ((pd)->Uart.rx.size)
#define lhc_modbus_tx_count(pd) ((pd)->Uart.tx.count)
#define lhc_modbus_rx_count(pd) ((pd)->Uart.rx.count)
#define lhc_modbus_tx_buf ((pd)->Uart.tx.pbuf)
#define lhc_modbus_rx_buf ((pd)->Uart.rx.pbuf)

#define LHC_MODBUS_WORD 1U
#define LHC_MODBUS_DWORD 2U
/*获取主机号*/
#define get_lhc_modbus_id() (lhc_modbus_rx_buf[0U])
/*获取Modbus功能号*/
#define get_lhc_modbus_fun_code() (lhc_modbus_rx_buf[1U])
/*获取Modbus协议数据*/
#define get_lhc_modbus_data(__pbuf, __s, __size)                             \
	((__size) < 2U ? ((__pbuf[__s] << 8U) |                                  \
					  (__pbuf[__s + 1U]))                                    \
				   : ((__pbuf[__s] << 24U) |                                 \
					  (__pbuf[__s + 1U] << 16U) | (__pbuf[__s + 2U] << 8U) | \
					  (__pbuf[__s + 3U])))

#if (LHC_MODBUS_USING_MALLOC == 0)
#define Get_SmallModbus_Object(id) (static MdbusHandle Connect_Str(Modbus_Object, id))
#define get_lhc_modbus_object(id) (&Connect_Str(Modbus_Object, id))
#else
#define lhc_modbus_handler(obj) (obj->Mod_Poll(obj))
extern void create_lhc_modbus(pModbusHandle *pd, pModbusHandle ps);
extern void free_lhc_modbus(pModbusHandle *pd);
#endif

#ifdef __cplusplus
}
#endif
#endif /* INC_LHC_MODBUS_H_ */

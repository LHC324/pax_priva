#ifndef INC_LHC_LORA_H_ 
#define INC_LHC_LORA_H_ 

#ifdef __cplusplus
extern "C"
{
#endif
#include "lhc_tool.h"

#define LORA_USING_DEBUG 0
#if (TOOL_USING_RTOS == 1)
#define LORA_SHELL &shell
#define LORA_DEBUG(fmt, ...) shellPrint(LORA_SHELL, fmt, ##__VA_ARGS__)
#define lhc_lora_malloc pvPortMalloc
#define lhc_lora_free vPortFree
#define lora_delay osDelay
#define lhc_lora_list_init vListInitialise
#define lhc_lora_list_t List_t
#define lhc_loara_list_item_t ListItem_t
#elif(TOOL_USING_RTOS == 2)
#define LORA_DEBUG(fmt, ...) rt_kprintf(fmt, ##__VA_ARGS__)
#define lhc_lora_malloc rt_malloc
#define lhc_lora_free rt_free
#define lhc_lora_delay rt_thread_mdelay
#define lhc_lora_list_init rt_list_init
#define lhc_lora_list_t struct lora_list_node
#define lhc_loara_list_item_t struct lora_list_node
#endif

#define lhc_lora_memcpy lhc_tool_memcpy
#define lhc_lora_memset lhc_tool_memset


#define USING_FREERTOS
/*lora使用中继模式*/
#define USING_REPEATER_MODE
/*1级中继地址*/
#define USING_REPEATER_1_ADDR (0x01)
/*支持的最大从机*/
#define SLAVE_MAX_NUMBER 16U
#define LORA_TX_BUF_SIZE 64U
#define LORA_RX_BUF_SIZE 128U
/*A型从机地址*/
// #define A_TYPE_SLAVE_MIN_ADDR 1U
// #define A_TYPE_SLAVE_MAX_ADDR 4U
// #define B_TYPE_SLAVE_START_ADDR 5U
// #define LORA_AI_SLAVE_START_ADDR 500U
/*不同类型从机的数量*/
#define LORA_DI_SLAVE_MAX_NUM 4U
#define LORA_D0_SLAVE_MAX_NUM 8U
#define LORA_AI_SLAVE_MAX_NUM 4U
#define LORA_AO_SLAVE_MAX_NUM 0U
/*A型从机数字输入路数*/
#define DIGITAL_INPUT_NUMBERS 8U
#define DIGITAL_INPUT_OFFSET 1U
#define DIGITAL_OUTPUT_OFFSET (LORA_DI_SLAVE_MAX_NUM + 1U)

#define ANALOG_INPUT_NUMBERS 8U
/*Lora模块调度时间:不宜太短，太短速率低时数据容易丢包*/
#define LORA_SCHEDULE_TIMES 200U
/*Lora模块无效ID号*/
#define LORA_NULL_ID 0xFF
/*Lora从机离线/在线情况统计，存储时偏移量*/
#define LORA_STATE_OFFSET (LORA_DI_SLAVE_MAX_NUM * 8U) // 32U

/*定义Master发送缓冲区字节数*/
#define PF_TX_SIZE 64U
#define LEVENTS (sizeof(L101_Map) / sizeof(L101_HandleTypeDef))
/*累计3次超时或者错误后，改变上报的时间*/
#define SUSPEND_TIMES 3U
/*30s增加一个离线设备扫描*/
#define TOTAL_SUM 5U
/*2轮调度均离线：则认为从机掉线，主动清除状态*/
#define SCHEDULING_COUNTS 2U
    /*Enter键值*/
    // #define ENTER_CODE 0x2F
    typedef enum
    {
        Lora_Nomal_Recive = 0,
        Lora_Param_Cfg,
        Lora_Unknown_Mode,
    } Lora_Work_Mode;

    typedef enum
    {
        Lora_Di_Slave,
        Lora_Do_Slave,
        Lora_Ai_Slave,
        Lora_Ao_Slave,
        Lora_Unknown_Slave,
    } Lora_Slave_Type;

    typedef enum
    {
        L_None,
        L_Wait,
        L_OK,
        L_Error,
        L_TimeOut,
    } Lora_State;

#if(TOOL_USING_RTOS == 2)
    struct lora_list_node
    {
        rt_list_t list;
        unsigned int id;
    };
#endif

    typedef struct Lora_HandleTypeDef *pLoraHandle;
    typedef struct Lora_HandleTypeDef Lora_Handle;
    struct Lora_HandleTypeDef
    {
        /*用于解决lora模块通过shell参数时，模式区分*/
        Lora_Work_Mode Mode;
        struct
        {
            Lora_State State;        /*当前状态*/
            uint8_t OverTimes;       /*超时时间*/
            uint8_t Counter;         /*错误计数器*/
            uint8_t Schedule_counts; /*调度次数*/
        } Check;
        /*建立lora模块各节点间调度数据结构*/
        struct
        {
#if (TOOL_USING_RTOS == 1)
            List_t *Ready, *Block;
#elif(TOOL_USING_RTOS == 2)
            struct lora_list_node Ready, Block;
            rt_list_t *temp_node;
#endif
            uint8_t Event_Id;
            uint8_t Period; /*阻塞设备调度周期*/
            bool First_Flag;
        } Schedule;
        void (*Set_Lora_Factory)(void);
        bool (*Get_Lora_Status)(pLoraHandle);
        // bool (*Lora_Handle)(pLoraHandle);
        void (*Lora_Recive_Poll)(pLoraHandle);
        void (*Lora_Transmit_Poll)(pLoraHandle);
        // void (*Lora_TI_Recive)(pLoraHandle, DMA_HandleTypeDef *);
        // bool (*Lora_MakeFrame)(pLoraHandle, Lora_Map *);
        void (*Lora_Send)(pLoraHandle, uint8_t *, uint16_t);
        bool (*Lora_Check_InputCoilState)(uint8_t *, uint8_t *, uint8_t);
        void (*Lora_CallBack)(pLoraHandle, void *);
        // UART_HandleTypeDef *huart;
        UartHandle Uart;
        Gpiox_info Cs;
        /*预留外部接口*/
        void *pHandle;
    } __attribute__((aligned(4)));

    extern pLoraHandle Lora_Object;
    extern void MX_Lora_Init(void);

#define Clear_LoraBuffer(__ptr, name)                                     \
    do                                                                    \
    {                                                                     \
        (__ptr)                        ? memset((__ptr)->name.rx.pbuf, 0, \
                                                (__ptr)->name.rx.count),  \
            (__ptr)->name.rx.count = 0 : false;                           \
    } while (0)

#define Check_FirstFlag(__ptr)                                                                          \
    do                                                                                                  \
    {                                                                                                   \
        if (!(__ptr)->Schedule.First_Flag)                                                              \
        {                                                                                               \
            (__ptr)->Schedule.First_Flag =                                                              \
                (++(__ptr)->Schedule.Event_Id >= (__ptr)->MapSize) ? (__ptr)->Schedule.Event_Id = 0xFF, \
            true                                                                                        \
                : false;                                                                                \
        }                                                                                               \
    } while (0)

/*获取主机号*/
#define Get_LoraId(__obj) ((__obj)->Uart.rx.pbuf[0U])
#define Lora_ReciveHandle(__obj, __dma) ((__obj)->Lora_TI_Recive((__obj), (__dma)))

#ifdef __cplusplus
}
#endif

#endif /* INC_LHC_LORA_H_ */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

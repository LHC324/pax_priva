#include "lhc_lora.h"
#include "main.h"
#include "lhc_modbus_port.h"

#ifdef DBG_TAG
#undef DBG_TAG
#define DBG_TAG "lhc.lora"
#endif
#define DBG_LVL DBG_LOG

/*定义lora模块对象*/
pLoraHandle Lora_Object;

/*静态函数声明*/
static bool inline Get_Lora_Status(pLoraHandle pl);
static void re_lora(void);
static void Lora_Recive_Poll(pLoraHandle pl);
static void Lora_Tansmit_Poll(pLoraHandle pl);
static bool Lora_Make_Frame(pLoraHandle pl);
static void Lora_Send(pLoraHandle pl, uint8_t *pdata, uint16_t size);
static bool Lora_Check_InputCoilState(uint8_t *p_current, uint8_t *p_last, uint8_t size);
static void Lora_CallBack(pLoraHandle pl, void *pdata);

/**
 * @brief  创建Lora模块对象
 * @param  pl 需要初始化对象指针
 * @param  ps 初始化数据指针
 * @retval None
 */
static void Create_LoraObject(pLoraHandle *pl, pLoraHandle ps)
{
    (*pl) = (pLoraHandle)lhc_lora_malloc(sizeof(Lora_Handle));
#if LORA_USING_DEBUG
    uint8_t ret = 0;
#endif
    if (*pl && ps)
    {
        lhc_lora_memcpy(&(*pl)->Check, &ps->Check, sizeof(ps->Check));
        (*pl)->Lora_Recive_Poll = Lora_Recive_Poll;
        (*pl)->Lora_Transmit_Poll = Lora_Tansmit_Poll;
        (*pl)->Get_Lora_Status = Get_Lora_Status;
        (*pl)->Set_Lora_Factory = re_lora;
        (*pl)->Lora_Send = Lora_Send;
        (*pl)->Lora_Check_InputCoilState = Lora_Check_InputCoilState;
        (*pl)->Lora_CallBack = Lora_CallBack;
#if (TOOL_USING_RTOS == 1)
        (*pl)->Schedule.Ready = (lhc_lora_list_t *)lhc_lora_malloc(sizeof(lhc_lora_list_t));
        (*pl)->Schedule.Ready ? lhc_lora_list_init((*pl)->Schedule.Ready)
                              : lhc_lora_free((*pl)->Schedule.Ready);

        (*pl)->Schedule.Block = (lhc_lora_list_t *)lhc_lora_malloc(sizeof(lhc_lora_list_t));
        (*pl)->Schedule.Block ? lhc_lora_list_init((*pl)->Schedule.Block)
                              : lhc_lora_free((*pl)->Schedule.Block);
#elif (TOOL_USING_RTOS == 2)
        lhc_lora_list_init(&(*pl)->Schedule.Ready.list);
        lhc_lora_list_init(&(*pl)->Schedule.Block.list);
#endif
        (*pl)->Schedule.First_Flag = false;
        (*pl)->Schedule.Period = 0;
        (*pl)->Schedule.Event_Id = 0;
        lhc_lora_memcpy(&(*pl)->Uart, &ps->Uart, sizeof(ps->Uart));
        (*pl)->Uart.tx.pbuf = (uint8_t *)lhc_lora_malloc(ps->Uart.tx.size);
        (*pl)->Uart.tx.pbuf ? (void)lhc_lora_memset((*pl)->Uart.tx.pbuf, 0, ps->Uart.tx.size) : lhc_lora_free((*pl)->Uart.tx.pbuf);
        (*pl)->Uart.rx.pbuf = (uint8_t *)lhc_lora_malloc(ps->Uart.rx.size);
        (*pl)->Uart.rx.pbuf ? (void)lhc_lora_memset((*pl)->Uart.rx.pbuf, 0, ps->Uart.rx.size) : lhc_lora_free((*pl)->Uart.tx.pbuf);
        // (*pl)->Uart.tx.size = ps->Uart.tx.size;
        // (*pl)->Uart.tx.count = 0;
        // (*pl)->Uart.rx.size = ps->Uart.rx.size;
        // (*pl)->Uart.rx.count = 0;

        // (*pl)->Slave.bSemaphore = ps->Slave.bSemaphore;
        // // (*pl)->Check = ps->Check;
        // (*pl)->huart = ps->huart;
        (*pl)->Cs = ps->Cs;
        (*pl)->pHandle = ps->pHandle;
#if LORA_USING_DEBUG
        LORA_DEBUG("Lora_handler = 0x%p,ret = %d.\r\n", *pl, ret);
#endif
    }
}

/**
 * @brief  销毁Lora对象
 * @param  pl 需要初始化对象指针
 * @retval None
 */
void Free_LoraObject(pLoraHandle *pl)
{
    if (*pl)
    {
#if (TOOL_USING_RTOS == 1)
        lhc_lora_free((*pl)->Schedule.Ready);
        lhc_lora_free((*pl)->Schedule.Block);
#endif
        lhc_lora_free((*pl)->Uart.tx.pbuf);
        lhc_lora_free((*pl)->Uart.rx.pbuf);
        lhc_lora_free(*pl);
    }
}

/**
 * @brief	Lora模块初始化
 * @details
 * @param	None
 * @retval	None
 */
void MX_Lora_Init(void)
{
    extern UART_HandleTypeDef huart2;
    extern DMA_HandleTypeDef hdma_usart2_rx;

    Lora_Handle lora = {
        .Check = {
            .OverTimes = SUSPEND_TIMES,
        },
        .Schedule = {
            .Period = 0,
        },
        .Uart = {
            .huart = &huart2,
            .phdma = &hdma_usart2_rx,
            .tx.count = 0,
            .tx.size = LORA_TX_BUF_SIZE,
            .rx.count = 0,
            .rx.size = LORA_RX_BUF_SIZE,
        },
        .pHandle = Lte_Modbus_Object,
    };
    Create_LoraObject(&Lora_Object, &lora);
}

/**
 * @brief	获取Lora模块当前是否空闲
 * @details	无线发送数据时拉低，用于指示发送繁忙状态
 * @param	None
 * @retval	ture/fale
 */
bool inline Get_Lora_Status(pLoraHandle pl)
{
    // return ((bool)HAL_GPIO_ReadPin(LORA_STATUS_GPIO_Port, LORA_STATUS_Pin));
    return 0;
}


/**
 * @brief	从列表中查找指定列表项
 * @details
 * @param	None
 * @retval	None
 */
lhc_loara_list_item_t *Find_ListItem(lhc_lora_list_t *list, unsigned int data)
{
#if (TOOL_USING_RTOS == 1)
    lhc_loara_list_item_t *p = list->xListEnd.pxNext;

    for (; p->xItemValue != portMAX_DELAY; p = p->pxNext)
    {
        if (listGET_LIST_ITEM_VALUE(p) == data)
        {
            return p;
        }
    }
#elif (TOOL_USING_RTOS == 2)
    struct rt_list_node *pos;

    rt_list_for_each(pos, &(list->list))
    {
        if (*&((lhc_lora_list_t *)pos)->id == data) // 仅仅适用于list是结构体首个成员
            return ((lhc_lora_list_t *)pos);
    }
#endif

    return NULL;
}

/**
 * @brief	添加列表项到列表
 * @details
 * @param	None
 * @retval	None
 */
void Add_ListItem(lhc_lora_list_t *list, unsigned int data)
{
    /*先从对应列表中查找到列表项，存在：则不添加*/
    lhc_loara_list_item_t *new = Find_ListItem(list, data);
    if (new)
        return;

    new = (lhc_loara_list_item_t *)lhc_lora_malloc(sizeof(lhc_loara_list_item_t));
    if (new == NULL)
    {
        lhc_lora_free(new);
        return;
    }

#if (TOOL_USING_RTOS == 1) 
    vListInitialiseItem(new);
    listSET_LIST_ITEM_VALUE(new, data);
    /*按照data大小升序排列*/
    // vListInsert(list, new);
    vListInsertEnd(list, new);
#elif (TOOL_USING_RTOS == 2)
    new->id = data;
    rt_list_insert_before(&(list->list), &new->list);
#endif
}

/**
 * @brief	从列表中移除列表
 * @details
 * @param	None
 * @retval	列表中剩余的列表项
 */
unsigned int Remove_ListItem(lhc_lora_list_t *list, unsigned int data)
{
    unsigned int items;

#if (TOOL_USING_RTOS == 1) 
    if (!listCURRENT_LIST_LENGTH(list))
        return 0;
    /*先从对应列表中查找到列表项，在移除*/
    lhc_loara_list_item_t *node = Find_ListItem(list, data);
    if (node && (node->xItemValue != portMAX_DELAY))
    {
        items = uxListRemove(node);
        /*释放链表节点*/
        lhc_lora_free(node);
        return items;
    }
#elif (TOOL_USING_RTOS == 2)
    // struct rt_list_node *pos, *n;

    if (!rt_list_len(&(list->list)))
        return 0;
    /*先从对应列表中查找到列表项，在移除*/
    lhc_loara_list_item_t *node = Find_ListItem(list, data);

    // rt_list_for_each_safe(pos, n, &(list->list))
    if (node)
    {
        rt_list_remove(&node->list);
        /*释放链表节点*/
        lhc_lora_free(node);
        return items;
    }
#endif

    return 0;
}

/**
 * @brief	查找目标设备（在线/离线）设备
 * @details
 * @param	None
 * @retval	None
 */
lhc_loara_list_item_t *Get_OneDevice(pLoraHandle pl, lhc_lora_list_t *list)
{
    RT_ASSERT(pl && list);

#if (TOOL_USING_RTOS == 1) 
    lhc_lora_list_t *const pxConstList = (list);

    if (!listCURRENT_LIST_LENGTH(list))
        return NULL;

    /* Increment the index to the next item and return the item, ensuring */
    /* we don't return the marker used at the end of the list.  */
    (pxConstList)->pxIndex = (pxConstList)->pxIndex->pxNext;
    if ((void *)(pxConstList)->pxIndex == (void *)&((pxConstList)->xListEnd))
    {
        (pxConstList)->pxIndex = (pxConstList)->pxIndex->pxNext;
    }
    /*记录本次调度位置*/
    // listx->pIter = (pxConstList)->pxIndex;
#if LORA_USING_DEBUG
    LORA_DEBUG("list\t\titems\tdata\r\n%p\t%d\t%d\r\n",
               list, list->uxNumberOfItems, list->pxIndex->xItemValue);
#endif

    return (pxConstList)->pxIndex;
#elif (TOOL_USING_RTOS == 2)

    if (!rt_list_len(&(list->list)))
        return NULL;

    pl->Schedule.temp_node = pl->Schedule.temp_node->next;

    if (pl->Schedule.temp_node->next == &list->list)
        pl->Schedule.temp_node = pl->Schedule.temp_node->next->next;

    return ((lhc_loara_list_item_t *)pl->Schedule.temp_node);
#endif

    return NULL;
}

/**
 * @brief  Check whether the status of each switch changes
 * @param  p_current current state
 * @param  p_last Last status
 * @param  size  Detection length
 * @retval true/false
 */
bool Lora_Check_InputCoilState(uint8_t *p_current, uint8_t *p_last, uint8_t size)
{
    bool ret = false;
    if (p_current && p_last && size)
    {
        for (uint8_t i = 0; i < size; i++)
        {
            if (p_current[i] != p_last[i])
            {
                p_last[i] = p_current[i];
                ret = true;
            }
        }
    }
    return ret;
}

/**
 * @brief	Lora模块回调函数
 * @details
 * @param	None
 * @retval	None
 */
void Lora_CallBack(pLoraHandle pl, void *pdata)
{
    pModbusHandle pd = (pModbusHandle)pl->pHandle;
    bool *pflag = (bool *)pd->Slave.pHandle;
    if (pl && pdata)
    {
        static uint8_t rbits[DIGITAL_INPUT_NUMBERS];
        /*读取对应寄存器*/
        if (!pd->Mod_Operatex(pd, InputCoil, lhc_modbus_read, (pl->Schedule.Event_Id * DIGITAL_INPUT_NUMBERS),
                              rbits, sizeof(rbits)))
        {
#if LORA_USING_DEBUG
            LORA_DEBUG("Coil reading failed!\r\n");
#endif
            return;
        }
        if (pl->Lora_Check_InputCoilState((uint8_t *)pdata, rbits, DIGITAL_INPUT_NUMBERS) && pflag)
        {
            *pflag = false;
        }
    }
}

/**
 * @brief	通过从机id得到目标从机类型
 * @details
 * @param   pl lora对象句柄
 * @param	slave_id 从机id
 * @retval	Lora_Slave_Type 当前从机类型
 */
Lora_Slave_Type Lora_Get_Slave_Type(pLoraHandle pl, uint8_t slave_id)
{
    if (NULL == pl || slave_id > SLAVE_MAX_NUMBER)
        return Lora_Unknown_Slave;

    Lora_Slave_Type type =
        slave_id <= LORA_DI_SLAVE_MAX_NUM
            ? Lora_Di_Slave
        : slave_id <= (LORA_DI_SLAVE_MAX_NUM + LORA_D0_SLAVE_MAX_NUM)
            ? Lora_Do_Slave
        : slave_id <= (LORA_DI_SLAVE_MAX_NUM + LORA_D0_SLAVE_MAX_NUM + LORA_AI_SLAVE_MAX_NUM)
            ? Lora_Ai_Slave
        : slave_id <= (LORA_DI_SLAVE_MAX_NUM + LORA_D0_SLAVE_MAX_NUM + LORA_AI_SLAVE_MAX_NUM + LORA_AO_SLAVE_MAX_NUM)
            ? Lora_Ao_Slave
            : Lora_Unknown_Slave;

    return type;
}

/**
 * @brief	检测目标调度从机是否响应
 * @details
 * @param	pl lora对象句柄
 * @retval	None
 */
void Lora_Check_Resp(pLoraHandle pl)
{
    if (pl && !pl->Schedule.First_Flag)
    {
        if (++pl->Schedule.Event_Id == SLAVE_MAX_NUMBER)
            pl->Schedule.First_Flag = true;
        pl->Schedule.Event_Id %= SLAVE_MAX_NUMBER;
    }
}

/**
 * @brief	主站处理从站发来的数据
 * @details
 * @param	pl lora对象句柄
 * @retval	None
 */
void Lora_Recive_Poll(pLoraHandle pl)
{
    uint16_t crc16 = 0;
    uint8_t slave_id = Get_LoraId(pl), event_id = (slave_id - 1U);
    pModbusHandle pd = (pModbusHandle)pl->pHandle;
    uint8_t wbits[DIGITAL_INPUT_NUMBERS];
    uint16_t target_addr = 0;

    if (NULL == pl || NULL == pd || event_id >= SLAVE_MAX_NUMBER)
        return;

    /*确认收到地址与目标从机地址相同*/
    if (pl->Uart.rx.count <= 5U)
        goto __exit;

#if LORA_USING_DEBUG
    LORA_DEBUG("\r\nLora_Buf[%d]:", pl->Uart.rx.count);
    for (uint8_t i = 0; i < pl->Uart.rx.count; i++)
    {
        LORA_DEBUG("%02X ", pl->Uart.rx.pbuf[i]);
    }
    LORA_DEBUG("\r\n");
#endif
    crc16 = get_crc16(pl->Uart.rx.pbuf, pl->Uart.rx.count - 2U, 0xffff);
#if LORA_USING_DEBUG
    LORA_DEBUG("la_rxcount = %d,crc16 = %#X.\r\n", pl->Uart.rx.count,
               (uint16_t)((crc16 >> 8U) | (crc16 << 8U)));
#endif

#if LORA_USING_DEBUG
    LORA_DEBUG("\r\n@note:current response event[%d].\r\n", event_id);
#endif
    if (get_lhc_modbus_data(pl->Uart.rx.pbuf, pl->Uart.rx.count - 2U, LHC_MODBUS_WORD) !=
        ((uint16_t)((crc16 >> 8U) | (crc16 << 8U))))
        goto __exit;

#if (TOOL_USING_RTOS == 1) 
    Add_ListItem(pl->Schedule.Ready, event_id);
    /*从阻塞列表中移除*/
    Remove_ListItem(pl->Schedule.Block, event_id);
#elif (TOOL_USING_RTOS == 2)
    Add_ListItem(&pl->Schedule.Ready, event_id);
    /*从阻塞列表中移除*/
    Remove_ListItem(&pl->Schedule.Block, event_id);
#endif
    /*此处极限容纳32个从机状态:取离散输入的后32字节空间作为状态存储*/
    if (pd->pPools->InputCoils[LORA_STATE_OFFSET + event_id] != 0x01)
    {
        pd->pPools->InputCoils[LORA_STATE_OFFSET + event_id] = 0x01;
        *(bool *)pd->Slave.pHandle = false; // 考虑线程会被抢占，导致状态未更新
    }
    pl->Check.Schedule_counts = 0;

    Lora_Check_Resp(pl);
    /*对应从站正确响应*/
    pl->Check.State = L_OK;

    /*按分区写数据到目标寄存器：数字输入响应帧*/
    switch (Lora_Get_Slave_Type(pl, slave_id))
    {
    case Lora_Di_Slave:
    {
        /*数据分离*/
#if LORA_USING_DEBUG
        LORA_DEBUG("\r\nslave[%d]:\r\n", slave_id);
#endif
        for (uint8_t i = 0; i < DIGITAL_INPUT_NUMBERS; i++)
        {
            wbits[i] = ((pl->Uart.rx.pbuf[3U] >> (i % 8U)) & 0x01);
#if LORA_USING_DEBUG
            LORA_DEBUG("%d  ", wbits[i]);
#endif
        }
        /*Lora回调函数：检测输入线圈状态是否产生变化*/
        pl->Lora_CallBack(pl, wbits);
        target_addr = event_id * DIGITAL_INPUT_NUMBERS;
#if LORA_USING_DEBUG
        LORA_DEBUG("\r\n@note: addr = %d.\r\n", target_addr);
#endif
        if (!pd->Mod_Operatex(pd, InputCoil, lhc_modbus_write, target_addr,
                              wbits, DIGITAL_INPUT_NUMBERS))
        {
#if LORA_USING_DEBUG
            LORA_DEBUG("Input coil write failed!\r\n");
#endif
        }
    }
    break;
    case Lora_Do_Slave:
    {
    }
    break;
    case Lora_Ai_Slave:
    {
        target_addr = event_id * ANALOG_INPUT_NUMBERS;
        if (!pd->Mod_Operatex(pd, InputRegister, lhc_modbus_write, target_addr,
                              &pl->Uart.rx.pbuf[3U], pl->Uart.rx.pbuf[2U]))
        {
#if LORA_USING_DEBUG
            LORA_DEBUG("Holding register write failed!\r\n");
#endif
        }
        /*检测到AI从机时：主动发中断，让主机读取数据*/
        *(bool *)pd->Slave.pHandle = false;
#if (TOOL_USING_RTOS == 1) 
        extern void dis_hex_data(uint8_t * dat, uint16_t width, uint32_t len);
        dis_hex_data(pl->Uart.rx.pbuf, 16, pl->Uart.rx.count);
#elif(TOOL_USING_RTOS == 2)
    LOG_HEX(DBG_TAG, 16, pl->Uart.rx.pbuf,  pl->Uart.rx.count);
#endif
    }
    break;
    case Lora_Ao_Slave:
    {
    }
    break;
    case Lora_Unknown_Slave:
    {
#if LORA_USING_DEBUG
        LORA_DEBUG("@Error:Input register address[%d] out of range!\r\n", target_addr);
#endif
    }
    break;
    default:
        break;
    }

__exit:
    if (pl->Check.State != L_OK)
        pl->Check.State = L_Error;

    Clear_LoraBuffer(pl, Uart);
}

/**
 * @brief	位变量组帧
 * @details 主机设备号为0，信道为0
 * @note    |---目标节点地址（2B）---|---信道（1B）---|---从机地址---|---Data---|---CRC---|
 * @param	pl Lora对象句柄
 * @retval	None
 */
// #define USING_TEST
// bool Lora_Make_Frame(pLoraHandle pl, Lora_Map *pm)
bool Lora_Make_Frame(pLoraHandle pl)
{
    pModbusHandle pd = (pModbusHandle)pl->pHandle;
    uint8_t slave_id = pl->Schedule.Event_Id + 1U;
#if !defined(USING_TRANSPARENT_MODE)
#if defined(USING_REPEATER_MODE)
    uint8_t buf[] = {USING_REPEATER_1_ADDR >> 8U, USING_REPEATER_1_ADDR, USING_REPEATER_1_ADDR, slave_id,
                     0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
#else
    uint8_t buf[] = {0x00, slave_id, slave_id, slave_id, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
#endif
    uint8_t coil_data = 0;
#else
    uint8_t buf[] = {slave_id, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
#endif
#if LORA_USING_DEBUG
    LORA_DEBUG("@note: current slave id[%d].\r\n", slave_id);
#endif
    if (NULL == pd)
        return false;

    switch (Lora_Get_Slave_Type(pl, slave_id))
    {
    case Lora_Di_Slave:
    {
#if !defined(USING_TRANSPARENT_MODE)
        buf[4] = ReadInputCoil, buf[8] = 0x08;
#else
        buf[1] = ReadInputCoil, buf[5] = 0x08;
#endif
    }
    break;
    case Lora_Do_Slave:
    {
#if !defined(USING_TEST)
        /*读取对应寄存器*/
        if (!pd->Mod_Operatex(pd, Coil, lhc_modbus_read, (slave_id - DIGITAL_OUTPUT_OFFSET),
                              &coil_data, sizeof(coil_data)))
        {
#if LORA_USING_DEBUG
            LORA_DEBUG("Coil reading failed!\r\n");
#endif
            return false;
        }
#else
        static uint8_t counts = 0;
        if (++counts >= 5)
        {
            counts = 0;
            coil_data ^= 1;
        }
#endif
#if !defined(USING_TRANSPARENT_MODE)
        buf[4] = WriteCoil, buf[7] = coil_data;
#else
        buf[1] = WriteCoil, buf[3] = coil_data;
#endif
    }
    break;
    case Lora_Ai_Slave:
    {
#if !defined(USING_TRANSPARENT_MODE)
        buf[4] = ReadInputReg, buf[8] = 0x08;
#else
        buf[1] = ReadInputReg, buf[5] = 0x08;
#endif
    }
    break;
    case Lora_Ao_Slave:
    {
    }
    break;
    case Lora_Unknown_Slave:
    {
#if LORA_USING_DEBUG
        LORA_DEBUG("@Error: Address is out of range!\r\n");
#endif
    }
    break;
    default:
        break;
    }
    pl->Uart.tx.count = sizeof(buf);
#if !defined(USING_TRANSPARENT_MODE)
    uint16_t crc = get_crc16(&buf[3U], sizeof(buf) - 5U, 0xffff);
#else
    uint16_t crc = get_crc16(buf, sizeof(buf) - 2U, 0xffff);
#endif
#if defined(USING_REPEATER_MODE)
    buf[3] |= 0x80;
#endif
    /*拷贝两个字节CRC到临时缓冲区*/
    lhc_lora_memcpy(&buf[sizeof(buf) - sizeof(uint16_t)], &crc, sizeof(uint16_t));
    lhc_lora_memcpy(pl->Uart.tx.pbuf, buf, pl->Uart.tx.count);
    pl->Lora_Send(pl, pl->Uart.tx.pbuf, pl->Uart.tx.count);

    return true;
}

/**
 * @brief	对Lora模块发送数据
 * @details
 * @param	pl lora对象句柄
 * @retval	None
 */
void Lora_Send(pLoraHandle pl, uint8_t *pdata, uint16_t size)
{
    if (NULL == pl)
        return;
    HAL_UART_Transmit_DMA(pl->Uart.huart, pdata, size);
    while (__HAL_UART_GET_FLAG((UART_HandleTypeDef *)pl->Uart.huart,
                               UART_FLAG_TC) == RESET)
    {
    }
}

/**
 * @brief	主站轮询发送数据给从站
 * @details
 * @param	pl lora对象句柄
 * @retval	None
 */
void Lora_Tansmit_Poll(pLoraHandle pl)
{
    pModbusHandle pd = (pModbusHandle)pl->pHandle;
    // Lora_Map *pm = NULL;

    if ((NULL == pl) || (NULL == pd) ||
#if (TOOL_USING_RTOS == 1) 
        (NULL == pl->Schedule.Ready) ||
        (NULL == pl->Schedule.Block) ||
#endif
        (pl->Schedule.Event_Id >= SLAVE_MAX_NUMBER))
        return;

    switch (pl->Check.State)
    { /*首次发出事件或者从机响应成功*/
    case L_None:
    case L_OK:
    {
        /*确保L101模块当前处于空闲状态*/
        while (!pl->Get_Lora_Status(pl))
        {
            lhc_lora_delay(1);
        }
        /*清除超时计数器*/
        pl->Check.Counter = 0;
        pl->Check.State = L_Wait; // 如果就绪列表为空：则阻塞在最后一号从机
        if (pl->Schedule.First_Flag)
        {
            /*每调度一个周期就绪列表，就检测阻塞列表中一个从机*/
#if (TOOL_USING_RTOS == 1)
            if (++pl->Schedule.Period > listCURRENT_LIST_LENGTH(pl->Schedule.Ready))
#elif (TOOL_USING_RTOS == 2)
            if (++pl->Schedule.Period > rt_list_len(&(pl->Schedule.Ready.list)))
#endif
            {
                pl->Schedule.Period = 0;
                /*循环的从阻塞列表中移除一个列表项*/
#if (TOOL_USING_RTOS == 1)
                lhc_loara_list_item_t *p = Get_OneDevice(pl, pl->Schedule.Block);
                if (p)
                {
                    /*加入就绪列表*/
                    Add_ListItem(pl->Schedule.Ready, listGET_LIST_ITEM_VALUE(p));
                    /*从阻塞列表中移除*/
                    Remove_ListItem(pl->Schedule.Block, listGET_LIST_ITEM_VALUE(p));
                }
#elif (TOOL_USING_RTOS == 2)
                lhc_loara_list_item_t *p = Get_OneDevice(pl, &pl->Schedule.Block);
                if (p)
                {
                    /*加入就绪列表*/
                    Add_ListItem(&pl->Schedule.Ready, p->id);
                    /*从阻塞列表中移除*/
                    Remove_ListItem(&pl->Schedule.Block, p->id);
                }
#endif
#if LORA_USING_DEBUG
                LORA_DEBUG("\r\n\r\nblock\tindex\r\n%d\t%d\r\n",
                           listCURRENT_LIST_LENGTH(pl->Schedule.Block),
                           listGET_LIST_ITEM_VALUE(pl->Schedule.Block->pxIndex));
                for (p = (lhc_loara_list_item_t *)pl->Schedule.Block->xListEnd.pxNext; p && (p->xItemValue != portMAX_DELAY);
                     p = p->pxNext)
                {
                    LORA_DEBUG("%d ", listGET_LIST_ITEM_VALUE(p));
                }
                LORA_DEBUG("\r\nready\tindex\r\n%d\t%d\r\n",
                           listCURRENT_LIST_LENGTH(pl->Schedule.Ready),
                           listGET_LIST_ITEM_VALUE(pl->Schedule.Ready->pxIndex));
                for (p = (lhc_loara_list_item_t *)pl->Schedule.Ready->xListEnd.pxNext; p && (p->xItemValue != portMAX_DELAY);
                     p = p->pxNext)
                {
                    LORA_DEBUG("%d ", listGET_LIST_ITEM_VALUE(p));
                }
                LORA_DEBUG("\r\nrema\tmini\r\n%u\t%u\r\n\r\n",
                           xPortGetFreeHeapSize(), xPortGetMinimumEverFreeHeapSize());
#endif
            }

#if (TOOL_USING_RTOS == 1)
            lhc_loara_list_item_t *p = Get_OneDevice(pl, pl->Schedule.Ready);
            if (p)
                pl->Schedule.Event_Id = (uint8_t)listGET_LIST_ITEM_VALUE(p);
#elif (TOOL_USING_RTOS == 2)
            lhc_loara_list_item_t *p = Get_OneDevice(pl, &pl->Schedule.Ready);
            if (p)
                pl->Schedule.Event_Id = (uint8_t)p->id;
#endif

#if LORA_USING_DEBUG
            LORA_DEBUG("@warning: [%d] event schedule.\r\n", pl->Schedule.Event_Id);
#endif
        }
        /*禁止接收引脚*/
        if (pl->Cs.port)
            HAL_GPIO_WritePin(pl->Cs.port, pl->Cs.pin, GPIO_PIN_SET);
        /*去掉首次扫描完毕的第一个静默周期*/
        // pl->Schedule.Event_Id != LORA_NULL_ID ?
        //     Lora_Make_Frame(pl, pm),
        //     pm->Check.State = L_Wait : false;

        // if (pl->Schedule.Event_Id != LORA_NULL_ID)
        //     Lora_Make_Frame(pl, pm);

        Lora_Make_Frame(pl);

        /*使能接收引脚*/
        if (pl->Cs.port)
            HAL_GPIO_WritePin(pl->Cs.port, pl->Cs.pin, GPIO_PIN_RESET);
    }
    break;
    case L_Wait:
    {
        /*接收超时，不是接收错误导致的超时*/
        if (++(pl->Check.Counter) >= pl->Check.OverTimes)
        {
            pl->Check.Counter = 0;
            pl->Check.State = L_TimeOut;
        }
#if LORA_USING_DEBUG
        LORA_DEBUG("[%d]wait...\r\n", pl->Schedule.Event_Id);
#endif
    }
    break;
    case L_Error:
    case L_TimeOut:
    {
        uint8_t actual_site = LORA_STATE_OFFSET + pl->Schedule.Event_Id;
        /*目标从机离线*/

        /*添加到当前设备到塞列表中*/
#if (TOOL_USING_RTOS == 1)
        Add_ListItem(pl->Schedule.Block, pl->Schedule.Event_Id);
        /*从就绪列表中移除:非首次扫描状态(就绪列表非空)*/
        pl->Schedule.First_Flag
            ? Remove_ListItem(pl->Schedule.Ready, pl->Schedule.Event_Id)
            : false;
#elif (TOOL_USING_RTOS == 2)
        Add_ListItem(&pl->Schedule.Block, pl->Schedule.Event_Id);
        /*从就绪列表中移除:非首次扫描状态(就绪列表非空)*/
        pl->Schedule.First_Flag
            ? Remove_ListItem(&pl->Schedule.Ready, pl->Schedule.Event_Id)
            : false;
#endif

#if LORA_USING_DEBUG
        LORA_DEBUG("@warning: [%d] slave timeout.\r\n\r\n", pl->Schedule.Event_Id);
#endif
        /*更新从机离线状态*/
        if (pd->pPools->InputCoils[actual_site] != 0x00)
        {
            pd->pPools->InputCoils[actual_site] = 0x00;
            *(bool *)pd->Slave.pHandle = false;
        }
        /*连续三轮调度均检测不到目标从机响应：则认为直接离线*/
        if (++pl->Check.Schedule_counts > SCHEDULING_COUNTS)
        {
            pl->Check.Schedule_counts = 0;
            //            /*对于输入型从机：离线时清除输入寄存器*/
            //            if (Lora_Get_Slave_Type(pl, (pl->Schedule.Event_Id + 1U)) == Lora_Di_Slave)
            //            {
            //                lhc_lora_memset(&pd->pPools->InputCoils[pl->Schedule.Event_Id * DIGITAL_INPUT_NUMBERS],
            //                       0x00, DIGITAL_INPUT_NUMBERS);
            //            }
            switch (Lora_Get_Slave_Type(pl, (pl->Schedule.Event_Id + 1U)))
            {
            case Lora_Di_Slave:
            {
                lhc_lora_memset(&pd->pPools->InputCoils[pl->Schedule.Event_Id * DIGITAL_INPUT_NUMBERS],
                                0x00, DIGITAL_INPUT_NUMBERS);
            }
            break;
            case Lora_Ai_Slave:
            {
                lhc_lora_memset(&pd->pPools->InputRegister[(pl->Schedule.Event_Id - LORA_DI_SLAVE_MAX_NUM) *
                                                           ANALOG_INPUT_NUMBERS * 2U],
                                0x00, ANALOG_INPUT_NUMBERS * 2U);
            }
            break;
            }
            *(bool *)pd->Slave.pHandle = false;
        }

        /*在此处递增事件，而不是在L_OK中*/
        Lora_Check_Resp(pl);

        pl->Check.State = L_None;
    }
    break;
    default:
        break;
    }
}

#ifdef RT_USING_FINSH
#include <finsh.h>

/**
 * @brief	Lora模块恢复出厂设置
 * @details	拉低 3s 以上恢复出厂设置
 * @param	None
 * @retval	None
 */
void re_lora(void)
{
//     HAL_GPIO_WritePin(LORA_RELOAD_GPIO_Port, LORA_RELOAD_Pin, GPIO_PIN_RESET);
// #if defined(USING_RTTHREAD)
//     rt_thread_mdelay(3500);
// #else
//     lhc_lora_delay(3500);
// #endif
//     HAL_GPIO_WritePin(LORA_RELOAD_GPIO_Port, LORA_RELOAD_Pin, GPIO_PIN_SET);
// #if LORA_USING_DEBUG
//     LORA_DEBUG("@Loara: Factory reset was successful.\r\n");
// #endif
}
#if (TOOL_USING_RTOS == 1)
SHELL_EXPORT_CMD(SHELL_CMD_PERMISSION(0) | SHELL_CMD_TYPE(SHELL_TYPE_CMD_FUNC), re_lora,
                 re_lora, config);
#elif (TOOL_USING_RTOS == 2)
MSH_CMD_EXPORT(re_lora, lora restore factory settings.);
#endif

#endif

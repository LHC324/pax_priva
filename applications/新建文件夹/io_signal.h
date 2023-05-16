#ifndef _IO_SIGNAL_H_
#define _IO_SIGNAL_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*定义外部数字量输入路数*/
#define SIGNAL_IO_DI_MAX 8U
/*定义外部模拟量输入路数*/
#define SIGNAL_IO_AI_MAX 16U
/*定义外部数字量输出路数*/
#define SIGNAL_IO_DO_MAX 28U
/*定义外部模拟量输入路数*/
#define SIGNAL_IO_AO_MAX 2U
/*输入数字信号量在内存中初始地址*/
#define SIGNAL_IO_DI_START_ADDR 0x00
/*输出数字信号量在内存中初始地址*/
#define SIGNAL_IO_DO_START_ADDR 0x00
/*输入模拟信号量在内存中初始地址*/
#define SIGNAL_IO_AI_START_ADDR 0x00
/*输出模拟信号量在内存中初始地址*/
#define SIGNAL_IO_AO_START_ADDR 0x00
/*手动模式有效信号地址*/
#define M_MODE_ADDR 0x08
/*模拟信号采集参数配置*/
#define ADC_DMA_CHANNEL 16U
#define ADC_SAMPLING_NUM 8U
#define ADC_DMA_SIZE (ADC_DMA_CHANNEL * ADC_SAMPLING_NUM)
#define ADC_FILTER_SHIFT 3U
/*定时器采样倍率*/
#define TIMER_SAMPLING_RATIO 32.0F
    typedef enum
    {
        DAC_OUT1 = 0x00,
        DAC_OUT2,
        DAC_OUT3,
        DAC_OUT4
    } Target_Channel;

    typedef enum
    {
        Input_A = 0x00,
        Input_B,
        Input_Other
    } MCP48xx_Channel;

    typedef struct
    {
        MCP48xx_Channel Mcpxx_Id;
        Target_Channel Channel;
    } Dac_HandleTypeDef;

#define IO_SIGNAL_ADC_NUM 3U
#define IO_SIGNAL_ADC_DATA_SIZE 256U
    typedef struct
    {
        unsigned int count;
        unsigned int buf[IO_SIGNAL_ADC_DATA_SIZE];
        unsigned int finish_flag;
    } adc_t;

    extern unsigned int Adc_buffer[ADC_DMA_SIZE];
    extern adc_t adc_group[3U];
    extern unsigned int adc_buf[IO_SIGNAL_ADC_NUM][IO_SIGNAL_ADC_DATA_SIZE];

    extern void Read_Digital_Io(void);
    extern void Read_Analog_Io(void);
    extern void Write_Digital_IO(void);
    extern void Write_Analog_IO(void);
    extern unsigned char Read_LTE_State(void);

#define Read_Io_Handle() (Read_Digital_Io(), Read_Analog_Io())
#define Write_Io_Handle() (Write_Digital_IO(), Write_Analog_IO())

#ifdef __cplusplus
}
#endif

#endif /* __IO_SIGNAL_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

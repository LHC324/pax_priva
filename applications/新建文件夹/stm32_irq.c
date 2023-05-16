/*
 * Copyright (c) 2006-2021, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2022-09-14     Administrator       the first version
 */
#include <rtthread.h>

#define DBG_TAG "main"
#define DBG_LVL DBG_LOG
#include <rtdbg.h>
#include "main.h"
#include "tool.h"
#include "dwin_port.h"
#include "small_modbus_port.h"

/* External variables --------------------------------------------------------*/
extern DMA_HandleTypeDef hdma_adc1;
extern DMA_HandleTypeDef hdma_adc3;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
// extern DMA_HandleTypeDef hdma_usart3_rx;
// extern DMA_HandleTypeDef hdma_usart3_tx;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

/******************************************************************************/
/* STM32F1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f1xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles DMA1 channel1 global interrupt.
 */
void DMA1_Channel1_IRQHandler(void)
{
    /* USER CODE BEGIN DMA1_Channel1_IRQn 0 */
    /* enter interrupt */
    rt_interrupt_enter();
    // extern rt_sem_t adc_semaphore;
    // release_semaphore(adc_semaphore);
    /* USER CODE END DMA1_Channel1_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_adc1);
    /* USER CODE BEGIN DMA1_Channel1_IRQn 1 */
    /* leave interrupt */
    rt_interrupt_leave();
    /* USER CODE END DMA1_Channel1_IRQn 1 */
}

/**
 * @brief This function handles DMA1 channel2 global interrupt.
 */
// void DMA1_Channel2_IRQHandler(void)
// {
//     /* USER CODE BEGIN DMA1_Channel2_IRQn 0 */
//     /* enter interrupt */
//     rt_interrupt_enter();
//     /* USER CODE END DMA1_Channel2_IRQn 0 */
//     HAL_DMA_IRQHandler(&hdma_usart3_tx);
//     /* USER CODE BEGIN DMA1_Channel2_IRQn 1 */
//     /* leave interrupt */
//     rt_interrupt_leave();
//     /* USER CODE END DMA1_Channel2_IRQn 1 */
// }

/**
 * @brief This function handles DMA1 channel3 global interrupt.
 */
// void DMA1_Channel3_IRQHandler(void)
// {
//     /* USER CODE BEGIN DMA1_Channel3_IRQn 0 */
//     /* enter interrupt */
//     rt_interrupt_enter();
//     /* USER CODE END DMA1_Channel3_IRQn 0 */
//     HAL_DMA_IRQHandler(&hdma_usart3_rx);
//     /* USER CODE BEGIN DMA1_Channel3_IRQn 1 */
//     /* leave interrupt */
//     rt_interrupt_leave();
//     /* USER CODE END DMA1_Channel3_IRQn 1 */
// }

/**
 * @brief This function handles DMA1 channel4 global interrupt.
 */
void DMA1_Channel4_IRQHandler(void)
{
    /* USER CODE BEGIN DMA1_Channel4_IRQn 0 */
    /* enter interrupt */
    rt_interrupt_enter();
    /* USER CODE END DMA1_Channel4_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_usart1_tx);
    /* USER CODE BEGIN DMA1_Channel4_IRQn 1 */
    /* leave interrupt */
    rt_interrupt_leave();
    /* USER CODE END DMA1_Channel4_IRQn 1 */
}

/**
 * @brief This function handles DMA1 channel5 global interrupt.
 */
// void DMA1_Channel5_IRQHandler(void)
// {
//     /* USER CODE BEGIN DMA1_Channel5_IRQn 0 */
//     /* enter interrupt */
//     rt_interrupt_enter();
//     /* USER CODE END DMA1_Channel5_IRQn 0 */
//     HAL_DMA_IRQHandler(&hdma_usart1_rx);
//     /* USER CODE BEGIN DMA1_Channel5_IRQn 1 */
//     /* leave interrupt */
//     rt_interrupt_leave();
//     /* USER CODE END DMA1_Channel5_IRQn 1 */
// }

/**
 * @brief This function handles DMA1 channel6 global interrupt.
 */
void DMA1_Channel6_IRQHandler(void)
{
    /* USER CODE BEGIN DMA1_Channel6_IRQn 0 */
    /* enter interrupt */
    rt_interrupt_enter();
    /* USER CODE END DMA1_Channel6_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_usart2_rx);
    /* USER CODE BEGIN DMA1_Channel6_IRQn 1 */
    /* leave interrupt */
    rt_interrupt_leave();
    /* USER CODE END DMA1_Channel6_IRQn 1 */
}

/**
 * @brief This function handles DMA1 channel7 global interrupt.
 */
void DMA1_Channel7_IRQHandler(void)
{
    /* USER CODE BEGIN DMA1_Channel7_IRQn 0 */
    /* enter interrupt */
    rt_interrupt_enter();
    /* USER CODE END DMA1_Channel7_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_usart2_tx);
    /* USER CODE BEGIN DMA1_Channel7_IRQn 1 */
    /* leave interrupt */
    rt_interrupt_leave();
    /* USER CODE END DMA1_Channel7_IRQn 1 */
}

/**
 * @brief This function handles USART1 global interrupt.
 */
// void USART1_IRQHandler(void)
// {
//     /* USER CODE BEGIN USART1_IRQn 0 */
//     /* enter interrupt */
//     rt_interrupt_enter();
//     /* USER CODE END USART1_IRQn 0 */
//     HAL_UART_IRQHandler(&huart1);
//     /* USER CODE BEGIN USART1_IRQn 1 */
//     /* leave interrupt */
//     rt_interrupt_leave();
//     /* USER CODE END USART1_IRQn 1 */
// }

/**
 * @brief This function handles USART2 global interrupt.
 */
void USART2_IRQHandler(void)
{
    /* USER CODE BEGIN USART2_IRQn 0 */
    /* enter interrupt */
    rt_interrupt_enter();
    if (Dwin_Object)
    {
        //        Dwin_Object->Dw_Recive((pUartHandle)&Dwin_Object->Uart);
        uartx_irq_recive(Dwin_Object, Dw);
    }
    /* USER CODE END USART2_IRQn 0 */
    HAL_UART_IRQHandler(&huart2);
    /* USER CODE BEGIN USART2_IRQn 1 */
    /* leave interrupt */
    rt_interrupt_leave();
    /* USER CODE END USART2_IRQn 1 */
}

/**
 * @brief This function handles USART3 global interrupt.
 */
// void USART3_IRQHandler(void)
// {
//     /* USER CODE BEGIN USART3_IRQn 0 */
//     /* enter interrupt */
//     rt_interrupt_enter();
//     if (Modbus_Object)
//     {
//         //        Modbus_Object->Mod_Recive((pModbusHandle)&Modbus_Object->Uart);
//         uartx_irq_recive(Modbus_Object, Mod);
//     }
//     /* USER CODE END USART3_IRQn 0 */
//     HAL_UART_IRQHandler(&huart3);
//     /* USER CODE BEGIN USART3_IRQn 1 */
//     /* leave interrupt */
//     rt_interrupt_leave();
//     /* USER CODE END USART3_IRQn 1 */
// }

/**
 * @brief This function handles ADC1 and ADC2 global interrupts.
 */
void ADC1_2_IRQHandler(void)
{
    /* USER CODE BEGIN ADC1_2_IRQn 0 */
    /* enter interrupt */
    rt_interrupt_enter();
    /* USER CODE END ADC1_2_IRQn 0 */
    HAL_ADC_IRQHandler(&hadc1);
    HAL_ADC_IRQHandler(&hadc2);
    /* USER CODE BEGIN ADC1_2_IRQn 1 */
    /* leave interrupt */
    rt_interrupt_leave();
    /* USER CODE END ADC1_2_IRQn 1 */
}

/**
 * @brief This function handles DMA2 channel4 and channel5 global interrupts.
 */
void DMA2_Channel4_5_IRQHandler(void)
{
    /* USER CODE BEGIN DMA2_Channel4_5_IRQn 0 */
    /* enter interrupt */
    rt_interrupt_enter();
    /* USER CODE END DMA2_Channel4_5_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_adc3);
    /* USER CODE BEGIN DMA2_Channel4_5_IRQn 1 */
    /* leave interrupt */
    rt_interrupt_leave();
    /* USER CODE END DMA2_Channel4_5_IRQn 1 */
}

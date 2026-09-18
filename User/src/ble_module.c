/**
 * @file        ble_module.c
 * @author      KimQi
 * @date        2024-12-18
 * @copyright   Copyright (c) 2024, KimQi. All rights reserved.
 *              This file is part of an embedded project and is licensed
 *              under the terms specified in the LICENSE file.
 */

/*******************************************************
 *                  Include Files
 *******************************************************/
#include "ble_module.h"
#include "motor_hall.h"

/*******************************************************
 *                  Global Variables
 *******************************************************/
uint8_t ble_tx_buf[BLE_MAX_BUF_LEN] = {0};
uint8_t ble_rx_buf[BLE_MAX_BUF_LEN] = {0};

QueueHandle_t ble_rx_queue = NULL;
QueueHandle_t all_off_queue = NULL;
TimerHandle_t xTimer_UsartTimeout = NULL;

/*******************************************************
 *                  Function Definitions
 *******************************************************/
void BLE_USART_init(void)
{
    GPIO_InitType GPIO_InitStructure;
    USART_InitType USART_InitStructure;
    NVIC_InitType NVIC_InitStructure;

    RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOB);
    RCC_APB2_Peripheral_Clock_Enable(RCC_APB2_PERIPH_UART4);

    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 4;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Initializes(&NVIC_InitStructure);

    GPIO_InitStructure.Pin = GPIO_PIN_14;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF7_UART4;
    GPIO_Peripheral_Initialize(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = GPIO_PIN_15;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF7_UART4;
    GPIO_Peripheral_Initialize(GPIOB, &GPIO_InitStructure);

    USART_InitStructure.BaudRate = 115200;
    USART_InitStructure.WordLength = USART_WL_8B;
    USART_InitStructure.StopBits = USART_STPB_1;
    USART_InitStructure.Parity = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode = USART_MODE_RX | USART_MODE_TX;

    USART_Initializes(UART4, &USART_InitStructure);
    USART_Interrput_Enable(UART4, USART_INT_RXDNE);
    USART_Enable(UART4);
}

void BLE_UART_DMA_Init(void)
{
    DMA_InitType DMA_InitStructure;
    NVIC_InitType NVIC_InitStructure;

    RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_DMA);
    NVIC_Priority_Group_Set(NVIC_PER4_SUB0_PRIORITYGROUP);

    NVIC_InitStructure.NVIC_IRQChannel = DMA_Channel4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Initializes(&NVIC_InitStructure);

    DMA_Reset(DMA_CH2);
    DMA_InitStructure.PeriphAddr = (UART4_BASE + 0x04);
    DMA_InitStructure.MemAddr = (uint32_t)ble_tx_buf;
    DMA_InitStructure.Direction = DMA_DIR_PERIPH_DST;
    DMA_InitStructure.BufSize = BLE_MAX_BUF_LEN;
    DMA_InitStructure.PeriphInc = DMA_PERIPH_INC_MODE_DISABLE;
    DMA_InitStructure.MemoryInc = DMA_MEM_INC_MODE_ENABLE;
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_WIDTH_BYTE;
    DMA_InitStructure.MemDataSize = DMA_MEM_DATA_WIDTH_BYTE;
    DMA_InitStructure.CircularMode = DMA_CIRCULAR_MODE_DISABLE;
    DMA_InitStructure.Priority = DMA_CH_PRIORITY_HIGHEST;
    DMA_InitStructure.Mem2Mem = DMA_MEM2MEM_DISABLE;
    DMA_Initializes(DMA_CH2, &DMA_InitStructure);
    DMA_Interrupts_Enable(DMA_CH2, DMA_INT_TXC);
    DMA_Channel_Request_Remap(DMA_CH2, DMA_REMAP_UART4_TX);
}

void BLE_dma_send(uint8_t *buf, uint8_t len)
{
    DMA_Memory_Address_Config(DMA_CH2, (uint32_t)buf);
    DMA_Buffer_Size_Config(DMA_CH2, len);
    USART_DMA_Transfer_Enable(UART4, USART_DMAREQ_TX);
    DMA_Channel_Enable(DMA_CH2);
}

uint8_t check_sum(uint8_t *data ,uint32_t len)
{
    uint8_t sum = 0x00;
    uint8_t i;

    if ((data == NULL) || (len == 0)) {
        return 0;
    }

    for (i = 3; i < len; i++) {
        sum += data[i];
    }
    sum = 0x100 - (sum % 0x100);
    return sum;
}

uint8_t ble_rx_count = 0;
uint8_t u4_receive_temp = 0;
void UART4_IRQHandler(void)
{
    uint8_t rx_data;
    FlagStatus status;

    if (USART_Interrupt_Status_Get(UART4, USART_INT_RXDNE) == SET)
    {
        u4_receive_temp = USART_Data_Receive(UART4);
        if (ble_rx_count < BLE_MAX_BUF_LEN) {
            ble_rx_buf[ble_rx_count++] = u4_receive_temp;
        }
        xTimerResetFromISR(xTimer_UsartTimeout, 0);
        USART_Interrupt_Status_Clear(UART4, USART_INT_RXDNE);
    }

    if (USART_Interrupt_Status_Get(UART4, USART_INT_IDLEF) == SET)
    {
        status = USART_Interrupt_Status_Get(UART4, USART_INT_IDLEF);
        rx_data = USART_Data_Receive(UART4);
        (void)status;
        (void)rx_data;
    }
}

void UsartTimeoutCallback(TimerHandle_t xTimer)
{
    (void)xTimer;
    if (ble_rx_count > 1) {
        xQueueSend(ble_rx_queue, &ble_rx_buf, NULL);
    }
    ble_rx_count = 0;
}

uint8_t cmd_buf[BLE_MAX_BUF_LEN] = {0};
void ble_control_Task(void* parameter)
{
    (void)parameter;
    BLE_UART_DMA_Init();
    BLE_USART_init();

    while (1)
    {
        if (xQueueReceive(ble_rx_queue, &cmd_buf, portMAX_DELAY) == pdTRUE)
        {
            if ((cmd_buf[0] == 0x03) && (cmd_buf[1] == 0x12)) {
                Motor_CommandPairSpeedRPM(0, 0, SPEED_D, 1); 
            } else if ((cmd_buf[0] == 0x03) && (cmd_buf[1] == 0x11)) {
                Motor_CommandPairSpeedRPM(0, 0, SPEED_D, 2); 
            } else if ((cmd_buf[0] == 0x03) && (cmd_buf[1] == 0x22)) {
                Motor_CommandPairSpeedRPM(SPEED_D, 1, 0, 0);
            } else if ((cmd_buf[0] == 0x03) && (cmd_buf[1] == 0x21)) {
                Motor_CommandPairSpeedRPM(SPEED_D, 2, 0, 0);
            } else if ((cmd_buf[0] == 0x03) && (cmd_buf[1] == 0x32)) {
                Motor_CommandPairSpeedRPM(SPEED_D, 1, SPEED_D, 1);
            } else if ((cmd_buf[0] == 0x03) && (cmd_buf[1] == 0x31)) {
                Motor_CommandPairSpeedRPM(SPEED_D, 2, SPEED_D, 2);
            } else if ((cmd_buf[0] == 0x03) && (cmd_buf[1] == 0x30)) {
                Motor_CommandPairSpeedRPM(0, 0, 0, 0);
            }
        }
    }
}

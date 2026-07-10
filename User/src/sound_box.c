#include "sound_box.h"

#define SOUND_TX_BUF_LEN 10
#define SOUND_RX_BUF_LEN 10
uint8_t Sound_tx_buf[SOUND_TX_BUF_LEN] = {0};
uint8_t Sound_rx_buf[SOUND_RX_BUF_LEN] = {0};
uint8_t sound_rx_count = 0;
uint8_t sound_tx_id = 0;
QueueHandle_t Sound_Box_CMD_Queue = NULL;
QueueHandle_t Sound_Box_Reply_Queue = NULL;

uint8_t EQ_mode = 0x01;
uint8_t sound_high_pitch = 50;
uint8_t sound_low_pitch = 50;
uint8_t vibrate_level = 0;
uint8_t sound_state = 0x01;
uint8_t sound_EQ = 0;

void UART3_init(void)
{
    GPIO_InitType GPIO_InitStructure;
    USART_InitType USART_InitStructure;
    NVIC_InitType NVIC_InitStructure;

    RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOB);
    RCC_APB2_Peripheral_Clock_Enable(RCC_APB2_PERIPH_AFIO);
    RCC_APB2_Peripheral_Clock_Enable(RCC_APB2_PERIPH_UART3);

    NVIC_Priority_Group_Set(NVIC_PER4_SUB0_PRIORITYGROUP);

    NVIC_InitStructure.NVIC_IRQChannel = UART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Initializes(&NVIC_InitStructure);

    GPIO_InitStructure.Pin = GPIO_PIN_10;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF10_UART3;
    GPIO_Peripheral_Initialize(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = GPIO_PIN_11;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF10_UART3;
    GPIO_Peripheral_Initialize(GPIOB, &GPIO_InitStructure);

    USART_InitStructure.BaudRate = 115200;
    USART_InitStructure.WordLength = USART_WL_8B;
    USART_InitStructure.StopBits = USART_STPB_1;
    USART_InitStructure.Parity = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode = USART_MODE_RX | USART_MODE_TX;

    USART_Initializes(UART3, &USART_InitStructure);

    USART_Interrput_Enable(UART3, USART_INT_RXDNE);
    USART_Interrput_Enable(UART3, USART_INT_IDLEF);

    USART_Enable(UART3);
}


void UART3_DMA_Init(void)
{
    DMA_InitType DMA_InitStructure;

    RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_DMA);

    /* USARTy_Tx_DMA_Channel (triggered by USARTy Tx event) Config */
    DMA_Reset(DMA_CH5);
    DMA_InitStructure.PeriphAddr = (UART3_BASE + 0x04);
    DMA_InitStructure.MemAddr = (uint32_t)Sound_tx_buf;
    DMA_InitStructure.Direction = DMA_DIR_PERIPH_DST;
    DMA_InitStructure.BufSize = SOUND_TX_BUF_LEN;
    DMA_InitStructure.PeriphInc = DMA_PERIPH_INC_MODE_DISABLE;
    DMA_InitStructure.MemoryInc = DMA_MEM_INC_MODE_ENABLE;
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_WIDTH_BYTE;
    DMA_InitStructure.MemDataSize = DMA_MEM_DATA_WIDTH_BYTE;
    DMA_InitStructure.CircularMode = DMA_CIRCULAR_MODE_DISABLE;
    DMA_InitStructure.Priority = DMA_CH_PRIORITY_HIGHEST;
    DMA_InitStructure.Mem2Mem = DMA_MEM2MEM_DISABLE;
    DMA_Initializes(DMA_CH5, &DMA_InitStructure);
    DMA_Channel_Request_Remap(DMA_CH5, DMA_REMAP_UART3_TX);
}

void UART3_dma_send(uint8_t *buf, uint8_t len)
{
    DMA_Memory_Address_Config(DMA_CH5, (uint32_t)buf);
    DMA_Buffer_Size_Config(DMA_CH5, len);
    USART_DMA_Transfer_Enable(UART3, USART_DMAREQ_TX);
    DMA_Channel_Enable(DMA_CH5);
}

uint8_t sound_check_sum(uint8_t *data, uint8_t size)
{
    int sum = 0;
    while (size--)
    {
        sum += *(unsigned char *)data++;
    }
    return sum;
}

void Sound_Box_Config_Task(void *parameter)
{
    vTaskDelay(10000);
    Sound_CMD_Typedef Sound_CMD;
    Sound_CMD.msg_fun = 0x05;
    Sound_CMD.msg_data = 0;
    xQueueSend(Sound_Box_CMD_Queue, &Sound_CMD, 0);

    vTaskDelete(NULL);
}

FlagStatus wait_reply_flag = RESET;
uint8_t wait_reply_cmd = 0x00;

void Sound_Box_Task(void *parameter)
{
    uint8_t i,j;
    Sound_CMD_Typedef Sound_CMD;
    Sound_CMD_Typedef Sound_Reply;
    UART3_DMA_Init();
    UART3_init();
    while (1)
    {
        if (xQueueReceive(Sound_Box_CMD_Queue, &Sound_CMD, portMAX_DELAY) == pdTRUE)
        {
            j = 0;
            Sound_tx_buf[j++] = 0x55;
            Sound_tx_buf[j++] = 0xaa;
            if(sound_tx_id < 200) sound_tx_id++;
            else sound_tx_id = 0;
            Sound_tx_buf[j++] = sound_tx_id;
            Sound_tx_buf[j++] = Sound_CMD.msg_fun;
            Sound_tx_buf[j++] = Sound_CMD.msg_data;
            Sound_tx_buf[j] = sound_check_sum(Sound_tx_buf,j);
            UART3_dma_send(Sound_tx_buf,++j);
			wait_reply_flag = SET;
			if(Sound_CMD.msg_fun == 0x09 || Sound_CMD.msg_fun == 0x0a)
            {
                wait_reply_cmd = 0xff;
            }   
            else
            {
                wait_reply_cmd = sound_tx_id;
            }
            for ( i = 0; i < 3; i++)
            {
                /* code */
                if (xQueueReceive(Sound_Box_Reply_Queue, &Sound_Reply, 200) == pdTRUE)
                {
                    if(Sound_Reply.msg_fun == 0 && Sound_Reply.msg_data == sound_tx_id)
                    {
                        switch (Sound_CMD.msg_fun)
                        {
                        case 0x01:
                            /* code */
                            EQ_mode = Sound_CMD.msg_data;
                            break;
                        case 0x02:
                            sound_high_pitch = Sound_CMD.msg_data;
                            break;
                        case 0x03:
                            sound_low_pitch = Sound_CMD.msg_data;
                            break;
                        case 0x05:
                            vibrate_level = Sound_CMD.msg_data;
                            break;
                        default:
                            break;
                        }
                    }
					wait_reply_flag = RESET;
                    break;
                }
                else
                {
                    UART3_dma_send(Sound_tx_buf,j);
                }
            }
			wait_reply_flag = RESET;
        }
    }
}

void UART3_IRQHandler(void)
{
    INTStatus status;
    uint8_t rx_data;
    uint8_t check;
    Sound_CMD_Typedef Sound_Reply;
    if (USART_Interrupt_Status_Get(UART3, USART_INT_RXDNE) == SET)
    {
        if (sound_rx_count < SOUND_RX_BUF_LEN)
        {
            Sound_rx_buf[sound_rx_count++] = USART_Data_Receive(UART3); // 接收数据字节
        }
        USART_Interrupt_Status_Clear(UART3, USART_INT_RXDNE);
    }

    if (USART_Interrupt_Status_Get(UART3, USART_INT_IDLEF) == SET)
    {
        status = USART_Interrupt_Status_Get(UART3, USART_INT_IDLEF);
        rx_data = USART_Data_Receive(UART3);
        if(sound_rx_count > 4)
        {
            if(Sound_rx_buf[0] == 0x55 && Sound_rx_buf[1] == 0xaa)
            {
                if(Sound_rx_buf[sound_rx_count-1] == sound_check_sum(Sound_rx_buf,sound_rx_count-1))
                {
                    if(wait_reply_flag == SET && Sound_rx_buf[3] == wait_reply_cmd)
                    {
						Sound_Reply.msg_fun = 0;
						Sound_Reply.msg_data = Sound_rx_buf[3];
						xQueueSendFromISR( Sound_Box_Reply_Queue, &Sound_Reply, NULL);
                    }
                }
            }
        }
		sound_rx_count = 0;
    }
}
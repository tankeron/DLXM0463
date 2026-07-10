#include "rs485_control.h"
#include "motor_hall.h"

uint8_t rs485_rx_count = 0;
uint8_t rs485_tx_buf[RS485_TX_BUF_LEN] = {0};
uint8_t rs485_rx_buf[RS485_RX_BUF_LEN] = {0};

uint16_t controller_register[20] = {0};
QueueHandle_t rs485_rx_queue = NULL;
QueueHandle_t rs485_tx_queue = NULL;
QueueHandle_t rs485_key_queue = NULL;
QueueHandle_t key_queue = NULL;


void RS485_GPIO_Init(void)
{
    GPIO_InitType GPIO_InitStructure;

    RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOB);

    GPIO_InitStructure.Pin            = GPIO_PIN_8;    
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF7_UART3;
    GPIO_Peripheral_Initialize(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.Pin            = GPIO_PIN_9;
    GPIO_InitStructure.GPIO_Alternate = GPIO_AF7_UART3;
    GPIO_Peripheral_Initialize(GPIOB, &GPIO_InitStructure);
	
	GPIO_InitStructure.Pin            = GPIO_PIN_7;    
    GPIO_InitStructure.GPIO_Mode      = GPIO_MODE_OUT_PP;
    GPIO_Peripheral_Initialize(GPIOB, &GPIO_InitStructure);
    
	GPIO_Pins_Reset(GPIOB, GPIO_PIN_7);
}

void RS485_UART_Init(void)
{
    USART_InitType USART_InitStructure;
    NVIC_InitType NVIC_InitStructure;

    RCC_APB2_Peripheral_Clock_Enable(RCC_APB2_PERIPH_UART3);

    NVIC_Priority_Group_Set(NVIC_PER4_SUB0_PRIORITYGROUP);

    NVIC_InitStructure.NVIC_IRQChannel                   = UART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority           = 5;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Initializes(&NVIC_InitStructure);

    RS485_GPIO_Init();

    USART_InitStructure.BaudRate            = 115200;
    USART_InitStructure.WordLength          = USART_WL_8B;
    USART_InitStructure.StopBits            = USART_STPB_1;
    USART_InitStructure.Parity              = USART_PE_NO;
    USART_InitStructure.HardwareFlowControl = USART_HFCTRL_NONE;
    USART_InitStructure.Mode                = USART_MODE_RX | USART_MODE_TX;

    USART_Initializes(UART3, &USART_InitStructure);

    USART_Interrput_Enable(UART3, USART_INT_RXDNE);
    USART_Interrput_Enable(UART3, USART_INT_IDLEF);

    USART_Enable(UART3);
}

void UART3_DMA_Init(void)
{
    DMA_InitType DMA_InitStructure;
    NVIC_InitType NVIC_InitStructure;
    RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_DMA);

    NVIC_Priority_Group_Set(NVIC_PER4_SUB0_PRIORITYGROUP);

    NVIC_InitStructure.NVIC_IRQChannel                   = DMA_Channel4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority           = 5;
    NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;
    NVIC_Initializes(&NVIC_InitStructure);

    /* USARTy_Tx_DMA_Channel (triggered by USARTy Tx event) Config */
    DMA_Reset(DMA_CH4);
    DMA_InitStructure.PeriphAddr = (UART3_BASE + 0x04);
    DMA_InitStructure.MemAddr = (uint32_t)rs485_tx_buf;
    DMA_InitStructure.Direction = DMA_DIR_PERIPH_DST;
    DMA_InitStructure.BufSize = RS485_TX_BUF_LEN;
    DMA_InitStructure.PeriphInc = DMA_PERIPH_INC_MODE_DISABLE;
    DMA_InitStructure.MemoryInc = DMA_MEM_INC_MODE_ENABLE;
    DMA_InitStructure.PeriphDataSize = DMA_PERIPH_DATA_WIDTH_BYTE;
    DMA_InitStructure.MemDataSize = DMA_MEM_DATA_WIDTH_BYTE;
    DMA_InitStructure.CircularMode = DMA_CIRCULAR_MODE_DISABLE;
    DMA_InitStructure.Priority = DMA_CH_PRIORITY_HIGHEST;
    DMA_InitStructure.Mem2Mem = DMA_MEM2MEM_DISABLE;
    DMA_Initializes(DMA_CH4, &DMA_InitStructure);
    DMA_Interrupts_Enable(DMA_CH4, DMA_INT_TXC);
    DMA_Channel_Request_Remap(DMA_CH4, DMA_REMAP_UART3_TX);
}

/**
 * @brief  微秒级延时程序
 * @param  us  微秒数
 * @retval None
 */
void Delay_us(uint32_t us)
{
    uint32_t cnt = 0;
    uint32_t sysclk = 128000000;
    uint32_t delay = sysclk / 1000000 * us;

    while (cnt < delay) {
        cnt++;
    }
}

/**
 * @brief  CRC16校验算法
 * @param  data  数据地址
 * @param  len   数据长度
 * @retval 0  CRC16校验结果
 */
uint16_t modbus_crc16(uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    uint8_t i;
    while (len--) {
        crc = crc ^ *data++;
        for (i = 0; i < 8; i++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    return crc;
}

void Uart3_dma_send(uint8_t *buf, uint8_t len)
{
    GPIO_Pins_Set(GPIOB, GPIO_PIN_7);
    Delay_us(8);
    DMA_Memory_Address_Config(DMA_CH4, (uint32_t)buf);
    DMA_Buffer_Size_Config(DMA_CH4, len);
    USART_DMA_Transfer_Enable(UART3, USART_DMAREQ_TX);
    DMA_Channel_Enable(DMA_CH4);
}

void DMA_Channel4_IRQHandler(void)
{
    if (DMA_Interrupt_Status_Get(DMA, DMA_CH4_INT_TXC) == SET)
    {
        DMA_Interrupt_Status_Clear(DMA, DMA_CH4_INT_TXC);
        Delay_us(15);
        GPIO_Pins_Reset(GPIOB, GPIO_PIN_7);
    }
}

void UART3_IRQHandler(void)
{
    uint8_t rx_data;
    FlagStatus status;
    uint16_t check_value = 0;
    if (USART_Interrupt_Status_Get(UART3, USART_INT_RXDNE) == SET)
    {
        if(rs485_rx_count<RS485_RX_BUF_LEN)
        {
            rs485_rx_buf[rs485_rx_count++] = USART_Data_Receive(UART3);   //接收数据字节
        }
        USART_Interrupt_Status_Clear(UART3, USART_INT_RXDNE);
    }

    if (USART_Interrupt_Status_Get(UART3, USART_INT_IDLEF) == SET)
    {
        if(rs485_rx_count > 4)
        {
            check_value = rs485_rx_buf[rs485_rx_count-1];
            check_value <<= 8;
            check_value |= rs485_rx_buf[rs485_rx_count-2];

            if(check_value == modbus_crc16(rs485_rx_buf, rs485_rx_count-2))
            {
                xQueueSendFromISR(rs485_rx_queue, &rs485_rx_buf, NULL);
            }
        }
        rs485_rx_count = 0;
        status = USART_Interrupt_Status_Get(UART3,USART_INT_IDLEF);
        rx_data = USART_Data_Receive(UART3);
    }
}



void rs485_control_Task(void* parameter)
{
    BaseType_t xReturn = pdTRUE;
    rs485_tx_state_t rs485_tx_state;
    FlagStatus Reissue_Flag = SET;
    uint16_t check;
    uint32_t key_value = 0;
    uint8_t tx_count = 0;
    uint8_t rs485_reply_buf[RS485_RX_BUF_LEN] = {0};
    RS485_GPIO_Init();
    UART3_DMA_Init();
    RS485_UART_Init();
    while (1)
    {
        /* code */
        xReturn = xQueueReceive(rs485_tx_queue, &rs485_tx_state, portMAX_DELAY);
        if (pdTRUE == xReturn)
        {
            uint16_t save_step[MAX_MOTOR_NUM];
            tx_count = 0;
            Reissue_Flag = SET;
            switch(rs485_tx_state)
            {
                case key_scan:
                    rs485_tx_buf[tx_count++] = CONTROLLER_ADDR;
                    rs485_tx_buf[tx_count++] = 0x03;
                    rs485_tx_buf[tx_count++] = 0;
                    rs485_tx_buf[tx_count++] = 3;
                    rs485_tx_buf[tx_count++] = 0;
                    rs485_tx_buf[tx_count++] = 2;
                    check = modbus_crc16(rs485_tx_buf, tx_count);
                    rs485_tx_buf[tx_count++] = check & 0x00ff;
                    rs485_tx_buf[tx_count++] = check >> 8;
                    Uart3_dma_send(rs485_tx_buf, tx_count);
                    break;
                case write_led:
                    break;
                default:
                    break;
            }
            xReturn = xQueueReceive(rs485_rx_queue, &rs485_reply_buf, 50);
            if (pdTRUE == xReturn)
            {
                do
                {
                    switch (rs485_tx_state)
                    {
                        case key_scan:
                            if(rs485_reply_buf[2] == 4 && rs485_reply_buf[1] == 0x03)
                            {
                                key_value = 0;
                                key_value |= rs485_reply_buf[5];
                                key_value <<= 8;
                                key_value |= rs485_reply_buf[6];
                                key_value <<= 8;
                                key_value |= rs485_reply_buf[3];
                                key_value <<= 8;
                                key_value |= rs485_reply_buf[4];
                                xQueueSend(rs485_key_queue, &key_value, portMAX_DELAY);
                                Reissue_Flag = RESET;
                            }
                            break;
                        case write_led:
                            break;
                        default:
                            break;
                    }
                } while (xQueueReceive(rs485_rx_queue, &rs485_reply_buf, 0) == pdTRUE);
                if(Reissue_Flag == SET)
                {
                    Uart3_dma_send(rs485_tx_buf, tx_count);
                    xReturn = xQueueReceive(rs485_rx_queue, &rs485_reply_buf, 50);
                    if (pdTRUE == xReturn)
                    {
                        switch (rs485_tx_state)
                        {
                            case key_scan:
                                if(rs485_reply_buf[2] == 4 && rs485_reply_buf[1] == 0x03)
                                {
                                    key_value = 0;
                                    key_value |= rs485_reply_buf[5];
                                    key_value <<= 8;
                                    key_value |= rs485_reply_buf[6];
                                    key_value <<= 8;
                                    key_value |= rs485_reply_buf[3];
                                    key_value <<= 8;
                                    key_value |= rs485_reply_buf[4];
                                    xQueueSend(rs485_key_queue, &key_value, portMAX_DELAY);
                                    Reissue_Flag = RESET;
                                }
                                break;
                            case write_led:
                                break;
                            default:
                                break;
                        }
                    }
                    else
                    {
                        if(rs485_tx_state == key_scan)
                        {
                            key_value = 0;
                        }
                    }
                }
            }
            else
            {
                Uart3_dma_send(rs485_tx_buf, tx_count);
                xReturn = xQueueReceive(rs485_rx_queue, &rs485_reply_buf, 50);
                if (pdTRUE == xReturn)
                {
                    switch (rs485_tx_state)
                    {
                        case key_scan:
                            if(rs485_reply_buf[2] == 4 && rs485_reply_buf[1] == 0x03)
                            {
                                key_value = 0;
                                key_value |= rs485_reply_buf[5];
                                key_value <<= 8;
                                key_value |= rs485_reply_buf[6];
                                key_value <<= 8;
                                key_value |= rs485_reply_buf[3];
                                key_value <<= 8;
                                key_value |= rs485_reply_buf[4];
                                xQueueSend(rs485_key_queue, &key_value, portMAX_DELAY);
                                Reissue_Flag = RESET;
                            }
                            break;
                        case write_led:
                            break;
                        default:
                            break;
                    }
                }
                else
                {
                    if(rs485_tx_state == key_scan)
                    {
                        key_value = 0;
                    }
                }
            }
        }
    }
}

void rs485_key_scan_Task(void* parameter)
{
    rs485_tx_state_t key_msg = key_scan;
    key_state_msg_t key_state_msg;
    uint32_t key_data;
    uint8_t last_rs485_key[MAX_RS485_KEY_NUM] = {0};
    uint16_t key_pess_count[MAX_RS485_KEY_NUM] = {0};

    while (1)
    {
        /* code */
        xQueueSend(rs485_tx_queue, &key_msg, 0);
        if(xQueueReceive(rs485_key_queue, &key_data, 0) == pdPASS)
        {
            for(uint8_t i = 0; i < MAX_RS485_KEY_NUM; i++)
            {
                if(key_data & (0x00000001 << i))
                {
                    if(last_rs485_key[i] == 0)
                    {
                        key_state_msg.key_num = i;
                        key_state_msg.key_state = KEY_PESS;
                        xQueueSend(key_queue, &key_state_msg, portMAX_DELAY);
                    }
                    else
                    {
                        if(key_pess_count[i] == LONG_PESS_START_CNT)
                        {
                            key_pess_count[i]++;
                            key_state_msg.key_num = i;
                            key_state_msg.key_state = KEY_LONG_PESS;
                            xQueueSend(key_queue, &key_state_msg, portMAX_DELAY);
                        }
                        else key_pess_count[i]++;
                        
                    }
                    last_rs485_key[i] = 1;
                }
                else
                {
                    if(last_rs485_key[i] == 1)
                    {
                        key_state_msg.key_num = i;
                        key_state_msg.key_state = KEY_RELEASE;
                        xQueueSend(key_queue, &key_state_msg, portMAX_DELAY);
                        key_pess_count[i] = 0;
                    }
                    last_rs485_key[i] = 0;
                }
            }
        }
        vTaskDelay(30);
    }
}

void rs485_key_press_Task(void* parameter)
{
    key_state_msg_t key_state_msg;
    rs485_tx_state_t rs485_tx_msg;
    FlagStatus key_long_flag[3] = {RESET,RESET,RESET};
    motor_msg_t motor_cmd;
	uint16_t step;
    while (1)
    {
        if(xQueueReceive(key_queue, &key_state_msg, portMAX_DELAY) == pdPASS)
        {
            switch (key_state_msg.key_num)
            {
            case 0:
                /* code */
                switch (key_state_msg.key_state)  
                {
                case KEY_PESS:
                    /* code */
                    break;
                case KEY_RELEASE:
                    /* code */
                    break;
                default:
                    break;
                }
                break;
            default:
                break;
            }
        }
    }
}
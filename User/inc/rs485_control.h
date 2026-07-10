#ifndef __RS485_CONTROL_H__
#define __RS485_CONTROL_H__

#include "main.h"

#define CONTROLLER_ADDR 0x01
#define MAX_RS485_KEY_NUM   32
#define LONG_PESS_START_CNT 60

#define RS485_TX_BUF_LEN 64
#define RS485_RX_BUF_LEN 64

typedef enum
{
    key_scan = 0,
    write_step1 = 1,
    write_step2 = 2,
    read_step1 = 3,
    read_step2 = 4,
    write_led = 5
}rs485_tx_state_t;

#define KEY_PESS 0x01
#define KEY_LONG_PESS 0x02
#define KEY_RELEASE 0x03
typedef struct
{
    uint8_t key_num;
    uint8_t key_state;
}key_state_msg_t;

extern QueueHandle_t rs485_rx_queue;
extern QueueHandle_t rs485_tx_queue;
extern QueueHandle_t rs485_key_queue;
extern QueueHandle_t key_queue;

void rs485_control_Task(void* parameter);
void rs485_key_scan_Task(void* parameter);
void rs485_key_press_Task(void* parameter);
void zero_press_Task(void* parameter);

#endif /* __RS485_CONTROL_H__ */

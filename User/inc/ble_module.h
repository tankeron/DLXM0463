/**
 * @file        ble_module.h
 * @author      KimQi
 * @date        2024-12-18
 */

#ifndef BLE_MODULE_H
#define BLE_MODULE_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************
 *                  Include Files
 *******************************************************/
#include "main.h"

#define SPEED_D  3000
/*******************************************************
 *                  Macro Definitions
 *******************************************************/
#define BLE_MAX_BUF_LEN     20
#define BLE_FUN_BYTE0       0X00
#define BLE_FUN_BYTE1       0X01
#define BLE_FUN_BYTE2       0X04	//PID

/*******************************************************
 *                  Type Definitions
 *******************************************************/


/*******************************************************
 *                  Global Variables
 *******************************************************/
extern QueueHandle_t ble_rx_queue;
extern QueueHandle_t all_off_queue;
extern TimerHandle_t xTimer_UsartTimeout;
/*******************************************************
 *                  Function Prototypes
 *******************************************************/
void ble_control_Task(void* parameter);
void UsartTimeoutCallback(TimerHandle_t xTimer);
uint8_t check_sum(uint8_t *data ,uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* BLE_MODULE_H */

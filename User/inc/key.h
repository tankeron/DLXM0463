/**
 * @file        key.h
 * @author      KimQi
 * @date        2024-12-20
 */

#ifndef KEY_H
#define KEY_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************
 *                  Include Files
 *******************************************************/
#include "main.h"

/*******************************************************
 *                  Macro Definitions
 *******************************************************/
#define NUM_KEYS                8

/*******************************************************
 *                  Type Definitions
 *******************************************************/
typedef enum
{
    KEY_EVENT_NONE = 0,
    KEY_EVENT_PRESS,
    KEY_EVENT_LONG_PRESS,
    KEY_EVENT_RELEASE_SHORT,
    KEY_EVENT_RELEASE_LONG
} KeyEvent_t;

typedef struct
{
    GPIO_Module *port;
    uint16_t pin;
} Key_t;

typedef struct
{
    uint16_t    key_id;
    KeyEvent_t  event;
} KeyEventInfo_t;

/*******************************************************
 *                  Global Variables
 *******************************************************/
extern QueueHandle_t keyEventQueue;

/*******************************************************
 *                  Function Prototypes
 *******************************************************/
void KeyInit(void);
void KeyScanTask(void *pvParameters);
void KeyEventHandlerTask(void *pvParameters);

#ifdef __cplusplus
}
#endif

#endif /* KEY_H */
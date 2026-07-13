/**
 * @file        key.c
 * @author      KimQi
 * @date        2024-12-20
 * @copyright   Copyright (c) 2024, KimQi. All rights reserved.
 *              This file is part of an embedded project and is licensed
 *              under the terms specified in the LICENSE file.
 */

/*******************************************************
 *                  Include Files
 *******************************************************/
#include "key.h"
#include "motor_hall.h"
#include "ble_module.h"
/*******************************************************
 *                  Macro Definitions
 *******************************************************/
#define SCAN_PERIOD_MS              10U

/* 下面本质上是扫描周期计数，不是毫秒绝对值 */
#define PRESS_CONFIRM_CNT           5U      /* 3 * 10ms = 30ms */
#define RELEASE_CONFIRM_CNT         5U      /* 2 * 10ms = 20ms */
#define LONG_PRESS_CONFIRM_CNT      200U    /* 100 * 10ms = 1000ms */

#define KEY_PRESS_LEVEL             PIN_SET

/*******************************************************
 *                  Type Definitions
 *******************************************************/
typedef struct
{
    uint16_t sample_value;        /* 当前采样值 */
    uint16_t last_sample_value;   /* 上次采样值 */

    uint16_t active_value;        /* 当前已经生效的逻辑键值，0 表示无 */
    uint8_t  active_long_sent;    /* 当前 active_value 是否已发送长按 */

    uint8_t  change_cnt;          /* 当前采样值连续稳定计数 */
    uint8_t  active_hold_cnt;     /* 当前 active_value 持续保持计数（用于长按） */
} KeyContext_t;

/*******************************************************
 *                  Global Variables
 *******************************************************/
Key_t keys[NUM_KEYS] =
{
    {GPIOB, GPIO_PIN_9},
    {GPIOB, GPIO_PIN_8},
    {GPIOA, GPIO_PIN_12},
    {GPIOB, GPIO_PIN_3},
//    {GPIOB, GPIO_PIN_4},
//    {GPIOD, GPIO_PIN_12},
//	{GPIOD, GPIO_PIN_13},
//    {GPIOB, GPIO_PIN_5}
};

QueueHandle_t keyEventQueue = NULL;

/*******************************************************
 *                  Static Variables
 *******************************************************/
static KeyContext_t s_key_ctx;

/*******************************************************
 *                  Static Function Prototypes
 *******************************************************/
static void Button_GPIO_Initialize(GPIO_Module* GPIOx, uint16_t pin);
static uint16_t Key_ReadRawValue(void);
static void Key_ResetContext(void);
static BaseType_t Key_SendEvent(uint16_t key_id, KeyEvent_t event);
static void Key_ActivateNewValue(uint16_t new_value);
static void Key_ReleaseActiveValue(void);

/*******************************************************
 *                  Function Definitions
 *******************************************************/
static void Button_GPIO_Initialize(GPIO_Module* GPIOx, uint16_t pin)
{
    GPIO_InitType GPIO_InitStructure;

    if (GPIOx == GPIOA)
    {
        RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOA);
    }
    else if (GPIOx == GPIOB)
    {
        RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOB);
    }
    else if (GPIOx == GPIOC)
    {
        RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOC);
    }
    else
    {
        RCC_AHB_Peripheral_Clock_Enable(RCC_AHB_PERIPH_GPIOD);
    }

    GPIO_Structure_Initialize(&GPIO_InitStructure);
    GPIO_InitStructure.Pin       = pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.GPIO_Pull = GPIO_PULL_UP;
    GPIO_Peripheral_Initialize(GPIOx, &GPIO_InitStructure);
}

static uint16_t Key_ReadRawValue(void)
{
    uint8_t i = 0U;
    uint16_t key_value = 0U;

    for (i = 0U; i < NUM_KEYS; i++)
    {
        if (GPIO_Input_Pin_Data_Get(keys[i].port, keys[i].pin) == KEY_PRESS_LEVEL)
        {
            key_value |= (uint16_t)(1U << i);
        }
    }

    return key_value;
}

static void Key_ResetContext(void)
{
    s_key_ctx.sample_value      = 0U;
    s_key_ctx.last_sample_value = 0U;
    s_key_ctx.active_value      = 0U;
    s_key_ctx.active_long_sent  = 0U;
    s_key_ctx.change_cnt        = 0U;
    s_key_ctx.active_hold_cnt   = 0U;
}

static BaseType_t Key_SendEvent(uint16_t key_id, KeyEvent_t event)
{
    KeyEventInfo_t key_event;

    if (keyEventQueue == NULL)
    {
        return pdFAIL;
    }

    key_event.key_id = key_id;
    key_event.event  = event;

    return xQueueSend(keyEventQueue, &key_event, 0);
}

static void Key_ReleaseActiveValue(void)
{
    if (s_key_ctx.active_value == 0U)
    {
        return;
    }

    if (s_key_ctx.active_long_sent != 0U)
    {
        (void)Key_SendEvent(s_key_ctx.active_value, KEY_EVENT_RELEASE_LONG);
    }
    else
    {
        (void)Key_SendEvent(s_key_ctx.active_value, KEY_EVENT_RELEASE_SHORT);
    }

    s_key_ctx.active_value     = 0U;
    s_key_ctx.active_long_sent = 0U;
    s_key_ctx.active_hold_cnt  = 0U;
}

static void Key_ActivateNewValue(uint16_t new_value)
{
    if (new_value == 0U)
    {
        return;
    }

    (void)Key_SendEvent(new_value, KEY_EVENT_PRESS);

    s_key_ctx.active_value     = new_value;
    s_key_ctx.active_long_sent = 0U;
    s_key_ctx.active_hold_cnt  = 0U;
}

void KeyInit(void)
{
    uint8_t i = 0U;

    for (i = 0U; i < NUM_KEYS; i++)
    {
        Button_GPIO_Initialize(keys[i].port, keys[i].pin);
    }

    if (keyEventQueue == NULL)
    {
        keyEventQueue = xQueueCreate(10U, sizeof(KeyEventInfo_t));
    }

    Key_ResetContext();
}

void KeyScanTask(void *pvParameters)
{
    uint16_t raw_value = 0U;

    (void)pvParameters;

    KeyInit();

    while (1)
    {
        raw_value = Key_ReadRawValue();
        s_key_ctx.sample_value = raw_value;

        /***************************************************
         * 1. 采样是否变化
         ***************************************************/
        if (s_key_ctx.sample_value == s_key_ctx.last_sample_value)
        {
            if (s_key_ctx.change_cnt < 0xFFU)
            {
                s_key_ctx.change_cnt++;
            }
        }
        else
        {
            s_key_ctx.change_cnt = 0U;
        }

        /***************************************************
         * 2. 当前无激活事件时，等待新键值稳定成立
         ***************************************************/
        if (s_key_ctx.active_value == 0U)
        {
            if ((s_key_ctx.sample_value != 0U) &&
                (s_key_ctx.sample_value == s_key_ctx.last_sample_value) &&
                (s_key_ctx.change_cnt >= PRESS_CONFIRM_CNT))
            {
                Key_ActivateNewValue(s_key_ctx.sample_value);

                /* 已经把当前值设为 active，避免本轮后续重复进入 */
                s_key_ctx.change_cnt = 0U;
            }
        }
        /***************************************************
         * 3. 当前已有激活事件
         ***************************************************/
        else
        {
            /************************************************
             * 3.1 当前采样值 == 激活值，做长按计时
             ************************************************/
            if (s_key_ctx.sample_value == s_key_ctx.active_value)
            {
                s_key_ctx.change_cnt = 0U;

                if (s_key_ctx.active_long_sent == 0U)
                {
                    if (s_key_ctx.active_hold_cnt < 0xFFU)
                    {
                        s_key_ctx.active_hold_cnt++;
                    }

                    if (s_key_ctx.active_hold_cnt >= LONG_PRESS_CONFIRM_CNT)
                    {
                        (void)Key_SendEvent(s_key_ctx.active_value, KEY_EVENT_LONG_PRESS);
                        s_key_ctx.active_long_sent = 1U;
                    }
                }
            }
            /************************************************
             * 3.2 当前采样值 != 激活值，说明逻辑键值发生变化
             *     先等待“新采样值”稳定，再执行：
             *     先释放旧 active，再激活新值
             ************************************************/
            else
            {
                if ((s_key_ctx.sample_value == s_key_ctx.last_sample_value) &&
                    (s_key_ctx.change_cnt >= RELEASE_CONFIRM_CNT))
                {
                    /* 先释放上一个逻辑键值 */
                    Key_ReleaseActiveValue();

                    /* 再根据新值决定是否激活新逻辑键值 */
                    if (s_key_ctx.sample_value != 0U)
                    {
                        Key_ActivateNewValue(s_key_ctx.sample_value);
                    }

                    s_key_ctx.change_cnt = 0U;
                }
            }
        }

        s_key_ctx.last_sample_value = s_key_ctx.sample_value;

        vTaskDelay(pdMS_TO_TICKS(SCAN_PERIOD_MS));
    }
}
KeyEventInfo_t key_event;
void KeyEventHandlerTask(void *pvParameters)
{
    
    (void)pvParameters;

    while (1)
    {
        if (xQueueReceive(keyEventQueue, &key_event, portMAX_DELAY) == pdPASS)
        {
            switch (key_event.event)
            {
            case KEY_EVENT_PRESS:
                switch (key_event.key_id)
                {
                case 0x0001:
					Motor_CommandPairSpeedRPM(SPEED_D, 1, 0, 0);
					break;
                case 0x0004:
					Motor_CommandPairSpeedRPM(0, 0, SPEED_D, 1);
                    break;
				case 0x0002:
					Motor_CommandPairSpeedRPM(SPEED_D, 2, 0, 0);
					break;
                case 0x0008:
					Motor_CommandPairSpeedRPM(0, 0, SPEED_D, 2);
                    break;
				case (0x0010|0x0001):
					Motor_CommandPairSpeedRPM(SPEED_D, 2, SPEED_D, 2);
					break;
				case (0x0020|0x0002):
					Motor_CommandPairSpeedRPM(SPEED_D, 1, SPEED_D, 1);
					break;
				case (0x0040|0x0004):
					Motor_CommandPairSpeedRPM(SPEED_D, 2, SPEED_D, 2);
					break;
				case (0x0080|0x0008):
					Motor_CommandPairSpeedRPM(SPEED_D, 1, SPEED_D, 1);
					break;
                default:
                    break;
                }
                break;

            case KEY_EVENT_LONG_PRESS:
				/* if(key_event.key_id == 0x0003)
				{
					Motor_Learn_Start();
					Wifi_SendEnterConfig(3,NULL,NULL);
				} */
                break;

            case KEY_EVENT_RELEASE_SHORT:
            case KEY_EVENT_RELEASE_LONG:
				Motor_CommandPairSpeedRPM(0, 0, 0, 0);
                /* switch (key_event.key_id)
                {
                case 0x0001:
                case 0x0002:
                    Motor_Stop_Run(0);
                    break;
                default:
                    break;
                }
                break; */

            default:
                break;
            }
        }
    }
}
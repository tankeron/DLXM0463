/*******************************************************************************
* @项目名称 ：贵妃椅
* @软件版本 ：
* @硬件版本 ：
* @功能说明 ：1.双电机位置同步。
			  2.双018手控器加遥控器，单边位可单控自己位置，也可同时控制
     
* @版本更改说明 ：
* @备注     ：		
*******************************************************************************/
#include "main.h"
#include "motor_hall.h"
#include "rs485_control.h"
#include "ble_module.h"
#include "key.h"

ErrorStatus HSIStartUpStatus;

void system_clock_init(void)
{
    /* Set up the system clock */
    RCC_Reset();
    /* Enable HSI */
    RCC_HSI_Enable();
    
    /* Wait till HSI is ready */
    HSIStartUpStatus = RCC_HSI_Stable_Wait();

    if (HSIStartUpStatus != SUCCESS)
    {
        /* If HSI fails to start-up, the application will have wrong clock
            configuration. User can add here some code to deal with this
            error */

        /* Go to infinite loop */
        while (1);
    }

    FLASH_Latency_Set(FLASH_LATENCY_3);
    /* HCLK = SYSCLK */
    RCC_Hclk_Config(RCC_SYSCLK_DIV1);

    /* PCLK2 = HCLK */
    RCC_Pclk2_Config(RCC_SYSCLK_DIV4);

    /* PCLK1 = HCLK */
    RCC_Pclk1_Config(RCC_SYSCLK_DIV4);

    RCC_PLL_Config(RCC_PLL_SRC_HSI_DIV2,RCC_PLL_MUL_32);

    /* Enable PLL */
    RCC_PLL_Enable();
    /* Wait till PLL is ready */
    while ((RCC->CTRL & RCC_CTRL_PLLRDF) == 0)
    {
    }
    /* Select PLL as system clock source */
    RCC_Sysclk_Config(RCC_SYSCLK_SRC_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while (RCC_Sysclk_Source_Get() != RCC_CFG_SCLKSTS_PLL);
}

int main(void)
{
	BaseType_t xReturn = pdPASS; 

	system_clock_init();

	NVIC_Priority_Group_Set(NVIC_PER4_SUB0_PRIORITYGROUP);
	//SEGGER_SYSVIEW_Conf();
	taskENTER_CRITICAL();	

//	motor_queue = xQueueCreate((UBaseType_t)30, (UBaseType_t)sizeof(motor_msg_t));
	
	ble_rx_queue = xQueueCreate((UBaseType_t)10, (UBaseType_t)BLE_MAX_BUF_LEN);
	
	all_off_queue = xQueueCreate((UBaseType_t)5, (UBaseType_t)sizeof(uint8_t));
	keyEventQueue = xQueueCreate(10, sizeof(KeyEventInfo_t));
	
	xTimer_UsartTimeout = xTimerCreate(
       					"SingleShotTimer",         
       					pdMS_TO_TICKS(10),       
       					pdFALSE,                   
       					0,                          
       					UsartTimeoutCallback);
	xTimerStop(xTimer_UsartTimeout, 0);
	MotorRTOS_Init();
	
//	xTimer_RemoteTimeout = xTimerCreate(
//       					"SingleShotTimer",         
//       					pdMS_TO_TICKS(10),       
//       					pdFALSE,                   
//       					0,                          
//       					remoteTimeoutCallback);
//	xTimerStop(xTimer_RemoteTimeout, 1);
	
//	xReturn = xTaskCreate(MotoHall_Task, "MotoHall", 256, NULL, 2, NULL);
//	if (pdPASS != xReturn) return -1;

	xReturn = xTaskCreate((TaskFunction_t)ble_control_Task,
						  (const char *)"ble_control_Task",
						  (uint16_t)128,						
						  (void *)NULL,						
						  (UBaseType_t)1,						
						  NULL);
	if (pdPASS != xReturn) return -1;
						  
	xReturn = xTaskCreate((TaskFunction_t)KeyScanTask,
						  (const char *)"KeyScanTask",
						  (uint16_t)64,						
						  (void *)NULL,						
						  (UBaseType_t)1,						
						  NULL);
	if (pdPASS != xReturn) return -1;
						  
	xReturn = xTaskCreate((TaskFunction_t)KeyEventHandlerTask,
						  (const char *)"KeyEventHandlerTask",
						  (uint16_t)64,						
						  (void *)NULL,						
						  (UBaseType_t)1,						
						  NULL);
	if (pdPASS != xReturn) return -1;	

//	Remote_USART_init();

	taskEXIT_CRITICAL();
	
	if (pdPASS == xReturn)
		vTaskStartScheduler(); 
	else
		return -1;

	while (1)
		;
}




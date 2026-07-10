#ifndef _SOUND_BOX_H_
#define _SOUND_BOX_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* Constants */

/* Macros */

/* Types */
typedef struct
{
    uint8_t msg_fun;
    uint8_t msg_data;
}Sound_CMD_Typedef;

extern uint8_t EQ_mode;
extern uint8_t sound_high_pitch;
extern uint8_t sound_low_pitch;
extern uint8_t vibrate_level;
extern QueueHandle_t Sound_Box_CMD_Queue;
extern QueueHandle_t Sound_Box_Reply_Queue;

/* Functions */
void Sound_Box_Task(void *parameter);
void Sound_Box_Config_Task(void *parameter);
#ifdef __cplusplus
}
#endif

#endif /* _SOUND_BOX_H_ */

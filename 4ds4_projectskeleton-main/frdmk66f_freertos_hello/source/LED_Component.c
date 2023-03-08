#include "LED_Component.h"

QueueHandle_t led_queue;
BaseType_t status;

void setupLEDComponent()
{
	setupLEDPins();

    /*************** LED Task ***************/
	led_queue = xQueueCreate(1, sizeof(ftm_chnl_t));
	status = xTaskCreate(ledTask, "producer", 200, (void*)led_queue, 2, NULL);
	if (status != pdPASS)
		PRINTF("Task creation failed!.\r\n");
}

void setupLEDPins()
{
	setupLEDs(FTM_RED_CHANNEL);
	setupLEDs(FTM_GREEN_CHANNEL);
    setupLEDs(FTM_BLUE_CHANNEL);

	//FTM_UpdatePwmDutycycle(FTM3, received_channel, kFTM_EdgeAlignedPwm, 255);
    FTM_SetSoftwareTrigger(FTM3, true);
}

void setupLEDs(ftm_chnl_t channel)
{
	ftm_config_t ftmInfo;
	ftm_chnl_pwm_signal_param_t ftmParam;

	ftmParam.chnlNumber = channel;
	ftmParam.level = kFTM_HighTrue;
	ftmParam.dutyCyclePercent = 0;
	ftmParam.firstEdgeDelayPercent = 0U;
	ftmParam.enableComplementary = false;
	ftmParam.enableDeadtime = false;

	FTM_GetDefaultConfig(&ftmInfo);

	FTM_Init(FTM3, &ftmInfo);
	FTM_SetupPwm(FTM3, &ftmParam, 1U, kFTM_EdgeAlignedPwm, 5000U, CLOCK_GetFreq(kCLOCK_BusClk));
	FTM_StartTimer(FTM3, kFTM_SystemClock);
}

void ledTask(void* pvParameters)
{
	BaseType_t status;
	char speed;
	int r, g, b;

	status = xQueueReceive(led_queue, (void *) &speed, portMAX_DELAY);

	if (status != pdPASS)
			PRINTF("Queue Receive failed!.\r\n");
	switch(speed){
	case 'f':
		r = 100;
		g = 0;
		break;
	case 'm':
		r = 100;
		g = 75;
		break;
	case 's':
		r = 0;
		g = 100;
		break;
	}

	FTM_UpdatePwmDutycycle(FTM3, FTM_RED_CHANNEL, kFTM_EdgeAlignedPwm, r);
	FTM_UpdatePwmDutycycle(FTM3, FTM_GREEN_CHANNEL, kFTM_EdgeAlignedPwm, g);
	FTM_UpdatePwmDutycycle(FTM3, FTM_BLUE_CHANNEL, kFTM_EdgeAlignedPwm, 0);
    FTM_SetSoftwareTrigger(FTM3, true);

//	vTaskDelete(NULL);
}

//THIS TO GO IN MAIN FILE


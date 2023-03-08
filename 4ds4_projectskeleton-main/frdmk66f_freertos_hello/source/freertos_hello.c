#include "FreeRTOS.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "clock_config.h"
#include "board.h"

#include "Motor_Control_Component.h"
#include "RC_Receiver_Component.h"
#include "Terminal_Component.h"
#include "LED_Component.h"
#include "Accelerometer_Component.h"

void led_producer_task(void* pvParameters) {
	char speed = (char)pvParameters;
	BaseType_t status;

	//led_value would be a global variable
	status = xQueueSendToBack(led_queue, (void*) &speed, portMAX_DELAY);
	if (status != pdPASS)
		PRINTF("Queue Send failed!.\r\n");

	vTaskDelete(NULL);
}

int main(void)
{
    /* Init board hardware. */
	BOARD_InitBootPins();
    BOARD_InitBootClocks();
    //
//    setupMotorComponent();
//    setupRCReceiverComponent();
//    setupTerminalComponent();
      setupLEDComponent();
//    setupAccelerometerComponent();

    char speed = 'm';
    xTaskCreate(led_producer_task, "producer", 200, (void*)speed, 2, NULL);


    vTaskStartScheduler();

    while(1)
    {}
}


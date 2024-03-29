#include "RC_Receiver_Component.h"
#include <stdio.h>

SemaphoreHandle_t rc_hold_semaphore;
TaskHandle_t rc_task_handle;


void setupRCReceiverComponent()
{
	setupRCPins();

	setupUART_RC();

    /*************** RC Task ***************/
	//Create RC Semaphore
	rc_hold_semaphore = xSemaphoreCreateBinary();
	//Create RC Task
    xTaskCreate(rcTask, "producer", 200, NULL, 2, NULL);

}

void setupRCPins()
{
	//Configure RC pins

	/* Set Port C to pin 3 Alt 3 for UART1_RX */
	PORT_SetPinMux(PORTC, 3U, kPORT_MuxAlt3);
}

void setupUART_RC()
{
	//setup UART for RC receiver
	uart_config_t config;
	UART_GetDefaultConfig(&config);
	config.baudRate_Bps = 115200;
	config.enableTx = false;
	config.enableRx = true;
	UART_Init(UART1, &config, CLOCK_GetFreq(kCLOCK_CoreSysClk));
}

void rcTask(void* pvParameters)
{
//	/*
//	This task is responsible for decoding the data coming from the
//	radio receiver, and it sends the corresponding values to Motor,
//	Angle, and LED queues. The task can be paused if it receives a
//	signal from Hold semaphore.
//	*/
	RC_Values rc_values;
	uint8_t* ptr = (uint8_t*)&rc_values;
	int servo_angle;
	int motor_speed;
	char speed;
	motor_value motor_val;

	BaseType_t status_angle;
	BaseType_t status_motor;
	BaseType_t status_LED;
	BaseType_t status_sem;

	xSemaphoreGive(rc_hold_semaphore);
	while (1)
	{

		UART_ReadBlocking(UART1, ptr, 1);
		if(*ptr != 0x20)
			continue;
		UART_ReadBlocking(UART1, &ptr[1], sizeof(rc_values) - 1);
		status_sem = xSemaphoreTake(rc_hold_semaphore, portMAX_DELAY);
		if (status_sem != pdPASS)
		{
			PRINTF("Failed to acquire producer1_semaphore\r\n");
			while (1);
		}
//		if(rc_values.header == 0x4020)
//		{
//			printf("Channel 1 = %d\t", rc_values.ch1);
//			PRINTF("Channel 2 = %d\t", rc_values.ch2);
//			PRINTF("Channel 3 = %d\t", rc_values.ch3);
//			PRINTF("Channel 4 = %d\t", rc_values.ch4);
//			PRINTF("Channel 5 = %d\t", rc_values.ch5);
//			PRINTF("Channel 6 = %d\t", rc_values.ch6);
//			PRINTF("Channel 7 = %d\t", rc_values.ch7);
//			PRINTF("Channel 8 = %d\r\n", rc_values.ch8);
//		}
		/*
		Ch 1 - L/R of right analog (for servo angle) left is 1000, right is 2000
		Ch 2 - up/down of right analog (NOT NEEDED)
		ch 3 - up/down of left analog (motor speed) down is 1000, up is 2000
		ch 4 - L/R of left analog (NOT NEEDED)
		ch 5 - SWA, up is 1000, down is 2000 (fwd and reverse)
		ch 6 - SWB, up is 1000, middle is 1500, down is 2000 (for speed levels)
		ch 7 - (NOT NEEDED)
		ch 8 - (NOT NEEDED)
		*/

		if (rc_values.ch5 == 1000) { //1000 for forward
			servo_angle = ((rc_values.ch1 - 1500) / 5);
			if (rc_values.ch6 == 1000) { // 1000 for full speed
				motor_speed = ((rc_values.ch3 - 1000) / 10);
				speed = 'f'; //fast
			}
			else if (rc_values.ch6 == 1500) { // middle speed setting
				motor_speed = ((rc_values.ch3 - 1000) / 15);
				speed = 'm'; //medium
			}
			else if (rc_values.ch6 == 2000) { // slowest speed setting
				motor_speed = ((rc_values.ch3 - 1000) / 30);
				speed = 's'; //slow
			}

		}
		else if (rc_values.ch5 == 2000) { // 2000 for reverse
			servo_angle = ((rc_values.ch1 - 1500) / 5);
			if (rc_values.ch6 == 1000) { // 1000 for full speed
				motor_speed = -((rc_values.ch3 - 1000) / 10);
				speed = 'f'; // fast
			}
			else if (rc_values.ch6 == 1500) { // middle speed setting
				motor_speed = -((rc_values.ch3 - 1000) / 15);
				speed = 'm'; //medium
			}
			else if (rc_values.ch6 == 2000) { // slowest speed setting
				motor_speed = -((rc_values.ch3 - 1000) / 30);
				speed = 's'; // slow
			}
		}

		motor_val.source = 'r';
		motor_val.speed = motor_speed;

		status_motor = xQueueSendToBack(motor_queue, (void*)&motor_val, portMAX_DELAY);
		if (status_motor != pdPASS)
		{
			PRINTF("Motor Speed Queue Send failed!.\r\n");
			while (1);
		}

		status_angle = xQueueSendToBack(angle_queue, (void*)&servo_angle, portMAX_DELAY);
		if (status_angle != pdPASS)
		{
			PRINTF("Servo Angle Queue Send failed!.\r\n");
			while (1);
		}

		status_LED = xQueueSendToBack(led_queue, (void*)&speed, portMAX_DELAY);
		if (status_LED != pdPASS)
		{
			PRINTF("LED Queue Send failed!.\r\n");
			while (1);
		}
		xSemaphoreGive(rc_hold_semaphore);
	}
}



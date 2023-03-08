#include "RC_Receiver_Component.h"

SemaphoreHandle_t rc_hold_semaphore;
TaskHandle_t rc_task_handle;

void setupRCReceiverComponent()
{
	setupRCPins();

	setupUART_RC();

    /*************** RC Task ***************/
	//Create RC Semaphore
	// 
	//Create RC Task

}

void setupRCPins()
{
	//Configure RC pins
	/* Port C Clock Gate Control: Clock enabled */
	CLOCK_EnableClock(kCLOCK_PortC);

	/* Set Port C to pin 3 Alt 3 for UART1_RX */
	PORT_SetPinMux(PORTC, 3U, kPORT_MuxAlt3);
}

void setupUART_RC()
{
	//setup UART for RC receiver
	uart_config_t config;
	RC_Values rc_values;
	uint8_t* ptr = (uint8_t*)&rc_values;
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	UART_GetDefaultConfig(&config);
	config.baudRate_Bps = 115200;
	config.enableTx = false;
	config.enableRx = true;
	UART_Init(UART1, &config, CLOCK_GetFreq(kCLOCK_CoreSysClk));
}

void rcTask(void* pvParameters)
{
	/*
	This task is responsible for decoding the data coming from the
	radio receiver, and it sends the corresponding values to Motor,
	Angle, and LED queues. The task can be paused if it receives a
	signal from Hold semaphore.
	*/
	char speed;
	//RC task implementation
	UART_ReadBlocking(UART1, ptr, 1);
	if (*ptr != 0x20)
		continue;
	UART_ReadBlocking(UART1, &ptr[1], sizeof(rc_values) - 1);
	if (rc_values.header == 0x4020)
	{
		printf("Channel 1 = %d\t", rc_values.ch1);
		printf("Channel 2 = %d\t", rc_values.ch2);
		printf("Channel 3 = %d\t", rc_values.ch3);
		printf("Channel 4 = %d\t", rc_values.ch4);
		printf("Channel 5 = %d\t", rc_values.ch5);
		printf("Channel 6 = %d\t", rc_values.ch6);
		printf("Channel 7 = %d\t", rc_values.ch7);
		printf("Channel 8 = %d\r\n", rc_values.ch8);
	}

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

	// Scale the values sent to the motor based on what mode we are in

	if (rc_values.ch5 == 1000) { //1000 for forward
		servo_angle = ((rc_values.ch1 - 1000) / 1000) * 100;
		if (rc_values.ch6 == 1000) { // 1000 for full speed
			motor_speed = ((rc_values.ch3 - 1000) / 1000) * 100;
			speed = "f"; //fast
		}
		else if (rc_values.ch6 == 1500) { // middle speed setting
			motor_speed = ((rc_values.ch3 - 1000) / 1000) * 66;
			speed = "m"; //medium
		}
		else if (rc_values.ch6 == 2000) { // slowest speed setting
			motor_speed = ((rc_values.ch3 - 1000) / 1000) * 33;
			speed = "s"; //slow
		}

	}
	else if (rc_values.ch5 == 2000) { // 2000 for reverse
		servo_angle = ((rc_values.ch1 - 1000) / 1000) * 100;
		if (rc_values.ch6 == 1000) { // 1000 for full speed
			motor_speed = -((rc_values.ch3 - 1000) / 1000) * 100;
			speed = "f"; // fast
		}
		else if (rc_values.ch6 == 1500) { // middle speed setting
			motor_speed = -((rc_values.ch3 - 1000) / 1000) * 66;
			speed = "m"; //medium
		}
		else if (rc_values.ch6 == 2000) { // slowest speed setting
			motor_speed = -((rc_values.ch3 - 1000) / 1000) * 33;
			speed = "s"; // slow
		}
	}

	status_motor = xQueueSendToBack(motor_queue, (void*)&motor_speed, portMAX_DELAY);
	if (status_motor != pdPASS)
	{
		PRINTF("Motor Speed Queue Send failed!.\r\n");
		while (1);
	}

	status_angle = xQueueSendToBack(servo_queue, (void*)&servo_angle, portMAX_DELAY);
	if (status_angle != pdPASS)
	{
		PRINTF("Servo Angle Queue Send failed!.\r\n");
		while (1);
	}

	status_LED = xQueueSendToBack(led_queue, (void*)&speed, portMAX_DELAY);
	if (status_angle != pdPASS)
	{
		PRINTF("LED Queue Send failed!.\r\n");
		while (1);
	}

}



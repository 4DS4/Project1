#include "Terminal_Component.h"

EventGroupHandle_t event_group;
QueueHandle_t uart_queue;
BaseType_t status_term;


volatile char ch;
volatile int new_char = 0;

void setupTerminalComponent()
{
	BaseType_t status;

	setupTerminalPins();

	setupUART();

    /*************** UART Task ***************/
	uart_queue = xQueueCreate(10, sizeof(char*));
	if (uart_queue == NULL)
	{
		PRINTF("Queue creation failed!.\r\n");
		while (1);
	}
    status = xTaskCreate(uartTask, "UART task", 200, NULL, 3, NULL);
    if (status != pdPASS)
    {
        PRINTF("Task creation failed!.\r\n");
        while (1);
    }

    /*************** Terminal Control Task ***************/


    //Create Event groups
	event_group = xEventGroupCreate();

	//Create Terminal Control Task
	status = xTaskCreate(terminalControlTask, "Terminal task", 200, NULL, 3, NULL);
    if (status != pdPASS)
    {
        PRINTF("Event creation failed!.\r\n");
        while (1);
    }
}

void setupTerminalPins()
{
    PORT_SetPinMux(PORTC, 13U, kPORT_MuxAlt3);	//UART4 CTS PTC13
    PORT_SetPinMux(PORTC, 14U, kPORT_MuxAlt3);	//UART4 RX PTC14
    PORT_SetPinMux(PORTC, 15U, kPORT_MuxAlt3);	//UART4 TX PTC15
    PORT_SetPinMux(PORTE, 27U, kPORT_MuxAlt3);	//UART4 RTS PTE27
}

void sendMessage(const char *format, ...)
{
	va_list args;

	char* str = (char*)pvPortMalloc(250 * sizeof(char));

	va_start(args, format);
	vsprintf(str, format, args);

	if(xQueueSendToBack(uart_queue, (void *) &str, portMAX_DELAY) != pdPASS )
	{
		PRINTF("Send message to uart_queue failed!.\r\n");
		while (1);
	}

	va_end(args);
}

void setupUART()
{
	uart_config_t config;
	UART_GetDefaultConfig(&config);
	config.baudRate_Bps = 57600;
	config.enableTx = true;
	config.enableRx = true;
	config.enableRxRTS = true;
	config.enableTxCTS = true;
	UART_Init(TARGET_UART, &config, CLOCK_GetFreq(kCLOCK_BusClk));
	/******** Enable Interrupts *********/
	UART_EnableInterrupts(TARGET_UART, kUART_RxDataRegFullInterruptEnable);
	NVIC_SetPriority(UART4_RX_TX_IRQn, 2);
	EnableIRQ(UART4_RX_TX_IRQn);
}

void uartTask(void* pvParameters)
{
	char* welcome_message = "UART task started\n\r";
	char* received_str;
	BaseType_t status;

	UART_WriteBlocking(TARGET_UART, welcome_message, strlen(welcome_message));

	while(1)
	{
		status = xQueueReceive(uart_queue, (void *) &received_str, portMAX_DELAY);
		if (status != pdPASS)
		{
			PRINTF("Queue Send failed!.\r\n");
			while (1);
		}

		UART_WriteBlocking(TARGET_UART, received_str, strlen(received_str));
		vPortFree(received_str);
	}
}

void UART4_RX_TX_IRQHandler()
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	UART_GetStatusFlags(TARGET_UART);
	char ch = UART_ReadByte(TARGET_UART);
	printf("char %c\n", ch);
	switch(ch)
	{
	case 'a':
		xEventGroupSetBitsFromISR(event_group, LEFT_BIT, &
				xHigherPriorityTaskWoken);
		break;
	case 's':
		xEventGroupSetBitsFromISR(event_group, DOWN_BIT, &
				xHigherPriorityTaskWoken);
		break;
	case 'd':
		xEventGroupSetBitsFromISR(event_group, RIGHT_BIT, &
				xHigherPriorityTaskWoken);
		break;
	case 'w':
		xEventGroupSetBitsFromISR(event_group, UP_BIT, &
				xHigherPriorityTaskWoken);
		break;
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void terminalControlTask(void* pvParameters)
{
	//Terminal Control Task implementation

	EventBits_t bits;
	int servo_angle;
	int motor_speed;
	char speed;
	motor_value motor_val;

	BaseType_t status_angle;
	BaseType_t status_motor;
	BaseType_t status_LED;

	while(1)
	{
		bits = xEventGroupWaitBits(event_group,
				LEFT_BIT | RIGHT_BIT | UP_BIT | DOWN_BIT,
				pdTRUE,
				pdFALSE,
				portMAX_DELAY);
		status_term = xSemaphoreTake(rc_hold_semaphore, portMAX_DELAY);

		if (status_term != pdPASS)
		{
			PRINTF("Failed to acquire producer1_semaphore\r\n");
			while (1);
		}

		if((bits & LEFT_BIT) == LEFT_BIT)
		{
			motor_speed = 25;
			servo_angle = 100;
		}
		if((bits & RIGHT_BIT) == RIGHT_BIT)
		{
			motor_speed = 25;
			servo_angle = -100;
		}
		if((bits & UP_BIT) == UP_BIT)
		{
			motor_speed = 25;
			servo_angle = 0;
		}
		if((bits & DOWN_BIT) == DOWN_BIT)
		{
			motor_speed = -25;
			servo_angle = 0;
		}
		//set led
		char speed = 'm';

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

		vTaskDelay(20 / portTICK_PERIOD_MS);

		xSemaphoreGive(rc_hold_semaphore);
	}
}

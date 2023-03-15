#include "Accelerometer_Component.h"

static fxos_handle_t fxosHandle;
static uint8_t data_scale;

volatile uint16_t SampleEventFlag;
fxos_handle_t g_fxosHandle;
uint8_t g_sensor_address[] = {0x1CU, 0x1EU, 0x1DU, 0x1FU};
uint8_t g_sensorRange      = 0;
uint8_t g_dataScale        = 1;

int16_t g_Ax_Raw = 0;
int16_t g_Ay_Raw = 0;
int16_t g_Az_Raw = 0;

double g_Ax = 0;
double g_Ay = 0;
double g_Az = 0;

int16_t g_Ax_buff[MAX_ACCEL_AVG_COUNT] = {0};
int16_t g_Ay_buff[MAX_ACCEL_AVG_COUNT] = {0};
int16_t g_Az_buff[MAX_ACCEL_AVG_COUNT] = {0};

double g_Yaw    = 0;
double g_Yaw_LP = 0;
double g_Pitch  = 0;
double g_Roll   = 0;

bool g_FirstRun = true;


void setupAccelerometerComponent()
{
	BaseType_t status;

	setupAccelerometerPins();
	voltageRegulator_enable();
	accelerometer_enable();

	spi_init();

	/*************** Accelerometer Task ***************/
	//Create Accelerometer Task
	status = xTaskCreate(accelerometerTask, "acc", 200, (void*)led_queue, 2, NULL);
	if (status != pdPASS)
		PRINTF("Task creation failed!.\r\n");
}

void setupAccelerometerPins()
{
	PORT_SetPinMux(PORTB, 10U, kPORT_MuxAlt2);
	PORT_SetPinMux(PORTB, 11U, kPORT_MuxAlt2);
	PORT_SetPinMux(PORTB, 16U, kPORT_MuxAlt2);
	PORT_SetPinMux(PORTB, 17U, kPORT_MuxAlt2);
	PORT_SetPinMux(PORTA, 25U, kPORT_MuxAsGpio);
	PORT_SetPinMux(PORTB, 8U, kPORT_MuxAsGpio);                  /* UART 0 transmit data source select: UART0_TX pin. */
}

void voltageRegulator_enable()
{
	//Enable voltage Regulator
	gpio_pin_config_t pin_config = {
			.pinDirection = kGPIO_DigitalOutput,
			.outputLogic = 0U};
	GPIO_PinInit(GPIOB, 8, &pin_config);
	GPIO_PinWrite(GPIOB, 8, 1U);
}

void accelerometer_enable()
{
	//Enable accelerometer
	gpio_pin_config_t pin_config = {
			.pinDirection = kGPIO_DigitalOutput,
			.outputLogic = 0U};
	GPIO_PinInit(GPIOA, 25, &pin_config);
	GPIO_PinWrite(GPIOA, 25, 0U);
}

void spi_init()
{
	//Initialize SPI
	dspi_master_config_t masterConfig;
	/* Master config */
	masterConfig.whichCtar = kDSPI_Ctar0;
	masterConfig.ctarConfig.baudRate = 500000;
	masterConfig.ctarConfig.bitsPerFrame = 8U;
	masterConfig.ctarConfig.cpol = kDSPI_ClockPolarityActiveHigh;
	masterConfig.ctarConfig.cpha = kDSPI_ClockPhaseFirstEdge;
	masterConfig.ctarConfig.direction = kDSPI_MsbFirst;
	masterConfig.ctarConfig.pcsToSckDelayInNanoSec = 1000000000U / 500000;
	masterConfig.ctarConfig.lastSckToPcsDelayInNanoSec = 1000000000U / 500000;
	masterConfig.ctarConfig.betweenTransferDelayInNanoSec = 1000000000U / 500000;
	masterConfig.whichPcs = kDSPI_Pcs0;
	masterConfig.pcsActiveHighOrLow = kDSPI_PcsActiveLow;
	masterConfig.enableContinuousSCK = false;
	masterConfig.enableRxFifoOverWrite = false;
	masterConfig.enableModifiedTimingFormat = false;
	masterConfig.samplePoint = kDSPI_SckToSin0Clock;
	DSPI_MasterInit(SPI1, &masterConfig, BUS_CLK);
}

status_t SPI_Send(uint8_t regAddress, uint8_t value)
{
	dspi_transfer_t masterXfer;
	uint8_t *masterTxData = (uint8_t*)malloc(3);
	uint8_t *masterRxData = (uint8_t*)malloc(3);
	masterTxData[0] = regAddress | 0x80; //Clear the most significant bit
	masterTxData[1] = regAddress & 0x80; //Clear the least significant 7 bits
	masterTxData[2] = value;
	masterXfer.txData = masterTxData;
	masterXfer.rxData = masterRxData;
	masterXfer.dataSize = 3;
	masterXfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs0 |
			kDSPI_MasterPcsContinuous;
	status_t ret = DSPI_MasterTransferBlocking(SPI1, &masterXfer);
	free(masterTxData);
	free(masterRxData);
	return ret;
}

status_t SPI_receive(uint8_t regAddress, uint8_t *rxBuff, uint8_t rxBuffSize)
{
	dspi_transfer_t masterXfer;
	uint8_t *masterTxData = (uint8_t*)malloc(rxBuffSize + 2);
	uint8_t *masterRxData = (uint8_t*)malloc(rxBuffSize + 2);
	masterTxData[0] = regAddress & 0x7F; //Clear the most significant bit
	masterTxData[1] = regAddress & 0x80; //Clear the least significant 7 bits
	masterXfer.txData = masterTxData;
	masterXfer.rxData = masterRxData;
	masterXfer.dataSize = rxBuffSize + 2;
	masterXfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs0 |
			kDSPI_MasterPcsContinuous;
	status_t ret = DSPI_MasterTransferBlocking(SPI1, &masterXfer);
	memcpy(rxBuff, &masterRxData[2], rxBuffSize);
	free(masterTxData);
	free(masterRxData);
	return ret;
}

static void Sensor_ReadData(int16_t *Ax, int16_t *Ay, int16_t *Az)
{
	fxos_data_t fxos_data;

	if (kStatus_Success != FXOS_ReadSensorData(&g_fxosHandle, &fxos_data))
	{
		PRINTF("Failed to read acceleration data!\r\n");
	}
	/* Get the accel data from the sensor data structure in 14 bit left format data*/
	*Ax = (int16_t)((uint16_t)((uint16_t)fxos_data.accelXMSB << 8) | (uint16_t)fxos_data.accelXLSB) / 4U;
	*Ay = (int16_t)((uint16_t)((uint16_t)fxos_data.accelYMSB << 8) | (uint16_t)fxos_data.accelYLSB) / 4U;
	*Az = (int16_t)((uint16_t)((uint16_t)fxos_data.accelZMSB << 8) | (uint16_t)fxos_data.accelZLSB) / 4U;
	*Ax *= g_dataScale;
	*Ay *= g_dataScale;
	*Az *= g_dataScale;
}



void accelerometerTask(void* pvParameters)
{
	motor_value motor_val;
	BaseType_t status;

	//Accelerometer task implementation
	fxos_handle_t fxosHandle = {0};
	uint8_t sensorRange = 0;
	fxos_config_t config = {0};
	uint8_t dataScale = 0;
	uint16_t i              = 0;
	uint16_t loopCounter    = 0;
	double sinAngle         = 0;
	double cosAngle         = 0;
	double Bx               = 0;
	double By               = 0;

	uint8_t array_addr_size = 0;
	fxos_data_t sensorData = {0};
	status_t result = kStatus_Fail;
	volatile int16_t xAngle = 0;
	volatile int16_t yAngle = 0;

	/* Configure the SPI function */
	config.SPI_writeFunc = SPI_Send;
	config.SPI_readFunc = SPI_receive;
	result = FXOS_Init(&fxosHandle, &config);
	if (result != kStatus_Success)
	{
		PRINTF("\r\nSensor device initialize failed!\r\n");
		return -1;
	}
	/* Get sensor range */
	if (FXOS_ReadReg(&fxosHandle, XYZ_DATA_CFG_REG, &sensorRange, 1) != kStatus_Success)
	{
		return -1;
	}
	if (sensorRange == 0x00)
	{
		dataScale = 2U;
	}
	else if (sensorRange == 0x01)
	{
		dataScale = 4U;
	}
	else if (sensorRange == 0x10)
	{
		dataScale = 8U;
	}
	else
	{
	}

	/* Initialize sensor devices */
	array_addr_size = sizeof(g_sensor_address) / sizeof(g_sensor_address[0]);
	for (i = 0; i < array_addr_size; i++)
	{
		config.slaveAddress = g_sensor_address[i];
		/* Initialize accelerometer sensor */
		result = FXOS_Init(&g_fxosHandle, &config);
		if (result == kStatus_Success)
		{
			break;
		}
	}

	if (kStatus_Success != result)
	{
		PRINTF("\r\nSensor device initialize failed!\r\n");
	}

	/* Infinite loops */
	for (;;)
	{
//		if (SampleEventFlag == 1) /* Fix loop */
//		{
//			SampleEventFlag = 0;
			g_Ax_Raw        = 0;
			g_Ay_Raw        = 0;
			g_Az_Raw        = 0;
			g_Ax            = 0;
			g_Ay            = 0;
			g_Az            = 0;


			/* Read sensor data */
			Sensor_ReadData(&g_Ax_Raw, &g_Ay_Raw, &g_Az_Raw);

			/* Average accel value */
			for (i = 1; i < MAX_ACCEL_AVG_COUNT; i++)
			{
				g_Ax_buff[i] = g_Ax_buff[i - 1];
				g_Ay_buff[i] = g_Ay_buff[i - 1];
				g_Az_buff[i] = g_Az_buff[i - 1];
			}

			g_Ax_buff[0] = g_Ax_Raw;
			g_Ay_buff[0] = g_Ay_Raw;
			g_Az_buff[0] = g_Az_Raw;

			for (i = 0; i < MAX_ACCEL_AVG_COUNT; i++)
			{
				g_Ax += (double)g_Ax_buff[i];
				g_Ay += (double)g_Ay_buff[i];
				g_Az += (double)g_Az_buff[i];
			}

			g_Ax /= MAX_ACCEL_AVG_COUNT;
			g_Ay /= MAX_ACCEL_AVG_COUNT;
			g_Az /= MAX_ACCEL_AVG_COUNT;


			/* Calculate roll angle g_Roll (-180deg, 180deg) and sin, cos */
			g_Roll   = atan2(g_Ay, g_Az) * RadToDeg;
			sinAngle = sin(g_Roll * DegToRad);
			cosAngle = cos(g_Roll * DegToRad);

			/* De-rotate by roll angle g_Roll */
			g_Az = g_Ay * sinAngle + g_Az * cosAngle;

			/* Calculate pitch angle g_Pitch (-90deg, 90deg) and sin, cos*/
			g_Pitch  = atan2(-g_Ax, g_Az) * RadToDeg;
			sinAngle = sin(g_Pitch * DegToRad);
			cosAngle = cos(g_Pitch * DegToRad);

			motor_val.source = 'a';
			motor_val.speed = (int)(g_Pitch/2);

			status = xQueueSendToBack(motor_queue, (void*)&motor_val, portMAX_DELAY);
			if (status != pdPASS)
			{
				PRINTF("Motor Speed Queue Send failed!.\r\n");
				while (1);
			}

			vTaskDelay(250 / portTICK_PERIOD_MS);

//		}
	} /* End infinite loops */
}

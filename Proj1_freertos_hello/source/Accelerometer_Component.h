#ifndef ACCELEROMETER_COMPONENT_H
#define ACCELEROMETER_COMPONENT_H

#include "math.h"
#include "pin_mux.h"
#include "fsl_port.h"
#include "fsl_dspi.h"
#include "fsl_fxos.h"
#include "fsl_gpio.h"
#include "fsl_debug_console.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "Motor_Control_Component.h"
#include "Terminal_Component.h"

#define I2C_RELEASE_BUS_COUNT 100U
#define MAX_ACCEL_AVG_COUNT 25U
#define HWTIMER_PERIOD      10000U
/* multiplicative conversion constants */
#define DegToRad 0.017453292
#define RadToDeg 57.295779

#define BOARD_GPIO_ACCEL_SCL_GPIO GPIOD

void setupAccelerometerComponent();

void setupAccelerometerPins();
void voltageRegulator_enable();
void accelerometer_enable();
void spi_init();

void accelerometerTask(void* pvParameters);

status_t SPI_Send(uint8_t regAddress, uint8_t value);
status_t SPI_receive(uint8_t regAddress, uint8_t *rxBuff, uint8_t rxBuffSize);


#endif /* ACCELEROMETER_COMPONENT_H */

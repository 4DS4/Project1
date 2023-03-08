#include "Motor_Control_Component.h"



//	dutyCycleSpeed = speed * 0.0125f/100.0f + 0.07025;	//use these conversions
//	dutyCycleAngle = angle * 0.0125f/100.0f + 0.079;


QueueHandle_t motor_queue;
QueueHandle_t angle_queue;

void setupMotorComponent()
{
	setupMotorPins();

	setupDCMotor();
	setupServo();

    /*************** Motor Task ***************/
	//Create Motor Queue
	//Create Motor Task

    /*************** Position Task ***************/
	//Create Angle Queue
	//Create Position Task
}

void setupMotorPins()
{
    //Configure PWM pins for DC and Servo motors
    /* Port A Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortA);
    /* Port B Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortB);
    /* Port C Clock Gate Control: Clock enabled */
    CLOCK_EnableClock(kCLOCK_PortC);

    PORT_SetPinMux(PORTC, 1U, kPORT_MuxAlt4);
    PORT_SetPinMux(PORTA, 6U, kPORT_MuxAlt3);

}

void setupPWM(ftm_chnl_t channel, FTM_Type* type)
{
	ftm_config_t ftmInfo;
	ftm_chnl_pwm_signal_param_t ftmParam;
	ftm_pwm_level_select_t pwmLevel = kFTM_HighTrue;
	ftmParam.chnlNumber = channel;
	ftmParam.level = pwmLevel;
	ftmParam.dutyCyclePercent = 7;
	ftmParam.firstEdgeDelayPercent = 0U;
	ftmParam.enableComplementary = false;
	ftmParam.enableDeadtime = false;
	FTM_GetDefaultConfig(&ftmInfo);
	ftmInfo.prescale = kFTM_Prescale_Divide_128;
	FTM_Init(type, &ftmInfo);
	FTM_SetupPwm(type, &ftmParam, 1U, kFTM_EdgeAlignedPwm, 50U, CLOCK_GetFreq(
			kCLOCK_BusClk));
	FTM_StartTimer(type, kFTM_SystemClock);
}


void setupDCMotor()
{
	//Initialize PWM for DC motor
    setupPWM(FTM_CHANNEL_DC_MOTOR, FTM_MOTORS);
}

void setupServo()
{
	//Initialize PWM for Servo motor
    setupPWM(FTM_CHANNEL_SERVO, FTM_MOTORS);
}

void updatePWM_dutyCycle(ftm_chnl_t channel, float dutyCycle)
{
	uint32_t cnv, cnvFirstEdge = 0, mod;

	/* The CHANNEL_COUNT macro returns -1 if it cannot match the FTM instance */
	assert(-1 != FSL_FEATURE_FTM_CHANNEL_COUNTn(FTM_MOTORS));

	mod = FTM_MOTORS->MOD;
	if(dutyCycle == 0U)
	{
		/* Signal stays low */
		cnv = 0;
	}
	else
	{
		cnv = mod * dutyCycle;
		/* For 100% duty cycle */
		if (cnv >= mod)
		{
			cnv = mod + 1U;
		}
	}

	FTM_MOTORS->CONTROLS[channel].CnV = cnv;
}

void motorTask(void* pvParameters)
{
	//Motor task implementation
}

void positionTask(void* pvParameters)
{
	//Position task implementation
}

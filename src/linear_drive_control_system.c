#include "board.h"
#include <cr_section_macros.h>
#include "arm_math.h"
#include "oled.h"

/* ------------------------------------------------------------------------ */
/* -------------------------------- MACROS -------------------------------- */
/* ------------------------------------------------------------------------ */

/* PWM output frequency */
#define PWMFS 			100000
#define MOTOR_PORT 		1

/* Motor direction A/B pins */
#define MOTOR_DIRA 		14
#define MOTOR_DIRB 		15
#define ENCODER_PORT	1
/* Motor encoder A/B channels */
#define ENCODER_CHA 	31
#define ENCODER_CHB 	16

/* Input encoder SIA/SIB channels */
#define CONTROL_CHA 	27
#define CONTROL_CHB 	26
/* Input encoder switch */
#define CONTROL_CHC 	28

/* Limit switches */
#define LIMIT_SW_PORT	0
#define LIMIT_SW_L 		22
#define LIMIT_SW_R 		23

/* PWM output */
#define PWM_PORT 		1
#define PWM_PIN  		13

#define ENCODER_IOCON_FUNC (IOCON_FUNC0 | IOCON_DIGMODE_EN | IOCON_MODE_PULLUP | IOCON_HYS_EN)
#define LIMIT_SW_IOCON_FUNC	(IOCON_FUNC0 | IOCON_DIGMODE_EN | IOCON_MODE_PULLUP)

/* Speed */
#define CALIB_POWER 20

/* ------------------------------------------------------------------------ */
/* ------------------------------- VARIABLES ------------------------------ */
/* ------------------------------------------------------------------------ */

char sbuffer[30];
volatile int8_t dataControl = 0;
volatile int16_t motorPower = 0;
volatile float32_t motorVelocity = 0;
volatile bool run = false;
volatile uint16_t tdiv = 0;

/* PID related */
volatile int32_t dataEncoder = 0, dataEncoder_1 = 0;

arm_pid_instance_f32 velCoef, posCoef;
volatile float32_t velU = 0, velE = 0, velX = 0;
volatile float32_t posU = 0, posE = 0;

volatile bool leftLimitStatus = false;
volatile bool rightLimitStatus = false;

/* Position related */
uint32_t maxValOnRight;
uint32_t newPosition;

typedef enum { OLED_DESKTOP, OLED_CALIB, OLED_IDLE } OLED_VIEW;
OLED_VIEW currentView;
typedef void(*CALLBACK)(void);

bool calibrated = false;

/* TODO Remove stub */
//int32_t tempPower;

/* ------------------------------------------------------------------------ */
/* --------------------------- PUBLIC FUNCTIONS --------------------------- */
/* ------------------------------------------------------------------------ */

static void OLED_show()
{
	OLED_Clear_Screen(0);
	sprintf(sbuffer, "IE %d %d %d", 0, maxValOnRight, maxValOnRight/2);
	OLED_Puts(0, 0, sbuffer);
	sprintf(sbuffer, "LL: %d | RL %d", leftLimitStatus, rightLimitStatus);
	OLED_Puts(0, 2, sbuffer);
	sprintf(sbuffer, "Position(ENC): %d", dataEncoder);
	OLED_Puts(0, 4, sbuffer);
	sprintf(sbuffer, "PU %.2f | VU %.2f", posU, velU);
	OLED_Puts(0, 6, sbuffer);
	OLED_Refresh_Gram();
}

static void OLED_calibrate()
{
	OLED_Clear_Screen(0);
	OLED_Puts(0, 0, "CALIBRATION");
	sprintf(sbuffer, "LL: %d | RL %d", leftLimitStatus, rightLimitStatus);
	OLED_Puts(0, 2, sbuffer);
	sprintf(sbuffer, "Position(ENC): %d", dataEncoder);
	OLED_Puts(0, 4, sbuffer);
	OLED_Refresh_Gram();
}

/* TODO Remove stub */
//void SysTick_Handler(void)
//{
//	if (run) {
//		if (tempPower > 0) { dataEncoder++; }
//		if (tempPower < 0) { dataEncoder--; }
//	}
//}

static void systemCalibrate(CALLBACK callback)
{
	if (calibrated) return;

	/* Reach left limit and reset encoder readouts */
	dataControl = -CALIB_POWER;
	run = true;
	while (leftLimitStatus) {
		if (callback) callback();
	}
	dataEncoder = dataEncoder_1 = 0;

	/* Reach right limit and set max encoder ticks */
	dataControl = CALIB_POWER;
	while (rightLimitStatus) {
		if (callback) callback();
	}
	maxValOnRight = dataEncoder;

	run = false;
	dataControl = 0;
	calibrated = true;

	/* Set 50% offset (middle) */
//	dataControl = CALIB_POWER;
//	newPosition = maxValOnRight / 2;
//	run = true;
}

static void calcMotorPower(void)
{
	if (!calibrated) {
		velU = dataControl / 10.0;
	} else {
		/* Position PID */
		posE = newPosition - dataEncoder;
		posU = arm_pid_f32(&posCoef, posE);

		velX = dataControl / 10.0;

		if (velX < 0 && posU < velX) posU = velX;
		if (velX > 0 && posU > velX) posU = velX;

		/* Velocity PID */
		velE = posU - motorVelocity;
		velU = arm_pid_f32(&velCoef, velE);
	}
}

static void resetCoef(void)
{
	velU = 0;
	velE = 0;
	posE = 0;
	posU = 0;
	arm_pid_reset_f32(&velCoef);
	arm_pid_reset_f32(&posCoef);
}

static void updateLimitSWStatus(void)
{
	rightLimitStatus = Chip_GPIO_GetPinState(LPC_GPIO_PORT, LIMIT_SW_PORT, LIMIT_SW_R) ?  true : false;
	leftLimitStatus = Chip_GPIO_GetPinState(LPC_GPIO_PORT, LIMIT_SW_PORT, LIMIT_SW_L) ? true : false;
}

static void motorPinsInit(void)
{
	Chip_IOCON_PinMuxSet(LPC_IOCON, MOTOR_PORT, MOTOR_DIRA, IOCON_FUNC0);
	Chip_IOCON_PinMuxSet(LPC_IOCON, MOTOR_PORT, MOTOR_DIRB, IOCON_FUNC0);
	Chip_IOCON_PinMuxSet(LPC_IOCON, ENCODER_PORT, ENCODER_CHA, ENCODER_IOCON_FUNC);
	Chip_IOCON_PinMuxSet(LPC_IOCON, ENCODER_PORT, ENCODER_CHB, ENCODER_IOCON_FUNC);
	Chip_GPIO_WriteDirBit(LPC_GPIO_PORT, MOTOR_PORT, MOTOR_DIRA, 1);
	Chip_GPIO_WriteDirBit(LPC_GPIO_PORT, MOTOR_PORT, MOTOR_DIRB, 1);
	Chip_GPIO_WritePortBit(LPC_GPIO_PORT, MOTOR_PORT, MOTOR_DIRA, 1);
	Chip_GPIO_WritePortBit(LPC_GPIO_PORT, MOTOR_PORT, MOTOR_DIRB, 1);

	/* Encoder A channel as falling edge interrupt */
	Chip_SYSCTL_SetPinInterrupt(0, ENCODER_PORT, ENCODER_CHA);
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH0);
	Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH0);
	Chip_PININT_EnableIntLow(LPC_PININT, PININTCH0);
	NVIC_SetPriority(PIN_INT0_IRQn, 0);
	NVIC_ClearPendingIRQ(PIN_INT0_IRQn);
	NVIC_EnableIRQ(PIN_INT0_IRQn);
}

/**
 * @brief Init control encoder pins
 */
static void encoderPinsInit(void)
{
	/* Encoder pins as GPIO input with pullup enabled */
	Chip_IOCON_PinMuxSet(LPC_IOCON, ENCODER_PORT, CONTROL_CHA, ENCODER_IOCON_FUNC);
	Chip_IOCON_PinMuxSet(LPC_IOCON, ENCODER_PORT, CONTROL_CHB, ENCODER_IOCON_FUNC);
	Chip_IOCON_PinMuxSet(LPC_IOCON, ENCODER_PORT, CONTROL_CHC, ENCODER_IOCON_FUNC);

	/* Encoder SIA and switch as falling edge interrupts */
	Chip_SYSCTL_SetPinInterrupt(1, ENCODER_PORT, CONTROL_CHA);
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH1);
	Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH1);
	Chip_PININT_EnableIntLow(LPC_PININT, PININTCH1);
	NVIC_SetPriority(PIN_INT1_IRQn, 0);
	NVIC_ClearPendingIRQ(PIN_INT1_IRQn);
	NVIC_EnableIRQ(PIN_INT1_IRQn);
	Chip_SYSCTL_SetPinInterrupt(2, ENCODER_PORT, CONTROL_CHC);
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH2);
	Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH2);
	Chip_PININT_EnableIntLow(LPC_PININT, PININTCH2);
	NVIC_SetPriority(PIN_INT2_IRQn, 2);
	NVIC_ClearPendingIRQ(PIN_INT2_IRQn);
	NVIC_EnableIRQ(PIN_INT2_IRQn);
}

static void timerInit(void)
{
	uint32_t timerFreq = Chip_Clock_GetSystemClockRate();
	Chip_TIMER_Init(LPC_TIMER16_0);
	Chip_TIMER_PrescaleSet(LPC_TIMER16_0, (timerFreq/PWMFS)-1);
	Chip_TIMER_SetMatch(LPC_TIMER16_0, 0, 0);
	Chip_TIMER_SetMatch(LPC_TIMER16_0, 1, 0);
	Chip_TIMER_SetMatch(LPC_TIMER16_0, 2, 0);
	Chip_TIMER_SetMatch(LPC_TIMER16_0, 3, 99);
	Chip_TIMER_ResetOnMatchEnable(LPC_TIMER16_0, 3);
	LPC_TIMER16_0->PWMC = 7;
	Chip_TIMER_MatchEnableInt(LPC_TIMER16_0, 3);
	NVIC_SetPriority(TIMER_16_0_IRQn, 1);
	NVIC_ClearPendingIRQ(TIMER_16_0_IRQn);
	NVIC_EnableIRQ(TIMER_16_0_IRQn);
	Chip_TIMER_Reset(LPC_TIMER16_0);
	Chip_TIMER_Enable(LPC_TIMER16_0);
	Chip_IOCON_PinMuxSet(LPC_IOCON, PWM_PORT, PWM_PIN, IOCON_FUNC2 ); /* PIO1_13 connected to MAT0 */
}

static void limitSWInit(void)
{
	Chip_IOCON_PinMuxSet(LPC_IOCON, LIMIT_SW_PORT, LIMIT_SW_L, LIMIT_SW_IOCON_FUNC);
	Chip_IOCON_PinMuxSet(LPC_IOCON, LIMIT_SW_PORT, LIMIT_SW_R, LIMIT_SW_IOCON_FUNC);

	/* Limit switches as falling edge interrupts */
	Chip_SYSCTL_SetPinInterrupt(3, LIMIT_SW_PORT, LIMIT_SW_L);
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH3);
	Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH3);
	Chip_PININT_EnableIntLow(LPC_PININT, PININTCH3);
	NVIC_SetPriority(PIN_INT3_IRQn, 0);
	NVIC_ClearPendingIRQ(PIN_INT3_IRQn);
	NVIC_EnableIRQ(PIN_INT3_IRQn);

	Chip_SYSCTL_SetPinInterrupt(4, LIMIT_SW_PORT, LIMIT_SW_R);
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH4);
	Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH4);
	Chip_PININT_EnableIntLow(LPC_PININT, PININTCH4);
	NVIC_SetPriority(PIN_INT4_IRQn, 0);
	NVIC_ClearPendingIRQ(PIN_INT4_IRQn);
	NVIC_EnableIRQ(PIN_INT4_IRQn);

	Chip_SYSCTL_SetPinInterrupt(5, LIMIT_SW_PORT, LIMIT_SW_L);
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH5);
	Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH5);
	Chip_PININT_EnableIntHigh(LPC_PININT, PININTCH5);
	NVIC_SetPriority(PIN_INT5_IRQn, 0);
	NVIC_ClearPendingIRQ(PIN_INT5_IRQn);
	NVIC_EnableIRQ(PIN_INT5_IRQn);

	Chip_SYSCTL_SetPinInterrupt(6, LIMIT_SW_PORT, LIMIT_SW_R);
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH6);
	Chip_PININT_SetPinModeEdge(LPC_PININT, PININTCH6);
	Chip_PININT_EnableIntHigh(LPC_PININT, PININTCH6);
	NVIC_SetPriority(PIN_INT6_IRQn, 0);
	NVIC_ClearPendingIRQ(PIN_INT6_IRQn);
	NVIC_EnableIRQ(PIN_INT6_IRQn);
}

/**
 * @brief 				DC motor control
 * @param power 		Motor power
 * @param fastbreak		True for fastbreak, false otherwise
 */
void motor(int16_t power, bool fastbreak)
{
	if (power > 100) power = 100;
	if (power < -100) power = -100;
	if (power < 0 && leftLimitStatus) {
		Chip_GPIO_WritePortBit(LPC_GPIO_PORT, MOTOR_PORT, MOTOR_DIRA, 1);
		Chip_GPIO_WritePortBit(LPC_GPIO_PORT, MOTOR_PORT, MOTOR_DIRB, 0);
	}
	else if (power > 0 && rightLimitStatus) {
		Chip_GPIO_WritePortBit(LPC_GPIO_PORT, MOTOR_PORT, MOTOR_DIRA, 0);
		Chip_GPIO_WritePortBit(LPC_GPIO_PORT, MOTOR_PORT, MOTOR_DIRB, 1);
	}
	else {
		Chip_GPIO_WritePortBit(LPC_GPIO_PORT, MOTOR_PORT, MOTOR_DIRA, fastbreak);
		Chip_GPIO_WritePortBit(LPC_GPIO_PORT, MOTOR_PORT, MOTOR_DIRB, fastbreak);
	}

	if ((power < 0 && !leftLimitStatus) || (power > 0 && !rightLimitStatus)) {
		resetCoef();
	}

	/* TODO Remove stub */
	//tempPower = power;

	power = (power < 0) ? -power : power;
	Chip_TIMER_SetMatch(LPC_TIMER16_0, 0, 100 - power);
}

/**
 * @brief Motor encoder ISR handler
 */
void PIN_INT0_IRQHandler(void)
{
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH0);
	if (LPC_GPIO_PORT->B[ENCODER_PORT][ENCODER_CHB])
	{
		dataEncoder++;
	}
	else {
		dataEncoder--;
	}

}

/**
 * @brief Input encoder knob ISR handler
 */
void PIN_INT1_IRQHandler(void)
{
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH1);
	if (LPC_GPIO_PORT->B[ENCODER_PORT][CONTROL_CHB]) {
		if(dataControl < 20)
			dataControl++;
	} else {
		if(dataControl > -20)
			dataControl--;
	}
}

/**
 * @brief Input encoder switch ISR handler
 */
void PIN_INT2_IRQHandler(void)
{
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH2);
	run = !run;
}

void PIN_INT3_IRQHandler(void)
{
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH3);
	leftLimitStatus = false;
}

void PIN_INT4_IRQHandler(void)
{
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH4);
	rightLimitStatus = false;
}

void PIN_INT5_IRQHandler(void)
{
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH5);
	leftLimitStatus = true;
}

void PIN_INT6_IRQHandler(void)
{
	Chip_PININT_ClearIntStatus(LPC_PININT, PININTCH6);
	rightLimitStatus = true;
}

/**
 * @brief Timer ISR Handler
 */
void TIMER16_0_IRQHandler(void)
{
	if (Chip_TIMER_MatchPending(LPC_TIMER16_0, 3)) {
		Chip_TIMER_ClearMatch(LPC_TIMER16_0, 3);
		tdiv++;
		if(tdiv >= 100){
			tdiv = 0;
			motorVelocity= (dataEncoder - dataEncoder_1) / 48.0;
			dataEncoder_1 = dataEncoder;
		}
		if(run) {
			calcMotorPower();
		}
		else {
			resetCoef();
		}

		motorPower = (int) velU;
		motor(motorPower, 1);
	}
}

/**
 * @brief 	Application entry point
 * @return	Zero for success, non-zero for failure
 */
int main(void)
{
	/* Generic initialization */
	SystemCoreClockUpdate();
	Board_Init();
	Board_LED_Set(0, false);
	Chip_Clock_EnablePeriphClock(SYSCTL_CLOCK_PINT); /* PININT clock */

	/* TODO Remove stub */
	//SysTick_Config(SystemCoreClock / 500);

	velCoef.Kp = 10;
	velCoef.Ki = 0.1;
	velCoef.Kd = 0;
	arm_pid_init_f32(&velCoef, 1);

	posCoef.Kp = 10;
	posCoef.Ki = 0.001;
	posCoef.Kd = 0;
	arm_pid_init_f32(&posCoef, 1);

	motorPinsInit();
	encoderPinsInit();
	timerInit();
	limitSWInit();
	OLED_Init();
	OLED_Clear_Screen(0);
	updateLimitSWStatus();

	currentView = OLED_CALIB;
	systemCalibrate(OLED_calibrate);
	currentView = OLED_DESKTOP;

	while(1) {
		switch (currentView) {
		case OLED_DESKTOP:
			OLED_show();
			break;
		case OLED_CALIB:
			OLED_calibrate();
			break;
		}
	}
	return 0;
}

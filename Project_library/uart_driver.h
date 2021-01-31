/* Header */
/**
  ******************************************************************************
  * @file    uart_driver.h
  * @brief   Uart Project Driver
  * @author  Muhammed Sadun Çakmaklı
  * @date    19-01-2021
  ******************************************************************************
  * @attention
  *
  *
 ******************************************************************************
  */
/* END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __UART_DRIVER
#define __UART_DRIVER

/* Private includes ----------------------------------------------------------*/
#include "step_driver.h"
/* END Includes */

/* Exported types ------------------------------------------------------------*/
typedef enum
{
	STEP_ENABLE,
	STEP_DISABLE
}Step_Enable;

typedef enum
{
	STEP_PULSE,
	STEP_NOPULSE
}Step_Pulse;

typedef enum
{
	STANDART_MODE,
	PULSE_TRIGGERED_STANDART_MODE,
	LIMITED_ANGLE_MODE,
	PULSE_TRIGGERED_LIMITED_ANGLE_MODE,
	ONE_DIRECTION_PULSE_TRIGGERED_LIMITED_ANGLE_MODE,
	MANUEL_MODE,
	CALIBRATION_MODE,
	PASSIVE_MODE
}StepMode;

typedef struct
{
	Step_Enable Enable;
	Step_Direction Direction;
	Step_Pulse Pulse;
}ManualCommand;

typedef struct
{
	StepMode Step_Mode;
	Step_Power Trigger;
	Step_Direction Step_Direction;
	uint16_t degree;
	uint16_t delay;

	ManualCommand manuelCommand;

}StepCommand;

typedef struct
{
	uint16_t MAX_PULSE_DELAY;
	uint16_t MAX_PULSE_HIGH_SIGNAL;
	uint16_t NUMBER_OF_SIGNALS_IN_FULL_TURN;
	uint16_t NUMBER_OF_SIGNAL_IN_FULL_TURN_ENCODER;
}CalibrationSettings;

/* END ET */

/* Exported values -----------------------------------------------------------*/
volatile StepCommand stepCommand;
/* END EV
 *
 */
/* Exported constants --------------------------------------------------------*/
#define __RECEIVE_BUFFER ((uint16_t)11U)
#define __RECEIVE_COMMAND_BUFFER ((uint16_t)7U)
/* END EC */

/* Exported macro ------------------------------------------------------------*/

/* END EM */

/* Exported functions prototypes ---------------------------------------------*/
void StrCopyInSet (int first_index, int last_index);
void CommandSearch();
void UART_RECEIVE_COMMAND();

void UART_SET_RECEIVE_DMA();

__weak void MOD_CHANGE_STANDART_MODE();
__weak void MOD_CHANGE_PULSE_TRIGGERED_STANDART_MODE();
__weak void MOD_CHANGE_LIMITED_ANGLE_MODE();
__weak void MOD_CHANGE_PULSE_TRIGGERED_LIMITED_ANGLE_MODE();
__weak void MOD_CHANGE_ONE_DIRECTION_PULSE_TRIGGERED_LIMITED_ANGLE_MODE();
__weak void MOD_CHANGE_MANUEL_MODE();
__weak void MOD_CHANGE_CALIBRATION_MODE();
__weak void MOD_CHANGE_PASSIVE_MODE();
__weak void COMMAND_SPEED(int speed);
__weak void COMMAND_DEGREE(int degree);
__weak void COMMAND_RUN();
__weak void COMMAND_STOP();
__weak void COMMAND_DIRECTION_CLOCKWISE();
__weak void COMMAND_DIRECTION_ANTI_CLOCKWISE();
__weak void COMMAND_MANUAL(Step_Enable enable, Step_Direction direction, Step_Pulse pulse);
__weak void COMMAND_CONFIRM_SETTINGS();
__weak void COMMAND_CONNECTION_TEST();
__weak void DEFAULT_SETTINGS();
void STEP_MOTOR_CONTROL(void);
uint32_t CalculateDelayToSpeed(uint8_t speed);
uint32_t CalculateDegreeToPulse(uint16_t degree);
/* END EFP */


#endif /* __UART_DRIVER */

/************************ (C) COPYRIGHT Muhammed Sadun Çakmaklı *****END OF FILE****/

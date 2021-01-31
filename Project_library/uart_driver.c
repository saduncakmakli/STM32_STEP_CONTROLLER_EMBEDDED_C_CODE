/* Header */
/**
  ******************************************************************************
  * @file           : uart_driver.c
  * @brief          : Uart Project Driver
  * @author			: Muhammed Sadun ÇAKMAKLI
  * @date			: 19.01.2021
  ******************************************************************************
  * @attention
  *
  *
  ******************************************************************************
  */
/* END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
#include "uart_driver.h"
#include "step_driver.h"

/* END Includes */

/* Private typedef -----------------------------------------------------------*/

/* END PTD */

/* Private define ------------------------------------------------------------*/

/* END PD */

/* Private macro -------------------------------------------------------------*/

/* END PM */

/* Private variables ---------------------------------------------------------*/
extern UART_HandleTypeDef huart3;
uint8_t rx_buffer[__RECEIVE_BUFFER]={};
volatile uint8_t rx_command_buffer[__RECEIVE_COMMAND_BUFFER]={};
volatile StepCommand stepCommand;
volatile CalibrationSettings calibrationSettings;
volatile Step_Direction ControlTurn;
volatile uint8_t CompleteTurn; //0 = NON COMPLETE, 1,2,3,4,5,6,7 = COMPLETE

/* END PV */


/* Private function prototypes -----------------------------------------------*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	//HAL_UART_Receive_IT(&huart3, rx_buffer, __RECEIVE_BUFFER); //Uart Receive IT Restart
	//HAL_UART_Receive_DMA(&huart3, rx_buffer, __RECEIVE_BUFFER-1);Uart Receive DMA Restart
	UART_SET_RECEIVE_DMA();
	UART_RECEIVE_COMMAND();
	//HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
}

void UART_SET_RECEIVE_DMA()
{
	HAL_UART_Receive_DMA(&huart3, rx_buffer, __RECEIVE_BUFFER-1);
}

void StrCopyInSet (int first_index, int last_index)
{
	uint8_t writeSize = last_index-first_index+1;
	for (uint8_t counter = 0; counter < writeSize; counter++)
	{
		rx_command_buffer[counter] = rx_buffer[counter+first_index];
	}
}

void CommandSearch()
{
	//COMMAND MOD
	if (rx_command_buffer[0] == 'M' && rx_command_buffer[1] == 'O' && rx_command_buffer[2] == 'D')
	{
		stepCommand.Trigger = STEP_PASSIVE;
		state.Step_Counter = 0;
		//STANDART MODE
		if (rx_command_buffer[4] == 'S' && rx_command_buffer[5] == 'T' && rx_command_buffer[6] == 'D')
			{
				MOD_CHANGE_STANDART_MODE();
			}
		//PULSE TIGGERED STANDART MODE
		else if (rx_command_buffer[4] == 'P' && rx_command_buffer[5] == 'S' && rx_command_buffer[6] == 'T')
			{
				MOD_CHANGE_PULSE_TRIGGERED_STANDART_MODE();
			}
		//LIMITED ANGLE MODE
		else if (rx_command_buffer[4] == 'D' && rx_command_buffer[5] == 'G' && rx_command_buffer[6] == 'R')
			{
				MOD_CHANGE_LIMITED_ANGLE_MODE();
			}
		//PULSE TRIGGERED LIMITED ANGLE MODE
		else if (rx_command_buffer[4] == 'P' && rx_command_buffer[5] == 'D' && rx_command_buffer[6] == 'G')
			{
				MOD_CHANGE_PULSE_TRIGGERED_LIMITED_ANGLE_MODE();
			}
		//ONE DIRECTION PULSE TRIGGERED LIMITED ANGLE MODE
		else if (rx_command_buffer[4] == 'O' && rx_command_buffer[5] == 'P' && rx_command_buffer[6] == 'D')
			{
				MOD_CHANGE_ONE_DIRECTION_PULSE_TRIGGERED_LIMITED_ANGLE_MODE();
			}
		//MANUEL MOD
		else if (rx_command_buffer[4] == 'M' && rx_command_buffer[5] == 'A' && rx_command_buffer[6] == 'N')
			{
				MOD_CHANGE_MANUEL_MODE();
			}
		//CALIBRATION MODE
		else if (rx_command_buffer[4] == 'C' && rx_command_buffer[5] == 'L' && rx_command_buffer[6] == 'B')
			{
				MOD_CHANGE_CALIBRATION_MODE();
			}
		//PASSIVE MOD
		else if (rx_command_buffer[4] == 'P' && rx_command_buffer[5] == 'S' && rx_command_buffer[6] == 'V')
			{
				MOD_CHANGE_PASSIVE_MODE();
			}

	}
	//COMMAND SPEED
	else if (rx_command_buffer[0] == 'S' && rx_command_buffer[1] == 'P' && rx_command_buffer[2] == 'D')
	{
		COMMAND_SPEED((rx_command_buffer[4]-48)*100+(rx_command_buffer[5]-48)*10+(rx_command_buffer[6]-48));
	}
	//COMMAND DEGREE
	else if (rx_command_buffer[0] == 'D' && rx_command_buffer[1] == 'E' && rx_command_buffer[2] == 'G')
	{
		COMMAND_DEGREE((rx_command_buffer[4]-48)*100+(rx_command_buffer[5]-48)*10+(rx_command_buffer[6]-48));
	}
	//COMMAND RUN
	else if (rx_command_buffer[0] == 'R' && rx_command_buffer[1] == 'U' && rx_command_buffer[2] == 'N')
	{
		COMMAND_RUN();
	}
	//COMMAND STOP
	else if (rx_command_buffer[0] == 'S' && rx_command_buffer[1] == 'T' && rx_command_buffer[2] == 'P')
	{
		COMMAND_STOP();
	}
	//COMMAND DIRECTION CLOCKWISE
	else if (rx_command_buffer[0] == 'D' && rx_command_buffer[1] == 'C' && rx_command_buffer[2] == 'L')
	{
		COMMAND_DIRECTION_CLOCKWISE();
	}
	//COMMAND DIRECTION ANTI-CLOCKWISE
	else if (rx_command_buffer[0] == 'D' && rx_command_buffer[1] == 'A' && rx_command_buffer[2] == 'C')
	{
		COMMAND_DIRECTION_ANTI_CLOCKWISE();
	}
	//COMMAND MANUAL
	else if (rx_command_buffer[0] == 'M' && rx_command_buffer[1] == 'A' && rx_command_buffer[2] == 'N')
	{
		COMMAND_MANUAL(rx_command_buffer[4] == '0' ? STEP_DISABLE : STEP_ENABLE, rx_command_buffer[5] == '0' ? ANTICLOCKWISE : CLOCKWISE, rx_command_buffer[6] == '0' ? STEP_NOPULSE : STEP_PULSE);
	}
	//COMMAND CONFIRM SETTINGS
	else if (rx_command_buffer[0] == 'C' && rx_command_buffer[1] == 'F' && rx_command_buffer[2] == 'R')
	{
		COMMAND_CONFIRM_SETTINGS();
	}
	//COMMAND CONNECTION TEST
	else if (rx_command_buffer[0] == 'T' && rx_command_buffer[1] == 'S' && rx_command_buffer[2] == 'T')
	{
		COMMAND_CONNECTION_TEST();
	}
}

void UART_RECEIVE_COMMAND()
{
	for (uint8_t counter = 0; counter < __RECEIVE_BUFFER; counter++) //$ Arama.
	{
		if (rx_buffer[counter] == '$') //$ Bulundu.
			for (uint8_t counter2 = counter+1; counter2 < __RECEIVE_BUFFER; counter2++) //! Arama.
			{
				if (rx_buffer[counter2] == '!')
				{
					StrCopyInSet((counter+1), (counter2-1));
					CommandSearch();
					counter = counter2;
					break;
				}
			}
	}

	for (uint8_t counter3 = 0; counter3 < __RECEIVE_BUFFER; counter3++)
	{
		rx_buffer[counter3] = '\0';
	}
}
__weak void MOD_CHANGE_STANDART_MODE()
{
	stepCommand.Trigger = STEP_PASSIVE;
	stepCommand.Step_Mode = STANDART_MODE;
	Step_Set_Enable();
}
__weak void MOD_CHANGE_PULSE_TRIGGERED_STANDART_MODE()
{
	stepCommand.Step_Mode = PULSE_TRIGGERED_STANDART_MODE;
	Step_Set_Enable();
}
__weak void MOD_CHANGE_LIMITED_ANGLE_MODE()
{
	stepCommand.Trigger = STEP_PASSIVE;
	stepCommand.Step_Mode = LIMITED_ANGLE_MODE;
	Step_Set_Enable();
}
__weak void MOD_CHANGE_PULSE_TRIGGERED_LIMITED_ANGLE_MODE()
{
	stepCommand.Step_Mode = PULSE_TRIGGERED_LIMITED_ANGLE_MODE;
	Step_Set_Enable();
}
__weak void MOD_CHANGE_ONE_DIRECTION_PULSE_TRIGGERED_LIMITED_ANGLE_MODE()
{
	stepCommand.Step_Mode = ONE_DIRECTION_PULSE_TRIGGERED_LIMITED_ANGLE_MODE;
	Step_Set_Disable();
}
__weak void MOD_CHANGE_MANUEL_MODE()
{
	stepCommand.Step_Mode = MANUEL_MODE;
	Step_Set_Disable();
}
__weak void MOD_CHANGE_CALIBRATION_MODE()
{
	stepCommand.Step_Mode = CALIBRATION_MODE;
	Step_Set_Enable();
}
__weak void MOD_CHANGE_PASSIVE_MODE()
{
	stepCommand.Step_Mode = PASSIVE_MODE;
	Step_Set_Disable();
}

__weak void COMMAND_SPEED(int speed)
{
	stepCommand.delay = CalculateDelayToSpeed((uint16_t)speed);
}
__weak void COMMAND_DEGREE(int degree)
{
	stepCommand.degree = degree;
}
__weak void COMMAND_RUN()
{
	stepCommand.Trigger = STEP_ACTIVE;

	if(stepCommand.Step_Mode == MANUEL_MODE) Step_Set_Enable();
	if(stepCommand.Step_Mode == ONE_DIRECTION_PULSE_TRIGGERED_LIMITED_ANGLE_MODE)
	{
		Step_Set_Enable();
		CompleteTurn = 0;
		state.Step_Counter = 0;
	}
}
__weak void COMMAND_STOP()
{
	stepCommand.Trigger = STEP_PASSIVE;

	if(stepCommand.Step_Mode == MANUEL_MODE) Step_Set_Disable();
	if(stepCommand.Step_Mode == ONE_DIRECTION_PULSE_TRIGGERED_LIMITED_ANGLE_MODE)
	{
		Step_Set_Disable();
	}
}
__weak void COMMAND_DIRECTION_CLOCKWISE()
{
	stepCommand.Step_Direction = CLOCKWISE;
	ControlTurn = CLOCKWISE;
	Step_Set_Direction(CLOCKWISE);
}
__weak void COMMAND_DIRECTION_ANTI_CLOCKWISE()
{
	stepCommand.Step_Direction = ANTICLOCKWISE;
	ControlTurn = ANTICLOCKWISE;
	Step_Set_Direction(ANTICLOCKWISE);
}
__weak void COMMAND_MANUAL(Step_Enable enable, Step_Direction direction, Step_Pulse pulse)
{
	stepCommand.manuelCommand.Enable = enable;
	stepCommand.manuelCommand.Direction = direction;
	stepCommand.manuelCommand.Pulse = pulse;

	if (enable == STEP_ENABLE)
	{
		Step_Set_Enable();
		if (pulse == STEP_PULSE) Step_Send_Single_Pulse();
	}
	else Step_Set_Disable();

	Step_Set_Direction(direction);
}
__weak void COMMAND_CONFIRM_SETTINGS()
{
	switch(stepCommand.Step_Mode)
		{
		case PASSIVE_MODE:
			break;
		case CALIBRATION_MODE:
			break;
		case STANDART_MODE:
			stepCommand.Trigger = STEP_ACTIVE;
			break;
		case PULSE_TRIGGERED_STANDART_MODE:
			break;
		case LIMITED_ANGLE_MODE:
			stepCommand.Trigger = STEP_ACTIVE;
			state.Step_Counter = 0;
			break;
		case PULSE_TRIGGERED_LIMITED_ANGLE_MODE:
			state.Step_Counter = 0;
			CompleteTurn = 0;
			break;
		case ONE_DIRECTION_PULSE_TRIGGERED_LIMITED_ANGLE_MODE:
			state.Step_Counter = 0;
			break;
		case MANUEL_MODE:
			break;
		}
}
__weak void COMMAND_CONNECTION_TEST()
{
	HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
}
__weak void DEFAULT_SETTINGS()
{
	calibrationSettings.MAX_PULSE_HIGH_SIGNAL=50;
	calibrationSettings.MAX_PULSE_DELAY=1000;
	calibrationSettings.NUMBER_OF_SIGNALS_IN_FULL_TURN=200;
	calibrationSettings.NUMBER_OF_SIGNAL_IN_FULL_TURN_ENCODER=953; //ENCODERIN BİR TAM TURU 1200, STEPİN BİR TAM TURU 1400 ENCODER SİNYALİ

	stepCommand.Trigger = PASSIVE_MODE;
}
///STEP MOTOR CONTROLLER
///PLEASE PUT IT IN AN ENDLESS LOOP
void STEP_MOTOR_CONTROL(void)
{
	switch(stepCommand.Step_Mode)
		{
		//Step_State state = Step_Get_State();
		case PASSIVE_MODE:
			break;


		case CALIBRATION_MODE:
			break;


		case STANDART_MODE:
		case PULSE_TRIGGERED_STANDART_MODE:
			if (stepCommand.Trigger == STEP_ACTIVE) Step_Send_Single_Pulse_With_DelayConfig(calibrationSettings.MAX_PULSE_HIGH_SIGNAL+5, calibrationSettings.MAX_PULSE_DELAY+stepCommand.delay);
			break;


		case LIMITED_ANGLE_MODE:
			if (stepCommand.Step_Direction == CLOCKWISE)
			{
				if ((state.Step_Counter == CalculateDegreeToPulse(stepCommand.degree)) && ControlTurn == CLOCKWISE)
				{
					ControlTurn = ANTICLOCKWISE;
					Step_Set_Direction(ANTICLOCKWISE);
				}
				else if ((state.Step_Counter == 0) && ControlTurn == ANTICLOCKWISE)
				{
					ControlTurn = CLOCKWISE;
					Step_Set_Direction(CLOCKWISE);
				}
			}
			else
			{
				if ((state.Step_Counter == CalculateDegreeToPulse(stepCommand.degree)*-1) && ControlTurn == ANTICLOCKWISE)
				{
					ControlTurn = CLOCKWISE;
					Step_Set_Direction(CLOCKWISE);
				}
				else if ((state.Step_Counter == 0) && ControlTurn == CLOCKWISE)
				{
					ControlTurn = ANTICLOCKWISE;
					Step_Set_Direction(ANTICLOCKWISE);
				}
			}
			if (stepCommand.Trigger == STEP_ACTIVE) Step_Send_Single_Pulse_With_DelayConfig(calibrationSettings.MAX_PULSE_HIGH_SIGNAL+5, calibrationSettings.MAX_PULSE_DELAY+stepCommand.delay);
			break;

		case PULSE_TRIGGERED_LIMITED_ANGLE_MODE:
			if (ControlTurn == CLOCKWISE)
			{
				if (stepCommand.Trigger == STEP_ACTIVE)
				{
					if (CompleteTurn == 0)
					{
						if (state.Step_Counter < CalculateDegreeToPulse(stepCommand.degree))
						{
							Step_Set_Direction(CLOCKWISE);
							Step_Send_Single_Pulse_With_DelayConfig(calibrationSettings.MAX_PULSE_HIGH_SIGNAL+5, calibrationSettings.MAX_PULSE_DELAY+stepCommand.delay);
						}
						else if(state.Step_Counter == CalculateDegreeToPulse(stepCommand.degree))
						{
							CompleteTurn = 1;
							Step_Set_Direction(ANTICLOCKWISE);
						}
					}
					else //CompeleteTurn == 1
					{
						CompleteTurn = 0;
					}
				}
				else //STEP PASSİVE
				{
					if(state.Step_Counter == 0)
					{
						CompleteTurn = 0;
						Step_Set_Direction(CLOCKWISE);
					}
					else if (state.Step_Counter <= CalculateDegreeToPulse(stepCommand.degree))
					{
						Step_Set_Direction(ANTICLOCKWISE);
						Step_Send_Single_Pulse_With_DelayConfig(calibrationSettings.MAX_PULSE_HIGH_SIGNAL+5, calibrationSettings.MAX_PULSE_DELAY+stepCommand.delay);
					}
				}
			}

			else //ControlTurn == ANTICLOCKWISE
			{
				if (stepCommand.Trigger == STEP_ACTIVE)
				{
					if (CompleteTurn == 0)
					{
						if(state.Step_Counter == CalculateDegreeToPulse(stepCommand.degree)*-1)
						{
							CompleteTurn = 1;
							Step_Set_Direction(CLOCKWISE);
						}
						else
						{
							Step_Set_Direction(ANTICLOCKWISE);
							Step_Send_Single_Pulse_With_DelayConfig(calibrationSettings.MAX_PULSE_HIGH_SIGNAL+5, calibrationSettings.MAX_PULSE_DELAY+stepCommand.delay);
						}

					}
					else //CompeleteTurn == 1
					{
						CompleteTurn = 0;
					}
				}
				else //STEP PASSİVE
				{
					if(state.Step_Counter == 0)
					{
						CompleteTurn = 0;
						Step_Set_Direction(ANTICLOCKWISE);
					}
					else if (state.Step_Counter >= CalculateDegreeToPulse(stepCommand.degree)*-1)
					{
						Step_Set_Direction(CLOCKWISE);
						Step_Send_Single_Pulse_With_DelayConfig(calibrationSettings.MAX_PULSE_HIGH_SIGNAL+5, calibrationSettings.MAX_PULSE_DELAY+stepCommand.delay);
					}
				}
			}
			break;


		case ONE_DIRECTION_PULSE_TRIGGERED_LIMITED_ANGLE_MODE:

			if (CompleteTurn == 0) //HEDEFE ULAŞILMADI İSE
			{
				if (ControlTurn == CLOCKWISE)
				{
					if(state.Step_Counter == CalculateDegreeToPulse(stepCommand.degree))
					{
						CompleteTurn = 1;
					}
					else
					{
						Step_Send_Single_Pulse_With_DelayConfig(calibrationSettings.MAX_PULSE_HIGH_SIGNAL+5, calibrationSettings.MAX_PULSE_DELAY+stepCommand.delay);
					}
				}
				else //ANTICLOCKWISE
				{
					if(state.Step_Counter == CalculateDegreeToPulse(stepCommand.degree)*-1)
					{
						CompleteTurn = 1;
					}
					else
					{
						Step_Send_Single_Pulse_With_DelayConfig(calibrationSettings.MAX_PULSE_HIGH_SIGNAL+5, calibrationSettings.MAX_PULSE_DELAY+stepCommand.delay);
					}
				}
			}
			break;

		case MANUEL_MODE:
			break;


		}
}
uint32_t CalculateDelayToSpeed(uint8_t speed)
{
	double delay = (60.0/(double)speed)*1000;
	delay -= calibrationSettings.MAX_PULSE_DELAY;
	return (uint16_t)delay;
}

uint32_t CalculateDegreeToPulse(uint16_t degree)
{
	uint32_t a = (uint32_t)((calibrationSettings.NUMBER_OF_SIGNALS_IN_FULL_TURN/360.0)*(double)degree);
	return a;
}

/* END PFP */

/* Private Code ---------------------------------------------------------*/

/* CODE END*/

/************************ (C) COPYRIGHT Muhammed Sadun Çakmaklı *****END OF FILE****/



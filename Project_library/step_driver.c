/* Header */
/**
  ******************************************************************************
  * @file    step_driver.c
  * @brief   Step motorun çalışmasını sağlayan kütüphane.
  * @author  Muhammed Sadun Çakmaklı
  * @date    03-01-2021
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
#include "step_driver.h"

/* END Includes */

/* Private typedef -----------------------------------------------------------*/

/* END PTD */

/* Private define ------------------------------------------------------------*/
/* END PD */

/* Private macro -------------------------------------------------------------*/

/* END PM */

/* Private variables ---------------------------------------------------------*/
volatile Step_State state;
/* END PV */

/* Private function prototypes -----------------------------------------------*/

//void Step_Pulse(int Number_of_Pulse, int )
//{
//	HAL_GPIO_WritePin(STEP_PULSE_GPIO_Port, STEP_PULSE_Pin, GPIO_PIN_SET);
//	HAL_Delay(10);
//	HAL_GPIO_WritePin(STEP_PULSE_GPIO_Port, STEP_PULSE_Pin, GPIO_PIN_RESET);
//}

void Step_Send_Single_Pulse(void)
{
	Step_Send_Single_Pulse_With_DelayConfig(50,950);
}

void Step_Send_Single_Pulse_With_DelayConfig(uint16_t delay_us_HIGH, uint16_t delay_us_FULL)
{
	if (state.Step_Power == STEP_PASSIVE) Step_Set_Enable();
	HAL_GPIO_WritePin(STEP_PULSE_GPIO_Port, STEP_PULSE_Pin, GPIO_PIN_SET);
	delay_us(delay_us_HIGH);

	if (state.Step_Direction == CLOCKWISE) state.Step_Counter++;
	else state.Step_Counter--;

	HAL_GPIO_WritePin(STEP_PULSE_GPIO_Port, STEP_PULSE_Pin, GPIO_PIN_RESET);
	delay_us(delay_us_FULL-delay_us_HIGH);
}

void Step_Set_Direction(Step_Direction direction)
{
	if (direction == CLOCKWISE)
	{
		HAL_GPIO_WritePin(STEP_DIRECTION_GPIO_Port, STEP_DIRECTION_Pin, GPIO_PIN_RESET);
		state.Step_Direction = CLOCKWISE;
	}
	else
	{
		HAL_GPIO_WritePin(STEP_DIRECTION_GPIO_Port, STEP_DIRECTION_Pin, GPIO_PIN_SET);
		state.Step_Direction = ANTICLOCKWISE;
	}
}
void Step_Set_Enable(void)
{
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(STEP_ENABLE_GPIO_Port, STEP_ENABLE_Pin, GPIO_PIN_RESET);
	state.Step_Power = STEP_ACTIVE;
}
void Step_Set_Disable(void)
{
	HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(STEP_ENABLE_GPIO_Port, STEP_ENABLE_Pin, GPIO_PIN_SET);
	state.Step_Power = STEP_PASSIVE;
}
Step_State Step_Get_State(void)
{
	return state;
}

/* END PFP */

/* Private Code ---------------------------------------------------------*/

/* CODE END*/

/************************ (C) COPYRIGHT Muhammed Sadun Çakmaklı *****END OF FILE****/

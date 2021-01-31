/* Header */
/**
  ******************************************************************************
  * @file           : encoder_driver.c
  * @brief          : Kodlayıcının algılamasını sağlayan interrupt ve diğer fonksiyonları barındırır.
  * @author			: Muhammed Sadun ÇAKMAKLI
  * @date			: 02.01.2021
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
#include "encoder_driver.h"

/* END Includes */

/* Private typedef -----------------------------------------------------------*/

/* END PTD */

/* Private define ------------------------------------------------------------*/
/* END PD */

/* Private macro -------------------------------------------------------------*/

/* END PM */

/* Private variables ---------------------------------------------------------*/
volatile int32_t encoder_step = 0;
extern UART_HandleTypeDef huart3;
/* END PV */

/* Private function prototypes -----------------------------------------------*/

/* END PFP */

/* Private Code ---------------------------------------------------------*/
extern void ENCODER_A_INTERRUPT(void)
{
	if ((ENCODER_A_GPIO_Port->IDR & ENCODER_A_Pin) == 0)
	{
		//ENCODER A RISING STATE
		if ((ENCODER_B_GPIO_Port->IDR & ENCODER_B_Pin) == 0)
			{
				//ENCODER B HIGH
				//ANTICLOCKWISE
				encoder_step--;


			}
			else
			{
				//ENCODER B LOW
				//CLOCKWISE
				encoder_step++;
			}
	}
	else
	{
		//ENCODER A FALLİNG STATE
		if ((ENCODER_B_GPIO_Port->IDR & ENCODER_B_Pin) == 0)
			{
				//ENCODER B HIGH
				//CLOCKWISE
				encoder_step++;
			}
			else
			{
				//ENCODER B LOW
				//ANTICLOCKWISE
				encoder_step--;
			}
	}
//	if (encoder_step%20 == 0)
//	{
//		//DEBUG İÇİN
//			char buffer[6];
//			for (int x = 0; x < 6; x++)
//			{
//				buffer[x]='\0';
//			}
//			itoa(encoder_step,buffer,10);
//
//		//DEBUG SWV DATA CONSOLE
//		//	printf(buffer);
//		//	printf("\n");
//
//		//DEBUG UART
//		HAL_UART_Transmit(&huart3, &buffer, 6, 100);
//		HAL_UART_Transmit(&huart3, "\n", 1, 100);
//	}
}

extern void ENCODER_B_INTERRUPT(void)
{
	if ((ENCODER_B_GPIO_Port->IDR & ENCODER_B_Pin) == 0)
	{
		//ENCODER B RISING STATE
		if ((ENCODER_A_GPIO_Port->IDR & ENCODER_A_Pin) == 0)
			{
				//ENCODER A HIGH
				//CLOCKWISE
				encoder_step++;
			}
			else
			{
				//ENCODER A LOW
				//ANTICLOCKWISE
				encoder_step--;
			}
	}
	else
	{
		//ENCODER B FALLİNG STATE
		if ((ENCODER_A_GPIO_Port->IDR & ENCODER_A_Pin) == 0)
			{
				//ENCODER A HIGH
				//ANTICLOCKWISE
				encoder_step--;
			}
			else
			{
				//ENCODER A LOW
				//CLOCKWISE
				encoder_step++;
			}
	}
//	if (encoder_step%20 == 0)
//	{
//		//DEBUG İÇİN
//			char buffer[6];
//			for (int x = 0; x < 6; x++)
//			{
//				buffer[x]='\0';
//			}
//			itoa(encoder_step,buffer,10);
//
//		//DEBUG SWV DATA CONSOLE
//		//	printf(buffer);
//		//	printf("\n");
//
//		//DEBUG UART
//		HAL_UART_Transmit(&huart3, &buffer, 6, 100);
//		HAL_UART_Transmit(&huart3, "\n", 1, 100);
//	}
}
/* CODE END*/

/************************ (C) COPYRIGHT Muhammed Sadun Çakmaklı *****END OF FILE****/

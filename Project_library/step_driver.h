/* Header */
/**
  ******************************************************************************
  * @file    step_driver.h
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STEP_DRIVER
#define __STEP_DRIVER

/* Private includes ----------------------------------------------------------*/

/* END Includes */

/* Exported types ------------------------------------------------------------*/
typedef enum
{
	CLOCKWISE = 0U,
	ANTICLOCKWISE
}Step_Direction;

typedef enum
{
	STEP_ACTIVE,
	STEP_PASSIVE
}Step_Power;

typedef struct
{
	int32_t Step_Counter;
	Step_Power Step_Power;
	Step_Direction Step_Direction;
}Step_State;
/* END ET */

/* Exported constants --------------------------------------------------------*/

/* END EC */

/* Exported macro ------------------------------------------------------------*/

/* END EM */

/* Exported functions prototypes ---------------------------------------------*/
volatile Step_State state;
void Step_Set_Enable(void);
void Step_Set_Disable(void);
void Step_Set_Direction(Step_Direction);
void Step_Send_Single_Pulse(void);
void Step_Send_Single_Pulse_With_DelayConfig(uint16_t delay_us_HIGH, uint16_t delay_us_FULL);
Step_State Step_Get_State(void);
/* END EFP */


#endif /* __STEP_DRIVER */

/************************ (C) COPYRIGHT Muhammed Sadun Çakmaklı *****END OF FILE****/

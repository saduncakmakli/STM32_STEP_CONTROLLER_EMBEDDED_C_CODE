/* Header */
/**
  ******************************************************************************
  * @file    encoder_driver.h
  * @brief   Kodlayıcının algılamasını sağlayan interrupt ve diğer fonksiyonları barındırır.
  * @author  Muhammed Sadun Çakmaklı
  * @date    02-01-2021
  ******************************************************************************
  * @attention
  *
  *
 ******************************************************************************
  */
/* END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __ENCODER_DRIVER
#define __ENCODER_DRIVER

/* Private includes ----------------------------------------------------------*/

/* END Includes */

/* Exported types ------------------------------------------------------------*/

/* END ET */

/* Exported constants --------------------------------------------------------*/

/* END EC */

/* Exported macro ------------------------------------------------------------*/

/* END EM */

/* Exported functions prototypes ---------------------------------------------*/
void ENCODER_A_INTERRUPT(void);
void ENCODER_B_INTERRUPT(void);
/* END EFP */


#endif /* __ENCODER_DRIVER */

/************************ (C) COPYRIGHT Muhammed Sadun Çakmaklı *****END OF FILE****/

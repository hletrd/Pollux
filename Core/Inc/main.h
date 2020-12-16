/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

#define ser_bufsize 100
extern uint8_t ser_buf[ser_bufsize];
extern int ser_pos;
extern int32_t ser_last;
extern int GUIDE_STATE[4];
extern float vin, iin;

#define melody_buf_len 100
extern float melody_queue[melody_buf_len];
extern int melody_len_queue[melody_buf_len];
extern int melody_play_cnt;
extern int melody_play_pos;
extern int melody_counter;

extern TIM_HandleTypeDef htim2;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

extern void guide_callback();
extern void serial_process();
extern void tim_ra_callback();
extern void tim_dec_callback();
extern void tim_acc_callback();
extern void debug();

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define ESP_PROG_Pin GPIO_PIN_13
#define ESP_PROG_GPIO_Port GPIOC
#define GUIDE1_Pin GPIO_PIN_0
#define GUIDE1_GPIO_Port GPIOC
#define GUIDE1_EXTI_IRQn EXTI0_IRQn
#define GUIDE2_Pin GPIO_PIN_1
#define GUIDE2_GPIO_Port GPIOC
#define GUIDE2_EXTI_IRQn EXTI1_IRQn
#define GUIDE3_Pin GPIO_PIN_2
#define GUIDE3_GPIO_Port GPIOC
#define GUIDE3_EXTI_IRQn EXTI2_IRQn
#define GUIDE4_Pin GPIO_PIN_3
#define GUIDE4_GPIO_Port GPIOC
#define GUIDE4_EXTI_IRQn EXTI3_IRQn
#define MUART2_TX_Pin GPIO_PIN_0
#define MUART2_TX_GPIO_Port GPIOA
#define MUART2_Pin GPIO_PIN_1
#define MUART2_GPIO_Port GPIOA
#define ACC2_TX_Pin GPIO_PIN_2
#define ACC2_TX_GPIO_Port GPIOA
#define ACC2_RX_Pin GPIO_PIN_3
#define ACC2_RX_GPIO_Port GPIOA
#define STEP2_Pin GPIO_PIN_4
#define STEP2_GPIO_Port GPIOA
#define DIR2_Pin GPIO_PIN_5
#define DIR2_GPIO_Port GPIOA
#define STDBY2_Pin GPIO_PIN_6
#define STDBY2_GPIO_Port GPIOA
#define INDEX2_Pin GPIO_PIN_7
#define INDEX2_GPIO_Port GPIOA
#define DIAG2_Pin GPIO_PIN_4
#define DIAG2_GPIO_Port GPIOC
#define NENBL2_Pin GPIO_PIN_5
#define NENBL2_GPIO_Port GPIOC
#define ISEN_Pin GPIO_PIN_0
#define ISEN_GPIO_Port GPIOB
#define NENBL1_Pin GPIO_PIN_2
#define NENBL1_GPIO_Port GPIOB
#define ACC1_TX_Pin GPIO_PIN_10
#define ACC1_TX_GPIO_Port GPIOB
#define ACC1_RX_Pin GPIO_PIN_11
#define ACC1_RX_GPIO_Port GPIOB
#define LED_Y_Pin GPIO_PIN_12
#define LED_Y_GPIO_Port GPIOB
#define LED_B_Pin GPIO_PIN_13
#define LED_B_GPIO_Port GPIOB
#define LED_W_Pin GPIO_PIN_14
#define LED_W_GPIO_Port GPIOB
#define LED_G_Pin GPIO_PIN_15
#define LED_G_GPIO_Port GPIOB
#define MUART1_TX_Pin GPIO_PIN_6
#define MUART1_TX_GPIO_Port GPIOC
#define MUART1_Pin GPIO_PIN_7
#define MUART1_GPIO_Port GPIOC
#define STEP1_Pin GPIO_PIN_8
#define STEP1_GPIO_Port GPIOC
#define DIR1_Pin GPIO_PIN_9
#define DIR1_GPIO_Port GPIOC
#define STDBY1_Pin GPIO_PIN_8
#define STDBY1_GPIO_Port GPIOA
#define UART_TX_Pin GPIO_PIN_9
#define UART_TX_GPIO_Port GPIOA
#define UART_RX_Pin GPIO_PIN_10
#define UART_RX_GPIO_Port GPIOA
#define INDEX1_Pin GPIO_PIN_11
#define INDEX1_GPIO_Port GPIOA
#define DIAG1_Pin GPIO_PIN_12
#define DIAG1_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define BUZZER_Pin GPIO_PIN_15
#define BUZZER_GPIO_Port GPIOA
#define HC_TX_Pin GPIO_PIN_12
#define HC_TX_GPIO_Port GPIOC
#define HC_RX_Pin GPIO_PIN_2
#define HC_RX_GPIO_Port GPIOD
#define SCL_DP_Pin GPIO_PIN_6
#define SCL_DP_GPIO_Port GPIOB
#define SDA_DP_Pin GPIO_PIN_7
#define SDA_DP_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

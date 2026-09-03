/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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

#include "stm32f4xx_ll_dma.h"
#include "stm32f4xx_ll_i2c.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_spi.h"
#include "stm32f4xx_ll_gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define mpu6500_irq_pin_Pin LL_GPIO_PIN_7
#define mpu6500_irq_pin_GPIO_Port GPIOC
#define mpu6500_irq_pin_EXTI_IRQn EXTI9_5_IRQn
#define TFT1_DC_Pin LL_GPIO_PIN_10
#define TFT1_DC_GPIO_Port GPIOE
#define TFT1_RST_Pin LL_GPIO_PIN_11
#define TFT1_RST_GPIO_Port GPIOE
#define TFT1_CS_Pin LL_GPIO_PIN_12
#define TFT1_CS_GPIO_Port GPIOE
#define TFT2_DC_Pin LL_GPIO_PIN_13
#define TFT2_DC_GPIO_Port GPIOE
#define TFT2_RST_Pin LL_GPIO_PIN_14
#define TFT2_RST_GPIO_Port GPIOE
#define TFT2_CS_Pin LL_GPIO_PIN_15
#define TFT2_CS_GPIO_Port GPIOE
#define SSD1306_SPI_RST_Pin LL_GPIO_PIN_6
#define SSD1306_SPI_RST_GPIO_Port GPIOC
#define SSD1306_SPI_CS_Pin LL_GPIO_PIN_8
#define SSD1306_SPI_CS_GPIO_Port GPIOC
#define SSD1306_SPI_DC_Pin LL_GPIO_PIN_9
#define SSD1306_SPI_DC_GPIO_Port GPIOC
#define nrf24_ce_pin_spi1_Pin LL_GPIO_PIN_0
#define nrf24_ce_pin_spi1_GPIO_Port GPIOD
#define nrf24_csn_pin_spi1_Pin LL_GPIO_PIN_1
#define nrf24_csn_pin_spi1_GPIO_Port GPIOD
#define nrf24_irq_pin_spi1_Pin LL_GPIO_PIN_2
#define nrf24_irq_pin_spi1_GPIO_Port GPIOD
#define nrf24_irq_pin_spi1_EXTI_IRQn EXTI2_IRQn
#define rc522_irq_pin_Pin LL_GPIO_PIN_3
#define rc522_irq_pin_GPIO_Port GPIOD
#define rc522_irq_pin_EXTI_IRQn EXTI3_IRQn
#define rc522_cs_pin_Pin LL_GPIO_PIN_6
#define rc522_cs_pin_GPIO_Port GPIOD
#define rc522_rst_pin_Pin LL_GPIO_PIN_7
#define rc522_rst_pin_GPIO_Port GPIOD
#define CAN_LINK_LED_Pin LL_GPIO_PIN_12
#define CAN_LINK_LED_GPIO_Port GPIOD
#define NRF_LINK_LED_Pin LL_GPIO_PIN_13
#define NRF_LINK_LED_GPIO_Port GPIOD

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

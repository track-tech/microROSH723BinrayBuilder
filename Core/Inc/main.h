/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32h7xx_hal.h"

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
#define PF6_SPI5_MCP2517FD4_CS_Pin GPIO_PIN_6
#define PF6_SPI5_MCP2517FD4_CS_GPIO_Port GPIOF
#define PF7_SPI5_MCP2517FD4_SCK_Pin GPIO_PIN_7
#define PF7_SPI5_MCP2517FD4_SCK_GPIO_Port GPIOF
#define PF8_SPI5_MCP2517FD4_MISO_Pin GPIO_PIN_8
#define PF8_SPI5_MCP2517FD4_MISO_GPIO_Port GPIOF
#define PF9_SPI5_MCP2517FD4_MOSI_Pin GPIO_PIN_9
#define PF9_SPI5_MCP2517FD4_MOSI_GPIO_Port GPIOF
#define PF10_SPI5_MCP2517FD4_INT_Pin GPIO_PIN_10
#define PF10_SPI5_MCP2517FD4_INT_GPIO_Port GPIOF
#define PA4_SPI6_W5100S_CS_Pin GPIO_PIN_4
#define PA4_SPI6_W5100S_CS_GPIO_Port GPIOA
#define PA5_SPI6_W5100S_SCK_Pin GPIO_PIN_5
#define PA5_SPI6_W5100S_SCK_GPIO_Port GPIOA
#define PA6_SPI6_W5100S_MISO_Pin GPIO_PIN_6
#define PA6_SPI6_W5100S_MISO_GPIO_Port GPIOA
#define PA6_SPI6_W5100S_MOSI_Pin GPIO_PIN_7
#define PA6_SPI6_W5100S_MOSI_GPIO_Port GPIOA
#define PC4_SPI6_W5100S_INT_Pin GPIO_PIN_4
#define PC4_SPI6_W5100S_INT_GPIO_Port GPIOC
#define PC4_SPI6_W5100S_INT_EXTI_IRQn EXTI4_IRQn
#define PC5_SPI6_W5100S_RST_Pin GPIO_PIN_5
#define PC5_SPI6_W5100S_RST_GPIO_Port GPIOC
#define PE_SPI4_MCP2517FD2_CS_Pin GPIO_PIN_11
#define PE_SPI4_MCP2517FD2_CS_GPIO_Port GPIOE
#define PE13_SPI4_MCP2517FD2_SCK_Pin GPIO_PIN_12
#define PE13_SPI4_MCP2517FD2_SCK_GPIO_Port GPIOE
#define PE13_SPI4_MCP2517FD2_MISO_Pin GPIO_PIN_13
#define PE13_SPI4_MCP2517FD2_MISO_GPIO_Port GPIOE
#define PE14_SPI4_MCP2517FD2_MOSI_Pin GPIO_PIN_14
#define PE14_SPI4_MCP2517FD2_MOSI_GPIO_Port GPIOE
#define PE15_SPI4_MCP2517FD2_INT_Pin GPIO_PIN_15
#define PE15_SPI4_MCP2517FD2_INT_GPIO_Port GPIOE
#define PB12_SPI2_MCP2517FD2_CS_Pin GPIO_PIN_12
#define PB12_SPI2_MCP2517FD2_CS_GPIO_Port GPIOB
#define PB13_SPI2_MCP2517FD2_SCK_Pin GPIO_PIN_13
#define PB13_SPI2_MCP2517FD2_SCK_GPIO_Port GPIOB
#define PB14_SPI2_MCP2517FD2_MISO_Pin GPIO_PIN_14
#define PB14_SPI2_MCP2517FD2_MISO_GPIO_Port GPIOB
#define PB15_SPI2_MCP2517FD2_MOSI_Pin GPIO_PIN_15
#define PB15_SPI2_MCP2517FD2_MOSI_GPIO_Port GPIOB
#define PD8_SPI2_MCP2517FD2_INT_Pin GPIO_PIN_8
#define PD8_SPI2_MCP2517FD2_INT_GPIO_Port GPIOD
#define PA10_R8SM_UART1_RX_SBUS_Pin GPIO_PIN_10
#define PA10_R8SM_UART1_RX_SBUS_GPIO_Port GPIOA
#define PA15_SPI3_MCP2517FD2_CS_Pin GPIO_PIN_15
#define PA15_SPI3_MCP2517FD2_CS_GPIO_Port GPIOA
#define PC10_SPI3_MCP2517FD2_SCK_Pin GPIO_PIN_10
#define PC10_SPI3_MCP2517FD2_SCK_GPIO_Port GPIOC
#define PC11_SPI3_MCP2517FD2_MISO_Pin GPIO_PIN_11
#define PC11_SPI3_MCP2517FD2_MISO_GPIO_Port GPIOC
#define PC12_SPI3_MCP2517FD2_MOSI_Pin GPIO_PIN_12
#define PC12_SPI3_MCP2517FD2_MOSI_GPIO_Port GPIOC
#define PD0_EXTI0_MCP2517FD2_INT_Pin GPIO_PIN_0
#define PD0_EXTI0_MCP2517FD2_INT_GPIO_Port GPIOD
#define PD5_USART2_TX_Pin GPIO_PIN_5
#define PD5_USART2_TX_GPIO_Port GPIOD
#define PD6_USART2_RX_Pin GPIO_PIN_6
#define PD6_USART2_RX_GPIO_Port GPIOD
#define PG11_USART10_RX_Pin GPIO_PIN_11
#define PG11_USART10_RX_GPIO_Port GPIOG
#define PG12_USART10_TX_Pin GPIO_PIN_12
#define PG12_USART10_TX_GPIO_Port GPIOG

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

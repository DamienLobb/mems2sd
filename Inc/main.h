/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

/* Includes ------------------------------------------------------------------*/
#include "stdint.h"
#include "wifi_conf.h"

/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

typedef enum {
  wifi_state_reset = 0,
  wifi_state_ready,
  wifi_state_idle,
  wifi_state_connected,
  wifi_state_connecting,
  wifi_state_socket,
  wifi_state_socket_write,
  wifi_state_disconnected,
  wifi_state_activity,
  wifi_state_inter,
  wifi_state_print_data,
  wifi_state_error,
  wifi_undefine_state       = 0xFF,
} wifi_state_t;

/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */

/* Private macro ------------------------------------------------------------ */
#ifdef USART_PRINT_MSG

#ifdef USE_STM32L0XX_NUCLEO

#define WIFI_UART_MSG                           USART2
#define USARTx_PRINT_CLK_ENABLE()              __USART2_CLK_ENABLE()
#define USARTx_PRINT_RX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()
#define USARTx_PRINT_TX_GPIO_CLK_ENABLE()      __GPIOA_CLK_ENABLE()

#define USARTx_PRINT_FORCE_RESET()             __USART2_FORCE_RESET()
#define USARTx_PRINT_RELEASE_RESET()           __USART2_RELEASE_RESET()

#define PRINTMSG_USARTx_TX_AF                       GPIO_AF4_USART2
#define PRINTMSG_USARTx_RX_AF                       GPIO_AF4_USART2

#endif //USE_STM32L0XX_NUCLEO

#if defined USE_STM32F4XX_NUCLEO

#define WIFI_UART_MSG                           USART2
#define USARTx_PRINT_CLK_ENABLE()              __HAL_RCC_USART2_CLK_ENABLE()
#define USARTx_PRINT_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_PRINT_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define USARTx_PRINT_FORCE_RESET()             __HAL_RCC_USART2_FORCE_RESET()
#define USARTx_PRINT_RELEASE_RESET()           __HAL_RCC_USART2_RELEASE_RESET()

#define PRINTMSG_USARTx_TX_AF                       GPIO_AF7_USART2
#define PRINTMSG_USARTx_RX_AF                       GPIO_AF7_USART2

#endif //(USE_STM32F1xx_NUCLEO) || (USE_STM32F4XX_NUCLEO)

#define WiFi_USART_PRINT_TX_PIN                    GPIO_PIN_2
#define WiFi_USART_PRINT_TX_GPIO_PORT              GPIOA
#define WiFi_USART_PRINT_RX_PIN                    GPIO_PIN_3
#define WiFi_USART_PRINT_RX_GPIO_PORT              GPIOA


/* Definition for USARTx's NVIC */
#define USARTx_PRINT_IRQn                      USART2_IRQn
#define USARTx_PRINT_IRQHandler                USART2_IRQHandler

#endif //USART_PRINT_MSG


#endif /* __MAIN_H */

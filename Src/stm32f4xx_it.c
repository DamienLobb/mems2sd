/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "wifi_module.h"
#include "stm32_spwf_wifi.h"
#include "wifi_globals.h"
#include "x_nucleo_cca02m1_audio_f4.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */

  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */

  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles Memory management fault.
*/
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
  /* USER CODE BEGIN MemoryManagement_IRQn 1 */

  /* USER CODE END MemoryManagement_IRQn 1 */
}

/**
* @brief This function handles Pre-fetch fault, memory access fault.
*/
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
  /* USER CODE BEGIN BusFault_IRQn 1 */

  /* USER CODE END BusFault_IRQn 1 */
}

/**
* @brief This function handles Undefined instruction or illegal state.
*/
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
  /* USER CODE BEGIN UsageFault_IRQn 1 */

  /* USER CODE END UsageFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
* @brief This function handles Debug monitor.
*/
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

void TIMx_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TimHandle);

}

/**
  * @brief  This function handles TIM interrupt request.
  * @param  None
  * @retval None
  */
void TIMp_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&PushTimHandle);

}


/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

void AUDIO_IN_I2S_IRQHandler(void)
{
  HAL_DMA_IRQHandler(hAudioInI2s.hdmarx);
}

//void USARTx_EXTI_IRQHandler(void)
//{
//  HAL_GPIO_EXTI_IRQHandler(WiFi_USART_RX_PIN);
//}

/**
  * @brief  This function GPIO EXTI Callback.
  * @param  Pin number of the GPIO generating the EXTI IRQ
  * @retval None
  */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  //RX_EXTI_Isr(GPIO_Pin);
}

/**
* @brief  Period elapsed callback in non blocking mode
*         This timer is used for calling back User registered functions with information
* @param  htim : TIM handle
* @retval None
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
#ifndef WIFI_USE_VCOM
  Wifi_TIM_Handler(htim);
#endif
}

/**
* @brief  HAL_UART_RxCpltCallback
*         Rx Transfer completed callback
* @param  UsartHandle: UART handle
* @retval None
*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandleArg)
{
#ifndef WIFI_USE_VCOM
  WiFi_HAL_UART_RxCpltCallback(UartHandleArg);
#endif
}

/**
* @brief  HAL_UART_TxCpltCallback
*         Tx Transfer completed callback
* @param  UsartHandle: UART handle
* @retval None
*/
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandleArg)
{
#ifndef WIFI_USE_VCOM
  WiFi_HAL_UART_TxCpltCallback(UartHandleArg);
#endif
}

/**
  * @brief  UART error callbacks
  * @param  UsartHandle: UART handle
  * @note   This example shows a simple way to report transfer error, and you can
  *         add your own implementation.
  * @retval None
  */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle)
{
  WiFi_HAL_UART_ErrorCallback(UartHandle);
}

/******************************************************************************/
/*                 STM32 Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32xxx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles USARTx Handler.
  * @param  None
  * @retval None
  */
void USARTx_IRQHandler(void)
{
  HAL_UART_IRQHandler(&UartWiFiHandle);
}

/**
  * @brief  This function handles USARTx vcom Handler.
  * @param  None
  * @retval None
  */
#ifdef USART_PRINT_MSG
void USARTx_PRINT_IRQHandler(void)
{
   HAL_UART_IRQHandler(UartMsgHandle);
}
#endif

#ifdef WIFI_USE_VCOM

#ifdef USE_STM32F4XX_NUCLEO
void DMA1_Stream5_IRQHandler(void)
{
   if(LL_DMA_IsActiveFlag_TC5(DMA1) == 1)
  {
    LL_DMA_ClearFlag_TC5(DMA1);
    DMA1_TransferComplete();
  }
}

void DMA2_Stream2_IRQHandler(void)
{
   if(LL_DMA_IsActiveFlag_TC2(DMA2) == 1)
  {
    LL_DMA_ClearFlag_TC2(DMA2);
    DMA2_TransferComplete();
  }
}
#endif //#ifdef USE_STM32F4XX_NUCLEO

#ifdef USE_STM32L4XX_NUCLEO
void DMA1_Channel6_IRQHandler(void)
{
   if(LL_DMA_IsActiveFlag_TC6(DMA1) == 1)
  {
    LL_DMA_ClearFlag_TC6(DMA1);
    DMA1_TransferComplete();
  }
}

void DMA1_Channel5_IRQHandler(void)
{
   if(LL_DMA_IsActiveFlag_TC5(DMA1) == 1)
  {
    LL_DMA_ClearFlag_TC5(DMA1);
    DMA2_TransferComplete();
  }
}
#endif //USE_STM32L4XX_NUCLEO


#ifdef USE_STM32L0XX_NUCLEO

void DMA_CONSOLE_IRQHANDLER(void)
{
  if(LL_DMA_IsActiveFlag_TC5(DMA1) == 1)
  {
    LL_DMA_ClearFlag_TC5(DMA1);
    DMA1_TransferComplete();
  }
}

void DMA_WIFI_IRQHANDLER(void)
{
  if(LL_DMA_IsActiveFlag_TC3(DMA1) == 1)
  {
    LL_DMA_ClearFlag_TC3(DMA1);
    DMA2_TransferComplete();
  }
}

#endif  //USE_STM32L0XX_NUCLEO


#ifdef USE_STM32F1xx_NUCLEO
static uint32_t DMA_IsActiveFlag_TC6(DMA_TypeDef *DMAx);
static uint32_t DMA_IsActiveFlag_TC5(DMA_TypeDef *DMAx);
static void DMA_ClearFlag_TC6(DMA_TypeDef *DMAx);
static void DMA_ClearFlag_TC5(DMA_TypeDef *DMAx);

static uint32_t DMA_IsActiveFlag_TC6(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->ISR ,DMA_FLAG_TC6)==(DMA_FLAG_TC6));
}

static uint32_t DMA_IsActiveFlag_TC5(DMA_TypeDef *DMAx)
{
  return (READ_BIT(DMAx->ISR ,DMA_FLAG_TC5)==(DMA_FLAG_TC5));
}

static void DMA_ClearFlag_TC6(DMA_TypeDef *DMAx)
{
  SET_BIT(DMAx->IFCR , DMA_FLAG_TC6);
}

static void DMA_ClearFlag_TC5(DMA_TypeDef *DMAx)
{
  SET_BIT(DMAx->IFCR , DMA_FLAG_TC5);
}

void DMA_CONSOLE_IRQHANDLER(void)
{
  if(DMA_IsActiveFlag_TC6(DMA1) == 1)
  {
    DMA_ClearFlag_TC6(DMA1);
    DMA1_TransferComplete();
  }
}

void DMA_WIFI_IRQHANDLER(void)
{
  if(DMA_IsActiveFlag_TC5(DMA1) == 1)
  {
    DMA_ClearFlag_TC5(DMA1);
    DMA2_TransferComplete();
  }
}
#endif //USE_STM32F1xx_NUCLEO

#endif //WIFI_USE_VCOM
/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

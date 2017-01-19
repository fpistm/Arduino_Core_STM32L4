/**
  ******************************************************************************
  * @file    clock.c
  * @author  WI6LABS
  * @version V1.0.0
  * @date    15-Febrary-2016
  * @brief   provide clock services for time purpose
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
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
/** @addtogroup CMSIS
  * @{
  */

/** @addtogroup stm32l4xx_system
  * @{
  */

/** @addtogroup STM32L4xx_System_Private_Includes
  * @{
  */
#include "stm32l4xx.h"
#include "hw_config.h"
#include "timer.h"
#include "digital_io.h"

#ifdef __cplusplus
 extern "C" {
#endif

/**
  * @}
  */

/** @addtogroup STM32L4xx_System_Private_TypesDefinitions
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32L4xx_System_Private_Defines
  * @{
  */

#define NB_MS_PER_TICK    10

/**
  * @}
  */

/** @addtogroup STM32L4xx_System_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32L4xx_System_Private_Variables
  * @{
  */
static volatile uint32_t g_current_ms = 1;

/**
  * @}
  */

/** @addtogroup STM32L4xx_System_Private_FunctionPrototypes
  * @{
  */

/**
  * @}
  */

/**
  * @brief  Function called wto read the current micro second
  * @param  None
  * @retval None
  */
uint32_t GetCurrentMicro(void)
{
  return (g_current_ms*1000) + (uint32_t)TIM5->CNT;
}


/**
  * @brief  Function called wto read the current millisecond
  * @param  None
  * @retval None
  */
uint32_t GetCurrentMilli(void)
{
  return g_current_ms;
}

/**
  * @brief  Function called when the tick interruption falls
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
  HAL_IncTick();
}

/**
  * @brief  This function configures the source of the time base.
  * @param  TickPriority: Tick interrupt priority
  * @retval None
  */
HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
  /*Configure the SysTick to have interrupt in 10ms time basis*/
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/(1000/NB_MS_PER_TICK));

  /*Configure the SysTick IRQ priority */
  HAL_NVIC_SetPriority(SysTick_IRQn, TickPriority ,0);


  /*configure TIM5 to get the microsecond precision time.
  1MhZ is applied to the timer. Timer will count until 1ms ends*/
  TimerHandleInit(TIM5_E, 1000 - 1, (uint16_t)(HAL_RCC_GetHCLKFreq() / 1000000) - 1);

   /* Return function status */
  return HAL_OK;
}

/**
  * @brief  TIM5 period elapsed callback in non blocking mode
  * @param  None
  * @retval None
  */
void HAL_TIM5_PeriodElapsedCallback(void)
{
  g_current_ms++;
}

/**
  * @brief  Function provides us delay (required by some arduino libraries).
  *         Can be called inside an interrupt.
  * @param  None
  * @retval None
  */
void delayInsideIT(uint32_t delay_us)
{
  uint32_t nb_loop;

  nb_loop = (((HAL_RCC_GetHCLKFreq() / 1000000)/4)*delay_us)+1; /* uS (divide by 4 because each loop take about 4 cycles including nop +1 is here to avoid delay of 0 */
  __asm__ volatile(
  "1: " "\n\t"
  " nop " "\n\t"
  " subs.w %0, %0, #1 " "\n\t"
  " bne 1b " "\n\t"
  : "=r" (nb_loop)
  : "0"(nb_loop)
  : "r3"
  );
}

/**
  * @}
  */

/**
  * @}
  */

/**
  * @}
  */
#ifdef __cplusplus
}
#endif

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

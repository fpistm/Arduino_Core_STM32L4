/**
  ******************************************************************************
  * @file    twi.c
  * @author  WI6LABS
  * @version V1.0.0
  * @date    14-April-2016
  * @brief   provide the TWI interface
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
#include "twi.h"

/**
  * @}
  */

#ifdef __cplusplus
 extern "C" {
#endif

/** @addtogroup STM32L4xx_System_Private_TypesDefinitions
  * @{
  */
/**
  * @}
  */

/** @addtogroup STM32L4xx_System_Private_Defines
  * @{
  */
  
/// @brief I2C timout in tick unit
#define I2C_TIMEOUT_TICK        100

#define I2C_TXRX_BUFFER_SIZE    32

#define SLAVE_MODE_TRANSMIT     0
#define SLAVE_MODE_RECEIVE      1

/**
  * @}
  */

/** @addtogroup STM32L4xx_System_Private_Macros
  * @{
  */

/**
  * @}
  */

/** @addtogroup STM32L4xx_System_Private_TypesDefinitions
  * @{
  */

/// @brief defines the global attributes of the I2C
typedef struct {
  uint8_t init_done;
  I2C_HandleTypeDef    i2c_handle;
  I2C_TypeDef *i2c_instance;
  IRQn_Type irq;
  void (*i2c_clock_init)(void);
  void (*i2c_scl_clock_init)(void);
  void (*i2c_sda_clock_init)(void);
  void (*i2c_clock_deinit)(void);
  void (*i2c_scl_clock_deinit)(void);
  void (*i2c_sda_clock_deinit)(void);
  void (*i2c_force_reset)(void);
  void (*i2c_release_reset)(void);
  GPIO_TypeDef  *sda_port;
  uint32_t sda_pin;
  uint32_t sda_mode;
  uint32_t sda_pull;
  uint32_t sda_speed;
  uint32_t sda_alternate;
  GPIO_TypeDef  *scl_port;
  uint32_t scl_pin;
  uint32_t scl_mode;
  uint32_t scl_pull;
  uint32_t scl_speed;
  uint32_t scl_alternate;
  void (*i2c_onSlaveReceive)(i2c_instance_e, uint8_t *, int);
  void (*i2c_onSlaveTransmit)(i2c_instance_e);
  uint8_t i2cTxRxBuffer[I2C_TXRX_BUFFER_SIZE];
  uint8_t i2cTxRxBufferSize;
  uint8_t slaveMode;
} i2c_init_info_t;

/**
  * @}
  */

/** @addtogroup STM32L4xx_System_Private_Variables
  * @{
  */
void i2c1_clk_enable(void)      { __HAL_RCC_I2C1_CLK_ENABLE();    }
void i2c1_scl_clk_enable(void)  { __HAL_RCC_GPIOB_CLK_ENABLE();   }
void i2c1_sda_clk_enable(void)  { __HAL_RCC_GPIOB_CLK_ENABLE();   }
void i2c1_clk_disable(void)     { __HAL_RCC_I2C1_CLK_DISABLE();   }
void i2c1_scl_clk_disable(void) { __HAL_RCC_GPIOB_CLK_DISABLE();  }
void i2c1_sda_clk_disable(void) { __HAL_RCC_GPIOB_CLK_DISABLE();  }
void i2c1_force_reset(void)     { __I2C1_FORCE_RESET();           }
void i2c1_release_reset(void)   { __I2C1_RELEASE_RESET();         }

void i2c3_clk_enable(void)      { __HAL_RCC_I2C3_CLK_ENABLE();    }
void i2c3_scl_clk_enable(void)  { __HAL_RCC_GPIOC_CLK_ENABLE();   }
void i2c3_sda_clk_enable(void)  { __HAL_RCC_GPIOC_CLK_ENABLE();   }
void i2c3_clk_disable(void)     { __HAL_RCC_I2C3_CLK_DISABLE();   }
void i2c3_scl_clk_disable(void) { __HAL_RCC_GPIOC_CLK_DISABLE();  }
void i2c3_sda_clk_disable(void) { __HAL_RCC_GPIOC_CLK_DISABLE();  }
void i2c3_force_reset(void)     { __I2C3_FORCE_RESET();           }
void i2c3_release_reset(void)   { __I2C3_RELEASE_RESET();         }

static i2c_init_info_t g_i2c_init_info[NB_I2C_INSTANCES] = {
  {
    .init_done = 0,
    .i2c_instance = I2C1,
    .irq = I2C1_EV_IRQn,
    .i2c_clock_init = i2c1_clk_enable,
    .i2c_scl_clock_init = i2c1_scl_clk_enable,
    .i2c_sda_clock_init = i2c1_sda_clk_enable,
    .i2c_clock_deinit = i2c1_clk_disable,
    .i2c_scl_clock_deinit = i2c1_scl_clk_disable,
    .i2c_sda_clock_deinit = i2c1_sda_clk_disable,
    .i2c_force_reset = i2c1_force_reset,
    .i2c_release_reset = i2c1_release_reset,
    .sda_port = GPIOB,
    .sda_pin = GPIO_PIN_9,
    .sda_mode = GPIO_MODE_AF_OD,
    .sda_pull = GPIO_NOPULL,
    .sda_speed = GPIO_SPEED_FREQ_VERY_HIGH,
    .sda_alternate = GPIO_AF4_I2C1,
    .scl_port = GPIOB,
    .scl_pin = GPIO_PIN_8,
    .scl_mode = GPIO_MODE_AF_OD,
    .scl_pull = GPIO_NOPULL,
    .scl_speed = GPIO_SPEED_FREQ_VERY_HIGH,
    .scl_alternate = GPIO_AF4_I2C1,
    .i2c_onSlaveReceive = NULL,
    .i2c_onSlaveTransmit = NULL,
    .i2cTxRxBufferSize = 0
  },
  {
    .init_done = 0,
    .i2c_instance = I2C3,
    .irq = I2C3_EV_IRQn,
    .i2c_clock_init = i2c3_clk_enable,
    .i2c_scl_clock_init = i2c3_scl_clk_enable,
    .i2c_sda_clock_init = i2c3_sda_clk_enable,
    .i2c_clock_deinit = i2c3_clk_disable,
    .i2c_scl_clock_deinit = i2c3_scl_clk_disable,
    .i2c_sda_clock_deinit = i2c3_sda_clk_disable,
    .i2c_force_reset = i2c3_force_reset,
    .i2c_release_reset = i2c3_release_reset,
    .sda_port = GPIOC,
    .sda_pin = GPIO_PIN_1,
    .sda_mode = GPIO_MODE_AF_OD,
    .sda_pull = GPIO_NOPULL,
    .sda_speed = GPIO_SPEED_FREQ_VERY_HIGH,
    .sda_alternate = GPIO_AF4_I2C3,
    .scl_port = GPIOC,
    .scl_pin = GPIO_PIN_0,
    .scl_mode = GPIO_MODE_AF_OD,
    .scl_pull = GPIO_NOPULL,
    .scl_speed = GPIO_SPEED_FREQ_VERY_HIGH,
    .scl_alternate = GPIO_AF4_I2C3,
    .i2c_onSlaveReceive = NULL,
    .i2c_onSlaveTransmit = NULL,
    .i2cTxRxBufferSize = 0
  }
};

/**
  * @}
  */

/** @addtogroup STM32L4xx_System_Private_FunctionPrototypes
  * @{
  */

static i2c_instance_e i2c_Instance(I2C_HandleTypeDef *hi2c);

/**
  * @}
  */
  
/** @addtogroup STM32L4xx_System_Private_Functions
  * @{
  */


/**
  * @brief  Default init and setup GPIO and I2C peripheral
  * @param  i2c_id : i2c instance to use
  * @retval none
  */
void i2c_init(i2c_instance_e i2c_id)
{
  i2c_custom_init(i2c_id, I2C_100KHz, I2C_ADDRESSINGMODE_7BIT, 0x33, 1);
}

/**
  * @brief  Initialize and setup GPIO and I2C peripheral
  * @param  i2c_id : i2c instance to use
  * @param  timing : one of the i2c_timing_e
  * @param  addressingMode : I2C_ADDRESSINGMODE_7BIT or I2C_ADDRESSINGMODE_10BIT
  * @param  ownAddress : device address
  * @param  master : set to 1 to choose the master mode 
  * @retval none
  */
void i2c_custom_init(i2c_instance_e i2c_id, i2c_timing_e timing, uint32_t addressingMode, uint32_t ownAddress, uint8_t master)
{
  GPIO_InitTypeDef  GPIO_InitStruct;

  if(g_i2c_init_info[i2c_id].init_done == 0) {
    g_i2c_init_info[i2c_id].i2c_scl_clock_init();
    g_i2c_init_info[i2c_id].i2c_sda_clock_init();

    //SCL
    GPIO_InitStruct.Pin = g_i2c_init_info[i2c_id].scl_pin;
    GPIO_InitStruct.Mode = g_i2c_init_info[i2c_id].scl_mode;
    GPIO_InitStruct.Speed = g_i2c_init_info[i2c_id].scl_speed;
    GPIO_InitStruct.Pull  = g_i2c_init_info[i2c_id].scl_pull;
    GPIO_InitStruct.Alternate  = g_i2c_init_info[i2c_id].scl_alternate;
    HAL_GPIO_Init(g_i2c_init_info[i2c_id].scl_port, &GPIO_InitStruct);

    //SDA
    GPIO_InitStruct.Pin = g_i2c_init_info[i2c_id].sda_pin;
    GPIO_InitStruct.Mode = g_i2c_init_info[i2c_id].sda_mode;
    GPIO_InitStruct.Speed = g_i2c_init_info[i2c_id].sda_speed;
    GPIO_InitStruct.Pull  = g_i2c_init_info[i2c_id].sda_pull;
    GPIO_InitStruct.Alternate  = g_i2c_init_info[i2c_id].sda_alternate;
    HAL_GPIO_Init(g_i2c_init_info[i2c_id].sda_port, &GPIO_InitStruct);

    if(i2c_id == I2C_1) {
        //force hisgh speed clock for I2C1
      __HAL_RCC_I2C1_CONFIG(RCC_I2C1CLKSOURCE_SYSCLK);
    }

    //starting I2C
    g_i2c_init_info[i2c_id].i2c_clock_init();
    g_i2c_init_info[i2c_id].i2c_force_reset();
    g_i2c_init_info[i2c_id].i2c_release_reset();

    g_i2c_init_info[i2c_id].i2c_handle.Init.Timing = timing;
    g_i2c_init_info[i2c_id].i2c_handle.Init.OwnAddress1 = ownAddress;
    g_i2c_init_info[i2c_id].i2c_handle.Init.AddressingMode = addressingMode;

    g_i2c_init_info[i2c_id].i2c_handle.Init.DualAddressMode = I2C_DUALADDRESS_DISABLED;
    g_i2c_init_info[i2c_id].i2c_handle.Init.GeneralCallMode = I2C_GENERALCALL_DISABLED;
    g_i2c_init_info[i2c_id].i2c_handle.Init.NoStretchMode = I2C_NOSTRETCH_DISABLED;
    g_i2c_init_info[i2c_id].i2c_handle.Instance = g_i2c_init_info[i2c_id].i2c_instance;
    
    if(master == 0) {
      HAL_NVIC_SetPriority(g_i2c_init_info[i2c_id].irq, 0, 1);
      HAL_NVIC_EnableIRQ(g_i2c_init_info[i2c_id].irq);
    }

    // Init the I2C
    HAL_I2C_Init(&g_i2c_init_info[i2c_id].i2c_handle);
    g_i2c_init_info[i2c_id].init_done = 1;
  }
}

/**
  * @brief  Initialize and setup GPIO and I2C peripheral
  * @param  i2c_id : i2c instance to use
  * @retval none
  */
void i2c_deinit(i2c_instance_e i2c_id)
{
  if(g_i2c_init_info[i2c_id].init_done == 1) {

    g_i2c_init_info[i2c_id].i2c_scl_clock_deinit();
    g_i2c_init_info[i2c_id].i2c_sda_clock_deinit();
    g_i2c_init_info[i2c_id].i2c_clock_deinit();
    
    HAL_NVIC_DisableIRQ(g_i2c_init_info[i2c_id].irq);

    HAL_I2C_DeInit(&g_i2c_init_info[i2c_id].i2c_handle);

    g_i2c_init_info[i2c_id].init_done = 0;
  }
}

/**
  * @brief  Setup transmission speed. I2C must be configured before.
  * @param  i2c_id : i2c instance to use
  * @param  frequency : i2c transmission speed
  * @retval none
  */
void i2c_setTiming(i2c_instance_e i2c_id, uint32_t frequency)
{
  if(g_i2c_init_info[i2c_id].init_done == 1) {
    __HAL_I2C_DISABLE(&g_i2c_init_info[i2c_id].i2c_handle);
    
    if(frequency <= 10000)
      g_i2c_init_info[i2c_id].i2c_handle.Init.Timing = I2C_10KHz;
    else if(frequency <= 50000)
      g_i2c_init_info[i2c_id].i2c_handle.Init.Timing = I2C_50KHz;
    else if(frequency <= 100000)
      g_i2c_init_info[i2c_id].i2c_handle.Init.Timing = I2C_100KHz;
    else if(frequency <= 200000)
      g_i2c_init_info[i2c_id].i2c_handle.Init.Timing = I2C_200KHz;
    else if(frequency <= 400000)
      g_i2c_init_info[i2c_id].i2c_handle.Init.Timing = I2C_400KHz;
    else if(frequency <= 600000)
      g_i2c_init_info[i2c_id].i2c_handle.Init.Timing = I2C_600KHz;
    else if(frequency <= 800000)
      g_i2c_init_info[i2c_id].i2c_handle.Init.Timing = I2C_800KHz;
    else
      g_i2c_init_info[i2c_id].i2c_handle.Init.Timing = I2C_1000KHz;
      
    HAL_I2C_Init(&g_i2c_init_info[i2c_id].i2c_handle);
    
    __HAL_I2C_ENABLE(&g_i2c_init_info[i2c_id].i2c_handle);
  }
}

/**
  * @brief  Write bytes at a given address
  * @param  i2c_id : i2c instance to use
  * @param  dev_address: specifies the address of the device.
  * @param  data: pointer to data to be write
  * @param  size: number of bytes to be write.
  * @retval read status
  */
i2c_status_e i2c_master_write(i2c_instance_e i2c_id, uint8_t dev_address,
                        uint8_t *data, uint8_t size)

{
  i2c_status_e ret = I2C_ERROR;
  HAL_StatusTypeDef status = HAL_OK;

  if(g_i2c_init_info[i2c_id].init_done == 1) {
    // Check the communication status
    status = HAL_I2C_Master_Transmit(&g_i2c_init_info[i2c_id].i2c_handle, dev_address,
                               data, size, I2C_TIMEOUT_TICK);
    if(status == HAL_OK)
      ret = I2C_OK;
    else if(status == HAL_TIMEOUT)
      ret = I2C_TIMEOUT;
    else
      ret = I2C_ERROR;
  }

  return ret;
}

/**
  * @brief  Write bytes to master
  * @param  i2c_id : i2c instance to use
  * @param  data: pointer to data to be write
  * @param  size: number of bytes to be write.
  * @retval none
  */
void i2c_slave_write_IT(i2c_instance_e i2c_id, uint8_t *data, uint8_t size)
{
  uint8_t i = 0;
  
  if(g_i2c_init_info[i2c_id].init_done == 1) {
    // Check the communication status
    for(i = 0; i < size; i++) {
      g_i2c_init_info[i2c_id].i2cTxRxBuffer[i] = *(data+i);
      g_i2c_init_info[i2c_id].i2cTxRxBufferSize++;
    }
  }
}

/**
  * @brief  read bytes in master mode at a given address
  * @param  i2c_id : i2c instance to use
  * @param  dev_address: specifies the address of the device.
  * @param  data: pointer to data to be read
  * @param  size: number of bytes to be read.
  * @retval read status
  */
i2c_status_e i2c_master_read(i2c_instance_e i2c_id, uint8_t dev_address,
                              uint8_t *data, uint8_t size)
{
  i2c_status_e ret = I2C_ERROR;

  if(g_i2c_init_info[i2c_id].init_done == 1) {

    if(HAL_I2C_Master_Receive(&g_i2c_init_info[i2c_id].i2c_handle,
                                dev_address, data, size,
                               I2C_TIMEOUT_TICK) == HAL_OK) {
      ret = I2C_OK;
    }

  }
  return ret;
}

/**
  * @brief  Checks if target device is ready for communication
  * @param  i2c_id : i2c instance to use
  * @param  devAddr: specifies the address of the device.
  * @param  trials : Number of trials.
  * @retval status
  */
i2c_status_e i2c_IsDeviceReady(i2c_instance_e i2c_id, uint8_t devAddr,
                               uint32_t trials)
{
  i2c_status_e ret = HAL_OK;
  if(HAL_I2C_IsDeviceReady( &g_i2c_init_info[i2c_id].i2c_handle, devAddr,
                            trials, I2C_TIMEOUT_TICK)!=HAL_OK) {
    ret = I2C_BUSY;
  }

  return ret;
}

/**
  * @brief  Checks if target device is ready for communication
  * @param  i2c_id : i2c instance to use
  * @param  devAddr: specifies the address of the device.
  * @param  trials : Number of trials.
  * @retval status
  */
i2c_instance_e i2c_Instance(I2C_HandleTypeDef *hi2c)
{
  int i = 0;
  
  for(i=0;i<NB_I2C_INSTANCES;i++) {
    if(hi2c == &g_i2c_init_info[i].i2c_handle)
      return i;
  }
  
  return NB_I2C_INSTANCES;
}

/** @brief  sets function called before a slave read operation
  * @param  i2c_id : i2c instance to use
  * @param  function: callback function to use
  * @retval None
  */
void i2c_attachSlaveRxEvent(i2c_instance_e i2c_id, void (*function)(i2c_instance_e, uint8_t*, int) )
{
  if(g_i2c_init_info[i2c_id].init_done == 1){
    g_i2c_init_info[i2c_id].i2c_onSlaveReceive = function;
    HAL_I2C_EnableListen_IT(&g_i2c_init_info[i2c_id].i2c_handle);
  }
}

/** @brief  sets function called before a slave write operation
  * @param  i2c_id : i2c instance to use
  * @param  function: callback function to use
  * @retval None
  */
void i2c_attachSlaveTxEvent(i2c_instance_e i2c_id, void (*function)(i2c_instance_e) )
{
  if(g_i2c_init_info[i2c_id].init_done == 1){
    g_i2c_init_info[i2c_id].i2c_onSlaveTransmit = function;
    HAL_I2C_EnableListen_IT(&g_i2c_init_info[i2c_id].i2c_handle);
  }
}

/**
  * @brief  Slave Address Match callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  TransferDirection: Master request Transfer Direction (Write/Read), value of @ref I2C_XferOptions_definition
  * @param  AddrMatchCode: Address Match Code
  * @retval None
  */
void HAL_I2C_AddrCallback(I2C_HandleTypeDef *hi2c, uint8_t TransferDirection, uint16_t AddrMatchCode)
{
  if(AddrMatchCode == hi2c->Init.OwnAddress1) {
    if(TransferDirection == 1) {
      g_i2c_init_info[i2c_Instance(hi2c)].i2cTxRxBufferSize = 0;
      g_i2c_init_info[i2c_Instance(hi2c)].slaveMode = SLAVE_MODE_TRANSMIT;
      
      if(g_i2c_init_info[i2c_Instance(hi2c)].i2c_onSlaveTransmit != NULL)
        g_i2c_init_info[i2c_Instance(hi2c)].i2c_onSlaveTransmit(i2c_Instance(hi2c));

      HAL_I2C_Slave_Sequential_Transmit_IT(hi2c, 
                                           g_i2c_init_info[i2c_Instance(hi2c)].i2cTxRxBuffer, 
                                           g_i2c_init_info[i2c_Instance(hi2c)].i2cTxRxBufferSize, 
                                           I2C_LAST_FRAME);
    } else {
      g_i2c_init_info[i2c_Instance(hi2c)].slaveMode = SLAVE_MODE_RECEIVE;
      HAL_I2C_Slave_Sequential_Receive_IT(hi2c, 
                                          g_i2c_init_info[i2c_Instance(hi2c)].i2cTxRxBuffer, 
                                          I2C_TXRX_BUFFER_SIZE, I2C_LAST_FRAME);
    }
  }
}

/**
  * @brief  Listen Complete callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_ListenCpltCallback(I2C_HandleTypeDef *hi2c)
{
  uint8_t nbData = 0;
      
  if((g_i2c_init_info[i2c_Instance(hi2c)].i2c_onSlaveReceive != NULL) &&
    (g_i2c_init_info[i2c_Instance(hi2c)].slaveMode == SLAVE_MODE_RECEIVE)) {
    nbData = I2C_TXRX_BUFFER_SIZE - g_i2c_init_info[i2c_Instance(hi2c)].i2c_handle.XferSize;

    if(nbData != 0)
      g_i2c_init_info[i2c_Instance(hi2c)].i2c_onSlaveReceive(i2c_Instance(hi2c), g_i2c_init_info[i2c_Instance(hi2c)].i2cTxRxBuffer, nbData);
  }

  HAL_I2C_EnableListen_IT(hi2c);
}

/**
  * @brief  I2C error callback.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval None
  */
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  HAL_I2C_EnableListen_IT(hi2c);
}

/**
* @brief  This function handles I2C1 interrupt.
* @param  None
* @retval None
*/
void I2C1_EV_IRQHandler(void)
{
  HAL_I2C_EV_IRQHandler(&g_i2c_init_info[I2C_1].i2c_handle);
}

/**
* @brief  This function handles I2C1 interrupt.
* @param  None
* @retval None
*/
void I2C3_EV_IRQHandler(void)
{
  HAL_I2C_EV_IRQHandler(&g_i2c_init_info[I2C_3].i2c_handle);
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

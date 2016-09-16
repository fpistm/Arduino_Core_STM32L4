/**
  ******************************************************************************
  * @file    twi.h
  * @author  WI6LABS
  * @version V1.0.0
  * @date    14-April-2016
  * @brief   Header for twi module
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


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __TWI_H__
#define __TWI_H__

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

#ifdef __cplusplus
 extern "C" {
#endif

/* Exported types ------------------------------------------------------------*/

///@brief define the possible I2C instances
typedef enum {
  I2C_1,
  I2C_3,
  NB_I2C_INSTANCES
}i2c_instance_e;

///@brief I2C state
typedef enum {
  I2C_OK = 0,
  I2C_TIMEOUT = 1,
  I2C_ERROR = 2,
  I2C_BUSY = 3
}i2c_status_e;

//calculated with SYSCLK = 64MHz at
/*https://www.google.fr/url?sa=t&rct=j&q=&esrc=s&source=web&cd=2&cad=rja&uact=8&ved=0ahUKEwiC4q6O7ojMAhWCOhoKHYlyBtIQFggmMAE&url=http%3A%2F%2Fuglyduck.ath.cx%2FPDF%2FSTMicro%2FARM%2FSTM32F0%2FI2C_Timing_Configuration_V1.0.1.xls&usg=AFQjCNGGjPSUAzVUdbUqMUxPub8Ojzhh9w&sig2=4YgzXFixj15GhqkAzVS4tA*/
typedef enum {
  I2C_10KHz =   0xE010A9FF,
  I2C_50KHz =   0x2070A8FD,
  I2C_100KHz =  0x10B07EBA,
  I2C_200KHz =  0x00C034FF,
  I2C_400KHz =  0x00C0246F,
  I2C_600KHz =  0x00900E50,
  I2C_800KHz =  0x00900E35,
  I2C_1000KHz = 0x00900E25
}i2c_timing_e;

/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */
void i2c_init(i2c_instance_e i2c_id);
void i2c_custom_init(i2c_instance_e i2c_id, i2c_timing_e timing, 
                     uint32_t addressingMode, uint32_t ownAddress, uint8_t master);
void i2c_deinit(i2c_instance_e i2c_id);
void i2c_setTiming(i2c_instance_e i2c_id, uint32_t frequency);
i2c_status_e i2c_master_write(i2c_instance_e i2c_id, uint8_t dev_address,
                        uint8_t *data, uint8_t size);
void i2c_slave_write_IT(i2c_instance_e i2c_id, uint8_t *data, uint8_t size);
i2c_status_e i2c_master_read(i2c_instance_e i2c_id, uint8_t dev_address,
                              uint8_t *data, uint8_t size);

i2c_status_e i2c_IsDeviceReady(i2c_instance_e i2c_id, uint8_t devAddr,
                               uint32_t trials);                         
void i2c_attachSlaveRxEvent(i2c_instance_e i2c_id, void (*function)(i2c_instance_e, uint8_t*, int) );
void i2c_attachSlaveTxEvent(i2c_instance_e i2c_id, void (*function)(i2c_instance_e) );

#ifdef __cplusplus
}
#endif

#endif /* __TWI_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

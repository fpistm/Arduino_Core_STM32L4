/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "Arduino.h"

#ifdef __cplusplus
 extern "C" {
#endif


//This is the list of the digital IOs configured
PinDescription g_digPinConfigured[MAX_DIGITAL_IOS];
extern PinDescription g_anOutputPinConfigured[MAX_DIGITAL_IOS];


extern void pinMode( uint32_t ulPin, uint32_t ulMode )
{
  int i;

  //not a valid pin
  if(ulPin>MAX_DIGITAL_IOS) {
    return ;
  }

  //find the pin.
  for(i = 0; i < NB_PIN_DESCRIPTIONS; i++) {
    if(g_APinDescription[i].arduino_id == ulPin) {
      g_digPinConfigured[ulPin] = g_APinDescription[i];
      g_digPinConfigured[ulPin].configured = true;
      break;
    }
  }
  
  // If the pin that support PWM or DAC output, we need to turn it off
  if(g_anOutputPinConfigured[ulPin].configured == true) {
    if((g_anOutputPinConfigured[ulPin].mode & GPIO_PIN_PWM) == GPIO_PIN_PWM) {
      pwm_stop(g_anOutputPinConfigured[ulPin].ulPort, g_anOutputPinConfigured[ulPin].ulPin);
    } else if((g_anOutputPinConfigured[ulPin].mode & GPIO_PIN_DAC) == GPIO_PIN_DAC) {
      dac_stop(g_anOutputPinConfigured[ulPin].ulPort, g_anOutputPinConfigured[ulPin].ulPin);
    }
    
    g_anOutputPinConfigured[ulPin].configured = false;
  }
  
  switch ( ulMode )
  {
    case INPUT:
      digital_io_init(g_digPinConfigured[ulPin].ulPort,
                    g_digPinConfigured[ulPin].ulPin,
                    GPIO_MODE_INPUT, GPIO_NOPULL);
    break;
    case INPUT_PULLUP:
      digital_io_init(g_digPinConfigured[ulPin].ulPort,
                    g_digPinConfigured[ulPin].ulPin,
                    GPIO_MODE_INPUT, GPIO_PULLUP);
    break;
    case INPUT_PULLDOWN:
      digital_io_init(g_digPinConfigured[ulPin].ulPort,
                    g_digPinConfigured[ulPin].ulPin,
                    GPIO_MODE_INPUT, GPIO_PULLDOWN);
    break;
    case OUTPUT:
      digital_io_init(g_digPinConfigured[ulPin].ulPort,
                    g_digPinConfigured[ulPin].ulPin,
                    GPIO_MODE_OUTPUT_PP, GPIO_NOPULL);
    break;
    default:
    break;
  }
}

extern void digitalWrite( uint32_t ulPin, uint32_t ulVal )
{
  //not a valid pin
  if(ulPin>MAX_DIGITAL_IOS) {
    return ;
  }

  if(g_digPinConfigured[ulPin].configured == true) {
    digital_io_write(g_digPinConfigured[ulPin].ulPort,
                  g_digPinConfigured[ulPin].ulPin,
                  ulVal);
  }
}

extern int digitalRead( uint32_t ulPin )
{

  uint8_t level = 0;
  //not a valid pin
  if(ulPin>MAX_DIGITAL_IOS) {
    return LOW;
  }

  if(g_digPinConfigured[ulPin].configured == true) {
    level = digital_io_read(g_digPinConfigured[ulPin].ulPort,
                        g_digPinConfigured[ulPin].ulPin);
  }

  if(level) {
    return HIGH;
  } else {
    return LOW;
  }
}

#ifdef __cplusplus
}
#endif

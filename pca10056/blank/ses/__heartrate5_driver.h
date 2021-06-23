/*
    __heartrate5_driver.h

-----------------------------------------------------------------------------

  This file is part of mikroSDK.
  
  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

----------------------------------------------------------------------------- */

/**
@file   __heartrate5_driver.h
@brief    Heart_Rate_5 Driver
@mainpage Heart_Rate_5 Click
@{

@image html libstock_fb_view.jpg

@}

@defgroup   HEARTRATE5
@brief      Heart_Rate_5 Click Driver
@{

| Global Library Prefix | **HEARTRATE5** |
|:---------------------:|:-----------------:|
| Version               | **1.0.0**    |
| Date                  | **mar 2018.**      |
| Developer             | **MikroE Team**     |

*/
/* -------------------------------------------------------------------------- */

#include "stdint.h"

#ifndef _HEARTRATE5_H_
#define _HEARTRATE5_H_

/** 
 * @macro T_HEARTRATE5_P
 * @brief Driver Abstract type 
 */
#define T_HEARTRATE5_P    const uint8_t*

/** @defgroup HEARTRATE5_COMPILE Compilation Config */              /** @{ */

//  #define   __HEARTRATE5_DRV_SPI__                            /**<     @macro __HEARTRATE5_DRV_SPI__  @brief SPI driver selector */
   #define   __HEARTRATE5_DRV_I2C__                            /**<     @macro __HEARTRATE5_DRV_I2C__  @brief I2C driver selector */                                          
// #define   __HEARTRATE5_DRV_UART__                           /**<     @macro __HEARTRATE5_DRV_UART__ @brief UART driver selector */ 

                                                                       /** @} */
/** @defgroup HEARTRATE5_VAR Variables */                           /** @{ */
extern const uint8_t HR5_REG0H  ;
extern const uint8_t HR5_REG1H  ;
extern const uint8_t HR5_REG2H  ;
extern const uint8_t HR5_REG3H  ;
extern const uint8_t HR5_REG4H  ;
extern const uint8_t HR5_REG5H  ;
extern const uint8_t HR5_REG6H  ;
extern const uint8_t HR5_REG7H  ;
extern const uint8_t HR5_REG8H  ;
extern const uint8_t HR5_REG9H  ;
extern const uint8_t HR5_REGAH  ;
extern const uint8_t HR5_REGBH  ;
extern const uint8_t HR5_REGCH  ;
extern const uint8_t HR5_REGDH  ;
extern const uint8_t HR5_REGEH  ;
extern const uint8_t HR5_REGFH  ;
extern const uint8_t HR5_REG10H ;
extern const uint8_t HR5_REG11H ;
extern const uint8_t HR5_REG12H ;
extern const uint8_t HR5_REG13H ;
extern const uint8_t HR5_REG14H ;
extern const uint8_t HR5_REG15H ;
extern const uint8_t HR5_REG16H ;
extern const uint8_t HR5_REG17H ;
extern const uint8_t HR5_REG18H ;
extern const uint8_t HR5_REG19H ;
extern const uint8_t HR5_REG1AH ;
extern const uint8_t HR5_REG1BH ;
extern const uint8_t HR5_REG1CH ;
extern const uint8_t HR5_REG1DH ;
extern const uint8_t HR5_REG1EH ;
extern const uint8_t HR5_REG20H ;
extern const uint8_t HR5_REG21H ;
extern const uint8_t HR5_REG22H ;
extern const uint8_t HR5_REG23H ;
extern const uint8_t HR5_REG29H ;
extern const uint8_t HR5_REG2AH ;
extern const uint8_t HR5_REG2BH ;
extern const uint8_t HR5_REG2CH ;
extern const uint8_t HR5_REG2DH ;
extern const uint8_t HR5_REG2EH ;
extern const uint8_t HR5_REG2FH ;
extern const uint8_t HR5_REG31H ;
extern const uint8_t HR5_REG32H ;
extern const uint8_t HR5_REG33H ;
extern const uint8_t HR5_REG34H ;
extern const uint8_t HR5_REG35H ;
extern const uint8_t HR5_REG36H ;
extern const uint8_t HR5_REG37H ;
extern const uint8_t HR5_REG39H ;
extern const uint8_t HR5_REG3AH ;
extern const uint8_t HR5_REG3DH ;
extern const uint8_t HR5_ADDR;


                                                                       /** @} */
/** @defgroup HEARTRATE5_TYPES Types */                             /** @{ */



                                                                       /** @} */
#ifdef __cplusplus
extern "C"{
#endif

/** @defgroup HEARTRATE5_INIT Driver Initialization */              /** @{ */

#ifdef   __HEARTRATE5_DRV_SPI__
void heartrate5_spiDriverInit(T_HEARTRATE5_P gpioObj, T_HEARTRATE5_P spiObj);
#endif
#ifdef   __HEARTRATE5_DRV_I2C__
void heartrate5_i2cDriverInit(T_HEARTRATE5_P gpioObj, T_HEARTRATE5_P i2cObj, uint8_t slave);
#endif
#ifdef   __HEARTRATE5_DRV_UART__
void heartrate5_uartDriverInit(T_HEARTRATE5_P gpioObj, T_HEARTRATE5_P uartObj);
#endif

// GPIO Only Drivers - remove in other cases
void heartrate5_gpioDriverInit(T_HEARTRATE5_P gpioObj);
                                                                       /** @} */
/** @defgroup HEARTRATE5_FUNC Driver Functions */                   /** @{ */

void heartrate5_writeReg(uint8_t regAddr,uint32_t wData); /** Function for writing a 24bit value to a register. */
uint32_t heartrate5_readReg(uint8_t regAddr); /** Function for reading a 24bit value from a register. */
void heartrate5_init(); /** Default initialization routine */
void heartrate5_hwReset();/** Hardware reset function */
void heartrate5_swReset();/** Software reset function */
uint32_t heartrate5_getLed2val( void );  /** Returns the value of the conversion resoult from LED2VAL.*/
uint32_t heartrate5_getAled2val_led3val( void ); //Returns the value of the conversion resoult from ALED2VAL.*/
uint32_t heartrate5_getLed1val( void );  /**Returns the value of the conversion resoult from LED1VAL.*/
uint32_t heartrate5_getAled1val( void ); /**Returns the value of the conversion resoult from ALED1VAL.*/
uint32_t heartrate5_getLed2_aled2val( void ); /**Returns the value of the conversion resoult from LED2_ALED2VAL.*/
uint32_t heartrate5_getLed1_aled1val( void ); /**Returns the value of the conversion resoult from LED1_ALED1VAL.*/




                                                                       /** @} */
#ifdef __cplusplus
} // extern "C"
#endif
#endif

/**
    @example Click_Heart_Rate_5_STM.c
    @example Click_Heart_Rate_5_TIVA.c
    @example Click_Heart_Rate_5_CEC.c
    @example Click_Heart_Rate_5_KINETIS.c
    @example Click_Heart_Rate_5_MSP.c
    @example Click_Heart_Rate_5_PIC.c
    @example Click_Heart_Rate_5_PIC32.c
    @example Click_Heart_Rate_5_DSPIC.c
    @example Click_Heart_Rate_5_AVR.c
    @example Click_Heart_Rate_5_FT90x.c
    @example Click_Heart_Rate_5_STM.mbas
    @example Click_Heart_Rate_5_TIVA.mbas
    @example Click_Heart_Rate_5_CEC.mbas
    @example Click_Heart_Rate_5_KINETIS.mbas
    @example Click_Heart_Rate_5_MSP.mbas
    @example Click_Heart_Rate_5_PIC.mbas
    @example Click_Heart_Rate_5_PIC32.mbas
    @example Click_Heart_Rate_5_DSPIC.mbas
    @example Click_Heart_Rate_5_AVR.mbas
    @example Click_Heart_Rate_5_FT90x.mbas
    @example Click_Heart_Rate_5_STM.mpas
    @example Click_Heart_Rate_5_TIVA.mpas
    @example Click_Heart_Rate_5_CEC.mpas
    @example Click_Heart_Rate_5_KINETIS.mpas
    @example Click_Heart_Rate_5_MSP.mpas
    @example Click_Heart_Rate_5_PIC.mpas
    @example Click_Heart_Rate_5_PIC32.mpas
    @example Click_Heart_Rate_5_DSPIC.mpas
    @example Click_Heart_Rate_5_AVR.mpas
    @example Click_Heart_Rate_5_FT90x.mpas
*/                                                                     /** @} */
/* -------------------------------------------------------------------------- */
/*
  __heartrate5_driver.h

  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.

3. All advertising materials mentioning features or use of this software
   must display the following acknowledgement:
   This product includes software developed by the MikroElektonika.

4. Neither the name of the MikroElektonika nor the
   names of its contributors may be used to endorse or promote products
   derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY MIKROELEKTRONIKA ''AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL MIKROELEKTRONIKA BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------------- */

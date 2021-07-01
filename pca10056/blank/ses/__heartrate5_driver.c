/*
    __heartrate5_driver.c

-----------------------------------------------------------------------------

  This file is part of mikroSDK.

  Copyright (c) 2017, MikroElektonika - http://www.mikroe.com

  All rights reserved.

----------------------------------------------------------------------------- */

#include "__heartrate5_driver.h"
#include "__heartrate5_hal.c"

/* ------------------------------------------------------------------- MACROS */



/* ---------------------------------------------------------------- VARIABLES */

#ifdef   __HEARTRATE5_DRV_I2C__
static uint8_t _slaveAddress;
#endif
const uint8_t HR5_REG0H                      = 0x00;
const uint8_t HR5_REG1H                      = 0x01; //LED2STC
const uint8_t HR5_REG2H                      = 0x02; //LED2ENDC
const uint8_t HR5_REG3H                      = 0x03; //LED1LEDSTC
const uint8_t HR5_REG4H                      = 0x04; //LED1LEDENDC
const uint8_t HR5_REG5H                      = 0x05; //ALED2STC\LED3STC
const uint8_t HR5_REG6H                      = 0x06; //ALED2ENDC\LED3ENDC
const uint8_t HR5_REG7H                      = 0x07; //LED1STC
const uint8_t HR5_REG8H                      = 0x08; //LED1ENDC
const uint8_t HR5_REG9H                      = 0x09; //LED2LEDSTC
const uint8_t HR5_REGAH                      = 0x0A; //LED2LEDENDC
const uint8_t HR5_REGBH                      = 0x0B; //ALED1STC
const uint8_t HR5_REGCH                      = 0x0C; //ALED1ENDC
const uint8_t HR5_REGDH                      = 0x0D; //LED2CONVST
const uint8_t HR5_REGEH                      = 0x0E; //LED2CONVEND
const uint8_t HR5_REGFH                      = 0x0F; //ALED2CONVST\LED3CONVST
const uint8_t HR5_REG10H                     = 0x10; //ALED2CONVEND\LED3CONVEND
const uint8_t HR5_REG11H                     = 0x11; //LED1CONVST
const uint8_t HR5_REG12H                     = 0x12; //LED1CONVEND
const uint8_t HR5_REG13H                     = 0x13; //ALED1CONVST
const uint8_t HR5_REG14H                     = 0x14; //ALED1CONVEND
const uint8_t HR5_REG15H                     = 0x15; //ADCRSTSTCT0
const uint8_t HR5_REG16H                     = 0x16; //ADCRSTENDCT0
const uint8_t HR5_REG17H                     = 0x17; //ADCRSTSTCT1
const uint8_t HR5_REG18H                     = 0x18; //ADCRSTENDCT1
const uint8_t HR5_REG19H                     = 0x19; //ADCRSTSTCT2
const uint8_t HR5_REG1AH                     = 0x1A; //ADCRSTENDCT2
const uint8_t HR5_REG1BH                     = 0x1B; //ADCRSTSTCT3
const uint8_t HR5_REG1CH                     = 0x1C; //ADCRSTENDCT3
const uint8_t HR5_REG1DH                     = 0x1D; //PRPCT
const uint8_t HR5_REG1EH                     = 0x1E; //timer controll
const uint8_t HR5_REG20H                     = 0x20;
const uint8_t HR5_REG21H                     = 0x21;
const uint8_t HR5_REG22H                     = 0x22;
const uint8_t HR5_REG23H                     = 0x23;
const uint8_t HR5_REG29H                     = 0x29;

const uint8_t HR5_REG2AH                     = 0x2A;//LED2VAL
const uint8_t HR5_REG2BH                     = 0x2B;//ALED2VAL\ALSED3VAL
const uint8_t HR5_REG2CH                     = 0x2C;//LED1VAL
const uint8_t HR5_REG2DH                     = 0x2D;//ALED1VAL
const uint8_t HR5_REG2EH                     = 0x2E;//LED2-ALED2VAL
const uint8_t HR5_REG2FH                     = 0x2F;//LED1-ALED1VAL

const uint8_t HR5_REG31H                     = 0x31;
const uint8_t HR5_REG32H                     = 0x32;
const uint8_t HR5_REG33H                     = 0x33;
const uint8_t HR5_REG34H                     = 0x34;
const uint8_t HR5_REG35H                     = 0x35;
const uint8_t HR5_REG36H                     = 0x36;
const uint8_t HR5_REG37H                     = 0x37;
const uint8_t HR5_REG39H                     = 0x39;
const uint8_t HR5_REG3AH                     = 0x3A;
const uint8_t HR5_REG3DH                     = 0x3D;
const uint8_t HR5_REG3FH                     = 0x3F;
const uint8_t HR5_REG40H                     = 0x40;

const uint8_t HR5_ADDR                       = 0x58;

/* -------------------------------------------- PRIVATE FUNCTION DECLARATIONS */



/* --------------------------------------------- PRIVATE FUNCTION DEFINITIONS */



/* --------------------------------------------------------- PUBLIC FUNCTIONS */

#ifdef   __HEARTRATE5_DRV_SPI__

void heartrate5_spiDriverInit(T_HEARTRATE5_P gpioObj, T_HEARTRATE5_P spiObj)
{
    hal_spiMap( (T_HAL_P)spiObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif
#ifdef   __HEARTRATE5_DRV_I2C__

void heartrate5_i2cDriverInit(T_HEARTRATE5_P gpioObj, T_HEARTRATE5_P i2cObj, uint8_t slave)
{
    _slaveAddress = slave;
    hal_i2cMap( (T_HAL_P)i2cObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif
#ifdef   __HEARTRATE5_DRV_UART__

void heartrate5_uartDriverInit(T_HEARTRATE5_P gpioObj, T_HEARTRATE5_P uartObj)
{
    hal_uartMap( (T_HAL_P)uartObj );
    hal_gpioMap( (T_HAL_P)gpioObj );

    // ... power ON
    // ... configure CHIP
}

#endif
void heartrate5_writeReg(uint8_t regAddr,uint32_t wData)
{
    uint8_t hr_storage[5];
    hr_storage[0]=regAddr;
    hr_storage[1]=wData>>16;
    hr_storage[2]=wData>>8;
    hr_storage[3]=wData;

    //hal_i2cStart();
    //hal_i2cWrite(_slaveAddress,hr_storage,4,END_MODE_STOP);
}
uint32_t heartrate5_readReg(uint8_t regAddr)
{
    uint8_t hr_storage[5];
    uint32_t returnValue=0;
    hr_storage[0]=regAddr;

    //hal_i2cStart();
    //hal_i2cWrite(_slaveAddress,hr_storage,1,END_MODE_RESTART);
    //hal_i2cRead(_slaveAddress,hr_storage,3,END_MODE_STOP);

    returnValue = hr_storage[0];
    returnValue = returnValue<<16;
    returnValue |= hr_storage[1]<<8;
    returnValue |= hr_storage[2];
    return returnValue;
}
void heartrate5_init()
{
    heartrate5_writeReg(0x00, 0x000000);
    heartrate5_writeReg(0x01, 0x000050);
    heartrate5_writeReg(0x02, 0x00018F);
    heartrate5_writeReg(0x03, 0x000320);
    heartrate5_writeReg(0x04, 0x0004AF);
    heartrate5_writeReg(0x05, 0x0001E0);
    heartrate5_writeReg(0x06, 0x00031F);
    heartrate5_writeReg(0x07, 0x000370);
    heartrate5_writeReg(0x08, 0x0004AF);
    heartrate5_writeReg(0x09, 0x000000);
    heartrate5_writeReg(0x0A, 0x00018F);
    heartrate5_writeReg(0x0B, 0x0004FF);
    heartrate5_writeReg(0x0C, 0x00063E);
    heartrate5_writeReg(0x0D, 0x000198);
    heartrate5_writeReg(0x0E, 0x0005BB);
    heartrate5_writeReg(0x0F, 0x0005C4);
    heartrate5_writeReg(0x10, 0x0009E7);
    heartrate5_writeReg(0x11, 0x0009F0);
    heartrate5_writeReg(0x12, 0x000E13);
    heartrate5_writeReg(0x13, 0x000E1C);
    heartrate5_writeReg(0x14, 0x00123F);
    heartrate5_writeReg(0x15, 0x000191);
    heartrate5_writeReg(0x16, 0x000197);
    heartrate5_writeReg(0x17, 0x0005BD);
    heartrate5_writeReg(0x18, 0x0005C3);
    heartrate5_writeReg(0x19, 0x0009E9);
    heartrate5_writeReg(0x1A, 0x0009EF);
    heartrate5_writeReg(0x1B, 0x000E15);
    heartrate5_writeReg(0x1C, 0x000E1B);
    heartrate5_writeReg(0x1D, 0x009C3E);
    heartrate5_writeReg(0x1E, 0x000103); // timeren set ; broj semplova od 0 do F
    heartrate5_writeReg(0x20, 0x008003); //ENSEPGAIN  ; TIA_CF_SEP
    heartrate5_writeReg(0x21, 0x000003); //PROG_TG_EN(disabled) ; TIA_CF(0); TIA_GAIN(3);
    heartrate5_writeReg(0x22, 0x01B6D9); //LED3(bit12-bit17); LED2(bit6-bit11); LED1(bit0-bit5);
    heartrate5_writeReg(0x23, 0x104218); //DYNAMIC1(bit20)=1; ILED_2X(bit17)=0; DYNAMIC2(bit14)=1; OSC_ENABLE(bit9)=1; DYNAMIC3(bit4) =1; DYNAMIC4(bit3)=1; PDNRX(bit1)=0; PDNAFE(bit0)=0; 
    heartrate5_writeReg(0x29, 0x000000); // ENABLE_CLKOUT(bit9)=0; CLKDIV_CLKOUT(bit1-bit4) =0;
    heartrate5_writeReg(0x31, 0x000000); //PD_DISCONNECT(bit10)=0; ENABLE_INPUT_SHORT(bit5)=0; CLKDIV_EXTMODE(bit0-bit2)=0;
    heartrate5_writeReg(0x32, 0x00155F); //PDNCYCLESTC(bit0-bit15) = 0x00155F;
    heartrate5_writeReg(0x33, 0x00991E); //PDNCYCLEENDC(bit0-bit15) = 0x00991E;
    heartrate5_writeReg(0x34, 0x000000); //PROG_TG_STC(bit0-bit15) = 0;
    heartrate5_writeReg(0x35, 0x000000); //PROG_TG_ENDC(bit0-bit15) = 0;
    heartrate5_writeReg(0x36, 0x000190); //LED3LEDSTC(bit0-bit15) =0x000190;
    heartrate5_writeReg(0x37, 0x00031F); //LED3LEDENC(bit0-bit15) =0x00031F;
    heartrate5_writeReg(0x39, 0x000000); //CLKDIV_PRF(bit0-bit2) = 0;
    heartrate5_writeReg(0x3A, 0x000000); // POL_OFFDAC_LED2(bit19)=0; I_OFFDAC_LED2(bit15-bit18)=0; POL_OFFDAC_AMB1(bit14)=0; I_OFFDAC_AMB1(bit10-bit13)=0; POL_OFFDAC_LED1(bit9)=0; I_OFFDAC_LED1(bit5-bit8)=0; POL_OFFDAC_AMB2\POL_OFFDAC_LED3(bit4)=0; O_OFFDAC_AMB\IOFFDAC_LED3(bit0-bit3)=0;
} 
void heartrate5_hwReset()
{
    hal_gpio_rstSet(0);
    Delay_50us();
    hal_gpio_rstSet(1);
}
void heartrate5_swReset()
{
    heartrate5_writeReg(HR5_REG0H, 0x000008);
}
uint32_t heartrate5_getLed2val( void )
{
    uint32_t regDat;
    regDat=heartrate5_readReg(0x2A);
    return regDat;
}
uint32_t heartrate5_getAled2val_led3val( void )
{
    uint32_t regDat;
    regDat = heartrate5_readReg(0x2B);
    return regDat;
}
uint32_t heartrate5_getLed1val( void )
{
    uint32_t regDat;
    regDat = heartrate5_readReg(0x2C);
    return regDat;
}
uint32_t heartrate5_getAled1val( void ) //?
{
    uint32_t regDat;
    regDat = heartrate5_readReg(0x2D);
    return regDat;
}
uint32_t heartrate5_getLed2_aled2val( void ) //prox
{
    uint32_t regDat;
    regDat = heartrate5_readReg(0x2E);
    return regDat;
}
uint32_t heartrate5_getLed1_aled1val( void )
{
    uint32_t regDat;
    regDat = heartrate5_readReg(0x2F);
    return regDat;
}


/* ----------------------------------------------------------- IMPLEMENTATION */







/* -------------------------------------------------------------------------- */
/*
  __heartrate5_driver.c

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

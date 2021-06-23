#include "nrf_drv_twi.h"

#ifdef __HEARTRATE5_DRV_I2C__
static uint8_t _slaveAddress;
#endif
const uint8_t HR5_REG0H = 0x00;
const uint8_t HR5_REG1H = 0x01;  //LED2STC
const uint8_t HR5_REG2H = 0x02;  //LED2ENDC
const uint8_t HR5_REG3H = 0x03;  //LED1LEDSTC
const uint8_t HR5_REG4H = 0x04;  //LED1LEDENDC
const uint8_t HR5_REG5H = 0x05;  //ALED2STC\LED3STC
const uint8_t HR5_REG6H = 0x06;  //ALED2ENDC\LED3ENDC
const uint8_t HR5_REG7H = 0x07;  //LED1STC
const uint8_t HR5_REG8H = 0x08;  //LED1ENDC
const uint8_t HR5_REG9H = 0x09;  //LED2LEDSTC
const uint8_t HR5_REGAH = 0x0A;  //LED2LEDENDC
const uint8_t HR5_REGBH = 0x0B;  //ALED1STC
const uint8_t HR5_REGCH = 0x0C;  //ALED1ENDC
const uint8_t HR5_REGDH = 0x0D;  //LED2CONVST
const uint8_t HR5_REGEH = 0x0E;  //LED2CONVEND
const uint8_t HR5_REGFH = 0x0F;  //ALED2CONVST\LED3CONVST
const uint8_t HR5_REG10H = 0x10; //ALED2CONVEND\LED3CONVEND
const uint8_t HR5_REG11H = 0x11; //LED1CONVST
const uint8_t HR5_REG12H = 0x12; //LED1CONVEND
const uint8_t HR5_REG13H = 0x13; //ALED1CONVST
const uint8_t HR5_REG14H = 0x14; //ALED1CONVEND
const uint8_t HR5_REG15H = 0x15; //ADCRSTSTCT0
const uint8_t HR5_REG16H = 0x16; //ADCRSTENDCT0
const uint8_t HR5_REG17H = 0x17; //ADCRSTSTCT1
const uint8_t HR5_REG18H = 0x18; //ADCRSTENDCT1
const uint8_t HR5_REG19H = 0x19; //ADCRSTSTCT2
const uint8_t HR5_REG1AH = 0x1A; //ADCRSTENDCT2
const uint8_t HR5_REG1BH = 0x1B; //ADCRSTSTCT3
const uint8_t HR5_REG1CH = 0x1C; //ADCRSTENDCT3
const uint8_t HR5_REG1DH = 0x1D; //PRPCT
const uint8_t HR5_REG1EH = 0x1E; //timer controll
const uint8_t HR5_REG20H = 0x20;
const uint8_t HR5_REG21H = 0x21;
const uint8_t HR5_REG22H = 0x22;
const uint8_t HR5_REG23H = 0x23;
const uint8_t HR5_REG29H = 0x29;

const uint8_t HR5_REG2AH = 0x2A; //LED2VAL
const uint8_t HR5_REG2BH = 0x2B; //ALED2VAL\ALSED3VAL
const uint8_t HR5_REG2CH = 0x2C; //LED1VAL
const uint8_t HR5_REG2DH = 0x2D; //ALED1VAL
const uint8_t HR5_REG2EH = 0x2E; //LED2-ALED2VAL
const uint8_t HR5_REG2FH = 0x2F; //LED1-ALED1VAL

const uint8_t HR5_REG31H = 0x31;
const uint8_t HR5_REG32H = 0x32;
const uint8_t HR5_REG33H = 0x33;
const uint8_t HR5_REG34H = 0x34;
const uint8_t HR5_REG35H = 0x35;
const uint8_t HR5_REG36H = 0x36;
const uint8_t HR5_REG37H = 0x37;
const uint8_t HR5_REG39H = 0x39;
const uint8_t HR5_REG3AH = 0x3A;
const uint8_t HR5_REG3DH = 0x3D;
const uint8_t HR5_REG3FH = 0x3F;
const uint8_t HR5_REG40H = 0x40;

const uint8_t HR5_ADDR = 0x58;

void heartrate5_write(uint8_t regAddr, uint32_t wData, nrf_drv_twi_t const *p_instance) {
  uint8_t hr_storage[5];
  hr_storage[0] = regAddr;
  hr_storage[1] = wData >> 16;
  hr_storage[2] = wData >> 8;
  hr_storage[3] = wData;

  nrf_drv_twi_tx(p_instance, regAddr, wData, sizeof(wData), false);
}

uint32_t heartrate5_readReg(uint8_t regAddr, nrf_drv_twi_t const *p_instance) {
  uint8_t hr_storage[5];
  uint32_t returnValue = 0;
  hr_storage[0] = regAddr;

  //hal_i2cStart();
  //heartrate5_write(_slaveAddress,hr_storage,1,END_MODE_RESTART);
  nrf_drv_twi_tx(p_instance, regAddr, hr_storage, sizeof(hr_storage), false);
  returnValue = nrf_drv_twi_rx(p_instance, regAddr, hr_storage, sizeof(hr_storage));
  //hal_i2cRead(_slaveAddress,hr_storage,3,END_MODE_STOP);

  returnValue = hr_storage[0];
  returnValue = returnValue << 16;
  returnValue |= hr_storage[1] << 8;
  returnValue |= hr_storage[2];
  return returnValue;
}
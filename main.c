#include "app_error.h"
#include "app_util_platform.h"
#include "boards.h"
#include "nrf_delay.h"
#include "nrf_drv_twi.h"
#include <stdio.h>

//#include "nrf_drv_timer.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_drv_twi.h"
//#include "hr5_drv.h"

#ifdef __HEARTRATE5_DRV_I2C__
static uint8_t _slaveAddress;
#endif
const uint8_t HR5_REG0H = 0x00;  //Control0
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

/* TWI instance ID. */
#if TWI0_ENABLED
#define TWI_INSTANCE_ID 0
#elif TWI1_ENABLED
#define TWI_INSTANCE_ID 1
#endif

/* Number of possible TWI addresses. */
#define TWI_ADDRESSES 127

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

//const nrf_drv_timer_t TIMER = NRF_DRV_TIMER_INSTANCE(0);

uint32_t get_rtc_counter(void) {
  return NRF_RTC1->COUNTER;
}

void heartrate5_writeReg(uint8_t regAddr, uint32_t wData) {
  uint8_t hr_storage[5] = {0};
  hr_storage[0] = regAddr;
  hr_storage[1] = wData >> 16;
  hr_storage[2] = wData >> 8;
  hr_storage[3] = wData;

  nrf_drv_twi_tx(&m_twi, HR5_ADDR, hr_storage, sizeof(hr_storage), false);
}

uint32_t heartrate5_readReg(uint8_t regAddr) {
  uint8_t hr_storage[3];
  uint32_t returnValue = 0;

  if (NRF_SUCCESS == nrf_drv_twi_tx(&m_twi, HR5_ADDR, &regAddr, 1, true)) {
    // Read the data back, 3 bytes
    if (NRF_SUCCESS == nrf_drv_twi_rx(&m_twi, HR5_ADDR, hr_storage, sizeof(hr_storage))) {
      returnValue = hr_storage[0];
      returnValue = (returnValue << 8) | hr_storage[1];
      returnValue = (returnValue << 8) | hr_storage[2];
    }
  }
  return returnValue;
}

void heartrate5_hwReset() {
  nrf_gpio_pin_clear(BSP_QSPI_IO0_PIN);
  nrf_delay_ms(50);
  nrf_gpio_pin_set(BSP_QSPI_IO0_PIN);
}

void delay10ms() {
  nrf_delay_ms(10);
}

uint8_t perform_hw_start_end(uint8_t regStart, uint8_t regEnd, uint16_t start, uint16_t end) {
  if (start > 65535 || end > 65535)
    return -1;

  heartrate5_writeReg(regStart, (uint32_t)start);
  heartrate5_writeReg(regEnd, (uint32_t)end);

  return 0;
}

void heartrate5_swReset() {
  heartrate5_writeReg(HR5_REG0H, 0x000008);
}
uint32_t heartrate5_getLed2val(void) {
  uint32_t regDat;
  regDat = heartrate5_readReg(0x2A);
  return regDat;
}
uint32_t heartrate5_getAled2val_led3val(void) {
  uint32_t regDat;
  regDat = heartrate5_readReg(0x2B);
  return regDat;
}
uint32_t heartrate5_getLed1val(void) {
  uint32_t regDat;
  regDat = heartrate5_readReg(0x2C);
  return regDat;
}
uint32_t heartrate5_getAled1val(void) //?
{
  uint32_t regDat;
  regDat = heartrate5_readReg(0x2D);
  return regDat;
}
uint32_t heartrate5_getLed2_aled2val(void) //prox
{
  uint32_t regDat;
  regDat = heartrate5_readReg(0x2E);
  return regDat;
}
uint32_t heartrate5_getLed1_aled1val(void) {
  uint32_t regDat;
  regDat = heartrate5_readReg(0x2F);
  return regDat;
}

uint8_t set_prpct_count(uint16_t count) {
  uint8_t reg = HR5_REG1DH;
  uint8_t temp[3] = {0};

  if (count > 65535)
    return -1;

  temp[1] = count >> 8;
  temp[2] = (uint8_t)count;

  heartrate5_writeReg(reg, (uint32_t)count);

  return 0;
}

uint8_t hr5_set_led_currents(uint8_t led1_current, uint8_t led2_current,
    uint8_t led3_current) {
  uint8_t temp[5] = {0};
  unsigned long currents = 0;

  if (led1_current > 63 ||
      led2_current > 63 ||
      led3_current > 63)
    return -1;

  currents |= (led3_current << 12);
  currents |= (led2_current << 6);
  currents |= led1_current;

  temp[4] |= currents;
  temp[3] |= currents >> 8;
  temp[2] |= currents >> 16;
  temp[0] = HR5_REG22H;

  //heartrate5_writeReg(HR5_REG22H, (uint32_t)temp);
  nrf_drv_twi_tx(&m_twi, HR5_ADDR, temp, sizeof(temp), false);

  return 0;
}

uint8_t hr5_set_timer_and_average_num(bool enable, uint8_t av_num) {
  uint8_t reg = HR5_REG1EH;
  uint8_t temp[3] = {0};

  if (av_num > 15 || av_num < 0)
    return -1;

  if (enable) {
    temp[1] |= (1 << 0);
    temp[2] |= (av_num << 0);
    heartrate5_writeReg(reg, (uint32_t)temp);
  } else {
    temp[2] |= (av_num << 0);
    heartrate5_writeReg(reg, (uint32_t)temp);
  }

  return 0;
}

uint8_t hr5_set_seperate_tia_gain(bool seperate, uint8_t cf_setting,
    uint8_t gain_setting) {
  uint8_t reg = HR5_REG20H;
  uint8_t temp[3] = {0};

  if (cf_setting > 7 || gain_setting > 7)
    return -1;

  if (seperate) {
    temp[1] = 8;
    temp[2] |= (cf_setting << 3);
    temp[2] |= (gain_setting << 0);
    heartrate5_writeReg(reg, (uint32_t)temp);
  } else {
    temp[2] |= (cf_setting << 3);
    temp[2] |= (gain_setting << 0);
    heartrate5_writeReg(reg, (uint32_t)temp);
  }

  return 0;
}

uint8_t hr5_set_tia_gain(bool replace, uint8_t cf_setting,
    uint8_t gain_setting) {
  uint8_t reg = HR5_REG21H;
  uint8_t temp[3] = {0};

  if (cf_setting > 7 || gain_setting > 7)
    return -1;

  if (replace) {
    temp[1] = 1;
    temp[2] |= (cf_setting << 3);
    temp[2] |= (gain_setting << 0);
    heartrate5_writeReg(reg, (uint32_t)temp);
  } else {
    temp[1] = 0;
    temp[2] |= (cf_setting << 3);
    temp[2] |= (gain_setting << 0);
    heartrate5_writeReg(reg, (uint32_t)temp);
  }

  return 0;
}

uint8_t hr5_set_clkout_div(bool enable, uint8_t div) {
  uint8_t reg = HR5_REG23H;
  uint8_t temp[3] = {0};

  if (div > 15)
    return -1;

  if (enable) {
    temp[1] = (1 << 1);
    temp[2] = (div << 1);
    heartrate5_writeReg(reg, (uint32_t)temp);

  } else {
    temp[2] = (div << 1);
    heartrate5_writeReg(reg, (uint32_t)temp);
  }

  return 0;
}

uint8_t hr5_set_int_clk_div(uint8_t div) {
  uint8_t reg = HR5_REG39H;
  uint8_t temp[3] = {0};

  if (div > 7)
    return -1;

  temp[2] = div;
  heartrate5_writeReg(reg, (uint32_t)temp);

  return 0;
}

#define SETTINGS 0x23  /**< Settings Address */
#define STT_DYNMC1 20  /**< 0: Transmitter not pwrd dwn 1: pwrd dwn  */
#define STT_ILED_2X 17 /**< 0: LED crrnt range 0-50 1: range 0-100   */
#define STT_DYNMC2 14  /**< 0: ADC not pwrd dwn 1: ADC pwrd dwn      */
#define STT_OSC_EN 9   /**< 0: External Clock 1: Internal Clock      */
#define STT_DYNMC3 4   /**< 0: TIA not pwrd dwn 1: TIA pwrd dwn      */
#define STT_DYNMC4 3   /**< 0: Rest of ADC ! pwrd dwn 1: Is pwrd dwn */
#define STT_PDNRX 1    /**< 0: Normal Mode 1: RX of AFE pwrd dwn     */
#define STT_PDNAFE 0   /**< 0: Normal Mode 1: Entire AFE pwrd dwn    */

typedef struct
{
  transmitter_t transmit;
  led_current_range_t curr_range;
  adc_pwer_t adc_power;
  clk_mode_t clk_mode;
  tia_pwer_t tia_power;
  rest_of_adc_t rest_of_adc;
  afe_rx_mode_t afe_rx_mode;
  afe_mode_t afe_mode;
} dynamic_modes_t;

uint8_t hr5_set_dynamic_settings(dynamic_modes_t *modes) {
  uint8_t reg = SETTINGS;
  uint8_t temp[3] = {0};
  unsigned long buffer = 0;

  buffer |= (modes->transmit << STT_DYNMC1);
  buffer |= (modes->curr_range << STT_ILED_2X);
  buffer |= (modes->adc_power << STT_DYNMC2);
  buffer |= (modes->clk_mode << STT_OSC_EN);
  buffer |= (modes->tia_power << STT_DYNMC3);
  buffer |= (modes->rest_of_adc << STT_DYNMC4);
  buffer |= (modes->afe_rx_mode << STT_PDNRX);
  buffer |= (modes->afe_mode << STT_PDNAFE);

  temp[2] |= buffer;
  temp[1] |= buffer >> 8;
  temp[0] |= buffer >> 16;

  heartrate5_writeReg(reg, temp);

  return 0;
}

void heartrate5_init() {
  //perform_hw_start_end(HR5_REG9H, HR5_REGAH, 0, 399);  //LED2
  dynamic_modes_t dynamic_modes;

  dynamic_modes.transmit = trans_dis;          //Transmitter disabled
  dynamic_modes.curr_range = led_double;       //LED range 0 - 100
  dynamic_modes.adc_power = adc_on;            //ADC on
  dynamic_modes.clk_mode = osc_mode;           //Use internal Oscillator
  dynamic_modes.tia_power = tia_off;           //TIA off
  dynamic_modes.rest_of_adc = rest_of_adc_off; //Rest of ADC off
  dynamic_modes.afe_rx_mode = afe_rx_normal;   //Normal Receiving on AFE
  dynamic_modes.afe_mode = afe_normal;         //Normal AFE functionality

  //uint8_t temp[3] = {0};

  //temp[2] |= (1 << 1);
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
  //hr5_set_dynamic_settings(dynamic_modes);
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

  //perform_hw_start_end(HR5_REG9H, HR5_REGAH, 0, 399);  //LED2
  //perform_hw_start_end(HR5_REG1H, HR5_REG2H, 80, 399); //LED2 SAMPLE

  //perform_hw_start_end(HR5_REG15H, HR5_REG16H, 401, 407); //ADC Reset 0

  //perform_hw_start_end(HR5_REGDH, HR5_REGEH, 408, 1467); //LED 2 convert

  //perform_hw_start_end(HR5_REG36H, HR5_REG37H, 400, 799); //LED3
  //perform_hw_start_end(HR5_REG5H, HR5_REG6H, 480, 799);   //LED3 SAMPLE

  //perform_hw_start_end(HR5_REG17H, HR5_REG18H, 1469, 1475); //ADC RESET 1

  //perform_hw_start_end(HR5_REGFH, HR5_REG10H, 1476, 2535); //LED3 convert

  //perform_hw_start_end(HR5_REG3H, HR5_REG4H, 800, 1199); //LED1
  //perform_hw_start_end(HR5_REG7H, HR5_REG8H, 880, 1199); //LED1 SAMPLE

  //perform_hw_start_end(HR5_REG19H, HR5_REG1AH, 2537, 2543); //ADC RESET 2

  //perform_hw_start_end(HR5_REG11H, HR5_REG12H, 2544, 3603); //LED1 CONVERT

  //perform_hw_start_end(HR5_REGBH, HR5_REGCH, 1279, 1598); //AMB1

  //perform_hw_start_end(HR5_REG1BH, HR5_REG1CH, 3605, 3611); //ADC RESET 3

  //perform_hw_start_end(HR5_REG13H, HR5_REG14H, 3612, 4671); //AMB1 CONVERT

  //perform_hw_start_end(HR5_REG32H, HR5_REG33H, 5471, 39199); //PDN cycle
  //set_prpct_count(39999);

  //hr5_set_led_currents(15, 3, 3);
  //hr5_set_timer_and_average_num(true, 3);
  //hr5_set_seperate_tia_gain(true, 0, 4);
  //hr5_set_tia_gain(false, 0, 3);
  //hr5_set_clkout_div(false, 2);
  //hr5_set_int_clk_div(1);
  //heartrate5_swReset();
  ////perform_hw_start_end(HR5_REG1DH, HR5_REG1DH, 401, 39999);  //PRPCT count
}

/**
 * @brief TWI initialization.
 */
void twi_init(void) {
  ret_code_t err_code;

  const nrf_drv_twi_config_t twi_config = {
      .scl = ARDUINO_SCL_PIN,
      .sda = ARDUINO_SDA_PIN,
      .frequency = NRF_DRV_TWI_FREQ_100K,
      .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
      .clear_bus_init = false};

  err_code = nrf_drv_twi_init(&m_twi, &twi_config, NULL, NULL);
  APP_ERROR_CHECK(err_code);
  NRF_LOG_INFO("val%x", err_code);
  nrf_drv_twi_enable(&m_twi);
}

void getReading() {
  uint32_t sensorVal;
  //sensorVal = heartrate5_getLed1val();
  //float idk = (float)sensorVal;
  delay10ms();

  float valOne = (float)heartrate5_getLed2val();
  delay10ms();

  float valTwo = (float)heartrate5_getAled2val_led3val();
  delay10ms();

  float valThree = (float)heartrate5_getLed1val();
  delay10ms();

  float valFour = (float)heartrate5_getAled1val();
  delay10ms();

  float valFive = (float)heartrate5_getLed2_aled2val();
  delay10ms();

  float valSix = (float)heartrate5_getLed1_aled1val();

  NRF_LOG_INFO("%u;%u;%u;%u;%u;", valOne, valTwo, valThree, valFour, valFive, valSix);
  NRF_LOG_FLUSH();
}

/**
 * @brief Function for main application entry.
 */
int main(void) {
  ret_code_t err_code;
  uint8_t address;
  uint8_t sample_data;
  bool detected_device = false;
  //uint32_t time_ms = 1;
  //uint32_t time_ticks;
  //uint32_t err_timer_code = NRF_SUCCESS;

  APP_ERROR_CHECK(NRF_LOG_INIT(get_rtc_counter));
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  nrf_gpio_cfg_output(BSP_QSPI_IO0_PIN);
  nrf_gpio_cfg_output(15);
  NRF_CLOCK->TASKS_HFCLKSTART = 1; //Start high frequency clock
  while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {
    //Wait for HFCLK to start
  }
  NRF_CLOCK->EVENTS_HFCLKSTARTED = 0; //Clear event

  //Configure GPIOTE to toggle pin 18
  NRF_GPIOTE->CONFIG[0] = GPIOTE_CONFIG_MODE_Task << GPIOTE_CONFIG_MODE_Pos |
                          GPIOTE_CONFIG_POLARITY_Toggle << GPIOTE_CONFIG_POLARITY_Pos |
                          15 << GPIOTE_CONFIG_PSEL_Pos |
                          GPIOTE_CONFIG_OUTINIT_Low << GPIOTE_CONFIG_OUTINIT_Pos;

  //Configure timer
  NRF_TIMER1->PRESCALER = 0;
  NRF_TIMER1->CC[0] = 1; // Adjust the output frequency by adjusting the CC.
  NRF_TIMER1->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Enabled << TIMER_SHORTS_COMPARE0_CLEAR_Pos;
  NRF_TIMER1->TASKS_START = 1;

  //Configure PPI
  NRF_PPI->CH[0].EEP = (uint32_t)&NRF_TIMER1->EVENTS_COMPARE[0];
  NRF_PPI->CH[0].TEP = (uint32_t)&NRF_GPIOTE->TASKS_OUT[0];

  NRF_PPI->CHENSET = PPI_CHENSET_CH0_Enabled << PPI_CHENSET_CH0_Pos;

  NRF_LOG_INFO("TWI scanner started.");
  NRF_LOG_FLUSH();
  twi_init();

  //nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
  //err_code = nrf_drv_timer_init(&TIMER, &timer_cfg, timer_led_event_handler);
  //APP_ERROR_CHECK(err_code);

  //HR5_ADDR
  err_code = nrf_drv_twi_rx(&m_twi, HR5_ADDR, &sample_data, sizeof(sample_data));
  NRF_LOG_INFO("Searching %x.", HR5_ADDR);
  if (err_code == NRF_SUCCESS) {
    NRF_LOG_INFO("Heart Rate 5 Click detected at address 0x%x.", HR5_ADDR);
    detected_device = true;
    heartrate5_hwReset();
    heartrate5_init();

    while (true) {
      getReading();
      nrf_delay_ms(100);
    }
  }
  NRF_LOG_FLUSH();

  if (!detected_device) {
    NRF_LOG_INFO("No device was found.");
    NRF_LOG_FLUSH();
  }

  while (true) {
  }
}

/** @} */
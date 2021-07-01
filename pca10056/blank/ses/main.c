#include "app_error.h"
#include "app_util_platform.h"
#include "boards.h"
#include "nrf_delay.h"
#include "nrf_drv_twi.h"
#include <stdio.h>

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "hr5_drv.h"
#include "nrf_drv_twi.h"

/* TWI instance ID. */
#if TWI0_ENABLED
#define TWI_INSTANCE_ID 0elif TWI1_ENABLED
#define TWI_INSTANCE_ID 1
#endif

/* Number of possible TWI addresses. */
#define TWI_ADDRESSES 127

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

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
  heartrate5_writeReg(DIAGNOSIS, 0x000008);
}
uint32_t heartrate5_getLed2val(void) {
  uint32_t regDat;
  regDat = heartrate5_readReg(LED2VAL);
  return regDat;
}
uint32_t heartrate5_getAled2val_led3val(void) {
  uint32_t regDat;
  regDat = heartrate5_readReg(LED3VAL);
  return regDat;
}
uint32_t heartrate5_getLed1val(void) {
  uint32_t regDat;
  regDat = heartrate5_readReg(LED1VAL);
  return regDat;
}
uint32_t heartrate5_getAled1val(void) //?
{
  uint32_t regDat;
  regDat = heartrate5_readReg(ALED1VAL);
  return regDat;
}
uint32_t heartrate5_getLed2_aled2val(void) //prox
{
  uint32_t regDat;
  regDat = heartrate5_readReg(LED2_ALED2VAL);
  return regDat;
}
uint32_t heartrate5_getLed1_aled1val(void) {
  uint32_t regDat;
  regDat = heartrate5_readReg(LED1_ALED1VAL);
  return regDat;
}

uint8_t set_prpct_count(uint16_t count) {
  uint8_t reg = PRPCT;
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
  uint8_t temp[4] = {0};
  unsigned long currents = 0;

  if (led1_current > 63 ||
      led2_current > 63 ||
      led3_current > 63)
    return -1;

  currents |= (led3_current << 12);
  currents |= (led2_current << 6);
  currents |= led1_current;

  temp[3] |= currents;
  temp[2] |= currents >> 8;
  temp[1] |= currents >> 16;
  //temp[3] = currents[0];
  //temp[2] = currents[1];
  //temp[3] = currents[2];
  temp[0] = LED_CONFIG;

  nrf_drv_twi_tx(&m_twi, HR5_ADDR, temp, sizeof(temp), false);

  return 0;
}

uint8_t hr5_set_timer_and_average_num(bool enable, uint8_t av_num) {
  uint8_t reg = TIM_NUMAV;
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
  uint8_t reg = TIA_GAINS2;
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
  uint8_t reg = TIA_GAINS1;
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
  uint8_t reg = CLKOUT;
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
  uint8_t reg = CLKDIV_PRF;
  uint8_t temp[3] = {0};

  if (div > 7)
    return -1;

  temp[2] = div;
  heartrate5_writeReg(reg, (uint32_t)temp);

  return 0;
}

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

  uint8_t hr_storage[5] = {0};
  hr_storage[0] = reg;
  hr_storage[1] = temp[2];
  hr_storage[2] = temp[1];
  hr_storage[3] = temp[0];

  nrf_drv_twi_tx(&m_twi, HR5_ADDR, hr_storage, sizeof(hr_storage), false);
  return 0;
}


void heartrate5_init() {
  uint32_t reg[NUM_REGISTERS][2] = {
      {0x00, 0x000000}, /*CONTROL0*/
      {0x01, 0x000050}, /*LED2STC*/
      {0x02, 0x20018F}, /*LED2ENDC*/
      {0x03, 0x000322}, /*LED1LEDSTC*/
      {0x04, 0x0004B1}, /*LED1LEDENDC*/
      {0x05, 0x0001E1}, /*ALED2STC*/
      {0x06, 0x000320}, /*ALED2ENDC*/
      {0x07, 0x000372}, /*LED1STC*/
      {0x08, 0x0004B1}, /*LED1ENDC*/
      {0x09, 0x000000}, /*LED2LEDSTC*/
      {0x0A, 0x00018F}, /*LED2LEDENDC*/
      {0x0B, 0x000503}, /*ALED1STC*/
      {0x0C, 0x000642}, /*ALED1ENDC*/
      {0x0D, 0x000195}, /*LED2CONVST*/
      {0x0E, 0x000323}, /*LED2CONVEND*/
      {0x0F, 0x000329}, /*ALED2CONVST*/
      {0x10, 0x0004B7}, /*ALED2CONVEND*/
      {0x11, 0x0004BD}, /*LED1CONVST*/
      {0x12, 0x00064B}, /*LED1CONVEND*/
      {0x13, 0x000651}, /*ALED1CONVST*/
      {0x14, 0x0007DF}, /*ALED1CONVEND*/
      {0x15, 0x000191}, /*ADCRSTSTCT0*/
      {0x16, 0x000193}, /*ADCRSTENDCT0*/
      {0x17, 0x000325}, /*ADCRSTSTCT1*/
      {0x18, 0x000327}, /*ADCRSTENDCT1*/
      {0x19, 0x0004B9}, /*ADCRSTSTCT2*/
      {0x1A, 0x0004BB}, /*ADCRSTENDCT2*/
      {0x1B, 0x00064D}, /*ADCRSTSTCT3*/
      {0x1C, 0x00064F}, /*ADCRSTENDCT3*/
      {0x1D, 0x009C3F}, /*PRPCOUNT*/
      {0x1E, 0x000103}, /*CONTROL1*/
      {0x20, 0x008005}, /*TIAGAIN*/
      {0x21, 0x000005}, /*TIA_AMB_GAIN*/
      {0x22, 0x004280}, /*LEDCNTRL*/
      {0x23, 0x124218}, /*CONTROL2*/
      {0x29, 0x000000}, /*CLKDIV1*/
      {0x2A, 0x000000}, /*LED2VAL*/
      {0x2B, 0x000000}, /*ALED2VAL*/
      {0x2C, 0x000000}, /*LED1VAL*/
      {0x2D, 0x000000}, /*ALED1VAL*/
      {0x2E, 0x000000}, /*LED2-ALED2VAL*/
      {0x2F, 0x000000}, /*LED1-ALED1VAL*/
      {0x31, 0x000020}, /*CONTROL3*/
      {0x32, 0x0008A7}, /*PDNCYCLESTC*/
      {0x33, 0x009B77}, /*PDNCYCLEENDC*/
      {0x34, 0x000000}, /*PROG_TG_STC*/
      {0x35, 0x000000}, /*PROG_TG_ENDC*/
      {0x36, 0x000191}, /*LED3LEDSTC*/
      {0x37, 0x000320}, /*LED3LEDENDC*/
      {0x39, 0x000005}, /*CLKDIV2*/
      {0x3A, 0x000000}, /*OFFDAC*/
  };

  // Do the i2c transfer
  for (uint8_t reg_index = 0; reg_index < NUM_REGISTERS; reg_index++) {
    printf("Debug: Register: 0x%lx - 0x%lx\r\n", reg[reg_index][0], reg[reg_index][1]);
    heartrate5_writeReg(reg[reg_index][0], reg[reg_index][1]);
    // If on error
    //if (!heartrate5_writeReg(reg[reg_index][0], reg[reg_index][1])) {
    //  printf("DEBUG: init_registers error!\r\n");
    //  //return false;
    //}
  }

  //perform_hw_start_end(HR5_REG13H, HR5_REG14H, 3612, 4671); //AMB1 CONVERT
  //hr5_set_dynamic_settings(&dynamic_modes);

  //perform_hw_start_end(HR5_REG32H, HR5_REG33H, 5471, 39199); //PDN cycle
  ////perform_hw_start_end(HR5_REG1DH, HR5_REG1DH, 401, 39199);  //PRPCT count
  //set_prpct_count(39999);

  //hr5_set_led_currents(50, 50, 50); //22h
  ////hr5_set_timer_and_average_num(true, 3); //1e 0x000103
  //heartrate5_writeReg(0x1E, 0x000103); // timeren set ; broj semplova od 0 do F

  ////hr5_set_seperate_tia_gain(true, 0, 4); // 20h 0x008003
  //heartrate5_writeReg(0x20, 0x008004); //ENSEPGAIN  ; TIA_CF_SEP
  ////heartrate5_writeReg(0x20, 0x008003); //ENSEPGAIN  ; TIA_CF_SEP

  ////hr5_set_tia_gain(false, 0, 3); // 21h 0x000003
  //heartrate5_writeReg(0x21, 0x000000); //PROG_TG_EN(disabled) ; TIA_CF(0); TIA_GAIN(3);

  ////hr5_set_clkout_div(false, 0); //29h 0x000000
  //heartrate5_writeReg(0x29, 0x000000); // ENABLE_CLKOUT(bit9)=0; CLKDIV_CLKOUT(bit1-bit4) =0;

  ////hr5_set_int_clk_div(0); // teoretycznie 0 39h 0x000000
  //heartrate5_writeReg(0x39, 0x000001); //CLKDIV_PRF(bit0-bit2) = 0;

  //heartrate5_swReset();
}


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
  float valOne = (float)heartrate5_getLed2val();
  float valTwo = (float)heartrate5_getAled2val_led3val();
  float valThree = (float)heartrate5_getLed1val();
  float valFour = (float)heartrate5_getAled1val();
  float valFive = (float)heartrate5_getLed2_aled2val();
  float valSix = (float)heartrate5_getLed1_aled1val();

  NRF_LOG_INFO("%u;%u;%u;%u;%u;", valOne, valTwo, valThree);
  NRF_LOG_FLUSH();
}


int main(void) {
  ret_code_t err_code;
  uint8_t address;
  uint8_t sample_data;
  bool detected_device = false;
  APP_ERROR_CHECK(NRF_LOG_INIT(get_rtc_counter));
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  nrf_gpio_cfg_output(BSP_QSPI_IO0_PIN);
  nrf_gpio_cfg_output(15);

  NRF_LOG_INFO("TWI scanner started.");
  NRF_LOG_FLUSH();
  twi_init();

  err_code = nrf_drv_twi_rx(&m_twi, HR5_ADDR, &sample_data, sizeof(sample_data));
  NRF_LOG_INFO("Searching %x.", HR5_ADDR);

  if (err_code == NRF_SUCCESS) {
    NRF_LOG_INFO("Heart Rate 5 Click detected at address 0x%x.", HR5_ADDR);
    detected_device = true;
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


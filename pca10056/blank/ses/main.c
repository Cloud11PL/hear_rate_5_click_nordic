#include "app_error.h"
#include "app_util_platform.h"
#include "boards.h"
#include "nrf_delay.h"
#include "nrf_drv_twi.h"
#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>

#include "nrf.h"
#include "nrf_drv_gpiote.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "hr5_drv.h"
#include "nrf_drv_twi.h"

/* TWI instance ID. */
#if TWI0_ENABLED
#define TWI_INSTANCE_ID 0
//elif TWI1_ENABLED
//#define TWI_INSTANCE_ID 1
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
      {0x20, 0x008015}, /*TIAGAIN*/
      //{0x20, 0x008005}, /*TIAGAIN*/
      //{0x21, 0x000001}, /*TIA_AMB_GAIN*/
      {0x21, 0x000021}, /*TIA_AMB_GAIN*/
      //{0x21, 0x000011}, /*TIA_AMB_GAIN*/
      //{0x21, 0x000011}, /*TIA_AMB_GAIN*/
      //{0x22, 0x000000}, /*LEDCNTRL*/
      //{0x22, 0x004280}, /*LEDCNTRL*/
      //{0x22, 0x004200}, /*LEDCNTRL*/
      {0x22, 0x0030C0}, /*LEDCNTRL*/
      //{0x22, 0x002080}, /*LEDCNTRL*/
      //{0x22, 0x0030C0}, /*LEDCNTRL*/
      //{0x22, 0x0040C0}, /*LEDCNTRL*/
      //{0x22, 0x004140}, /*LEDCNTRL*/
      //{0x22, 0x004140}, /*LEDCNTRL*/
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
      //{0x39, 0x000000}, /*CLKDIV2*/
      //{0x39, 0x000005}, /*CLKDIV2*/
      {0x3A, 0x000000}, /*OFFDAC*/
                        //{0x3D, 0x000028}, /*AVG*/
                        //{0x3D, 0x000022}, /*AVG*/
                        //{0x3D, 0x000004}, /*AVG*/
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
  //hr5_set_timer_and_average_num(true, 3);

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

  //NRF_LOG_INFO(";%u;%u;%u;", valOne, valTwo, valThree);
  //NRF_LOG_INFO(";%u;%u;", valTwo, valThree);
  NRF_LOG_INFO(";%u;%u;%u;%u;%u;%u;", valOne, valTwo, heartrate5_readReg(0x3F), heartrate5_readReg(0x40), valFive, valSix);
  //NRF_LOG_INFO(";%u;%u;%u;%u;%u;%u;", valOne, valTwo, valThree, valFour, valFive, valSix);
  NRF_LOG_FLUSH();
}

#define BUFFER_SIZE 25
#define READING_DELAY 40
#define min(a, b) ((a) < (b) ? (a) : (b))

void find_peaks_above_min_height(int32_t *pn_locs, int32_t *n_npks, int32_t *pn_x, int32_t n_size, int32_t n_min_height) {
  int32_t i = 1, n_width;
  *n_npks = 0;

  while (i < n_size - 1) {
    if (pn_x[i] > n_min_height && pn_x[i] > pn_x[i - 1]) { // find left edge of potential peaks
      n_width = 1;
      while (i + n_width < n_size && pn_x[i] == pn_x[i + n_width]) // find flat peaks
        n_width++;
      if (pn_x[i] > pn_x[i + n_width] && (*n_npks) < 15) { // find right edge of peaks
        pn_locs[(*n_npks)++] = i;
        // for flat peaks, peak location is left edge
        i += n_width + 1;
      } else
        i += n_width;
    } else
      i++;
  }
}

void sort_ascend(int32_t *pn_x, int32_t n_size) {
  int32_t i, j, n_temp;
  for (i = 1; i < n_size; i++) {
    n_temp = pn_x[i];
    for (j = i; j > 0 && n_temp < pn_x[j - 1]; j--)
      pn_x[j] = pn_x[j - 1];
    pn_x[j] = n_temp;
  }
}

void sort_indices_descend(int32_t *pn_x, int32_t *pn_indx, int32_t n_size) {
  int32_t i, j, n_temp;
  for (i = 1; i < n_size; i++) {
    n_temp = pn_indx[i];
    for (j = i; j > 0 && pn_x[n_temp] > pn_x[pn_indx[j - 1]]; j--)
      pn_indx[j] = pn_indx[j - 1];
    pn_indx[j] = n_temp;
  }
}

void remove_close_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_min_distance) {
  int32_t i, j, n_old_npks, n_dist;

  /* Order peaks from large to small */
  sort_indices_descend(pn_x, pn_locs, *pn_npks);

  for (i = -1; i < *pn_npks; i++) {
    n_old_npks = *pn_npks;
    *pn_npks = i + 1;
    for (j = i + 1; j < n_old_npks; j++) {
      n_dist = pn_locs[j] - (i == -1 ? -1 : pn_locs[i]); // lag-zero peak of autocorr is at index -1
      if (n_dist > n_min_distance || n_dist < -n_min_distance)
        pn_locs[(*pn_npks)++] = pn_locs[j];
    }
  }

  // Resort indices int32_to ascending order
  sort_ascend(pn_locs, *pn_npks);
};

#define FS 25

void find_peaks(int32_t *pn_locs, int32_t *n_npks, int32_t *pn_x, int32_t n_size, int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num) {
  find_peaks_above_min_height(pn_locs, n_npks, pn_x, n_size, n_min_height);
  remove_close_peaks(pn_locs, n_npks, pn_x, n_min_distance);
  *n_npks = min(*n_npks, n_max_num); // ????????????
}

bool adc_rdy = false;

int32_t ir_buffer[BUFFER_SIZE];
int32_t red_buffer[BUFFER_SIZE];

void get_hr_vals_init() {
  // get n readings (25Hz) -> 25 times per second
  // eg 100 buffer -> 4 sec
  // 4 seconds algorithm

  int32_t k, i, n_exact_ir_valley_locs_count;
  int32_t n_threshold_1, n_peaks, n_peak_interval_sum;
  int32_t an_ir_valley_locs[15];
  int32_t ir_buffer[BUFFER_SIZE];
  int32_t red_buffer[BUFFER_SIZE];
  int32_t an_x[BUFFER_SIZE];
  int32_t an_y[BUFFER_SIZE];
  static int32_t n_last_peak_interval = FS;

  for (int i = 0; i < BUFFER_SIZE; i++) {
    ir_buffer[i] = (float)heartrate5_getLed2val();
    red_buffer[i] = (float)heartrate5_getAled2val_led3val();
    nrf_delay_ms(READING_DELAY);
  }

  // DC and subtract DC from IR

  uint32_t ir_mean_val;

  ir_mean_val = 0;
  for (k = 0; k < BUFFER_SIZE; k++) {
    ir_mean_val += ir_buffer[k];
  }

  ir_mean_val = ir_mean_val / BUFFER_SIZE;

  // remove DC

  for (k = 0; k < BUFFER_SIZE; k++) {
    an_x[k] = ir_mean_val - ir_buffer[k];
  }

  // 4 pt moving avg

  for (k = 0; k < (BUFFER_SIZE - 4); k++) {
    an_x[k] = (an_x[k] + an_x[k + 1] + an_x[k + 2] + an_x[k + 3]) / (int)4;
  }

  int32_t n_spo2_calc;
  float *pn_spo2;
  int8_t *pch_spo2_valid;
  int8_t *pch_hr_valid;

  // calculate threshold

  n_threshold_1 = 0;
  for (k = 0; k < (BUFFER_SIZE - 4); k++) {
    n_threshold_1 += an_x[k];
  }

  n_threshold_1 = n_threshold_1 / (BUFFER_SIZE - 4);

  NRF_LOG_INFO("TH1 %d.", n_threshold_1);
  NRF_LOG_FLUSH();

  for (k = 0; k < 15; k++)
    an_ir_valley_locs[k] = 0;

  find_peaks(an_ir_valley_locs, &n_peaks, an_x, (BUFFER_SIZE - 4), n_threshold_1, 4, 15);
  n_peak_interval_sum = 0;

  int32_t *pn_heart_rate; // !!!!!!!!!!!!!!!!

  NRF_LOG_INFO("Peaks %d.", n_peaks);
  NRF_LOG_FLUSH();

  NRF_LOG_INFO("Interval sum %d.", n_peak_interval_sum);
  NRF_LOG_FLUSH();

  if (n_peaks >= 2) {
    for (k = 1; k < n_peaks; k++)
      n_peak_interval_sum += (an_ir_valley_locs[k] - an_ir_valley_locs[k - 1]);
    n_peak_interval_sum = n_peak_interval_sum / (n_peaks - 1);
    *pn_heart_rate = (int32_t)((FS * 60) / n_peak_interval_sum);
    *pch_hr_valid = 1;
  } else {
    *pn_heart_rate = -999; // unable to calculate because # of peaks are too small
    *pch_hr_valid = 0;
  }

  NRF_LOG_INFO("HR %x.", pn_heart_rate);
  NRF_LOG_FLUSH();

  n_exact_ir_valley_locs_count = n_peaks;

  int32_t n_ratio_average, n_i_ratio_count, n_middle_idx;
  int32_t an_ratio[5];

  n_ratio_average = 0;
  n_i_ratio_count = 0;

  for (k = 0; k < 5; k++)
    an_ratio[k] = 0;

  for (k = 0; k < n_exact_ir_valley_locs_count; k++) {
    if (an_ir_valley_locs[k] > BUFFER_SIZE) {
      *pn_spo2 = -999; // do not use SPO2 since valley loc is out of range
      *pch_spo2_valid = 0;
      return;
    }
  }

  int32_t n_y_ac, n_x_ac;
  int32_t n_y_dc_max, n_x_dc_max;
  int32_t n_y_dc_max_idx, n_x_dc_max_idx;
  int32_t n_nume, n_denom;

  for (k = 0; k < n_exact_ir_valley_locs_count - 1; k++) {
    n_y_dc_max = -16777216;
    n_x_dc_max = -16777216;
    if (an_ir_valley_locs[k + 1] - an_ir_valley_locs[k] > 3) {
      for (i = an_ir_valley_locs[k]; i < an_ir_valley_locs[k + 1]; i++) {
        if (an_x[i] > n_x_dc_max) {
          n_x_dc_max = an_x[i];
          n_x_dc_max_idx = i;
        }
        if (an_y[i] > n_y_dc_max) {
          n_y_dc_max = an_y[i];
          n_y_dc_max_idx = i;
        }
      }
      n_y_ac = (an_y[an_ir_valley_locs[k + 1]] - an_y[an_ir_valley_locs[k]]) * (n_y_dc_max_idx - an_ir_valley_locs[k]); //red
      n_y_ac = an_y[an_ir_valley_locs[k]] + n_y_ac / (an_ir_valley_locs[k + 1] - an_ir_valley_locs[k]);
      n_y_ac = an_y[n_y_dc_max_idx] - n_y_ac;                                                                           // subracting linear DC compoenents from raw
      n_x_ac = (an_x[an_ir_valley_locs[k + 1]] - an_x[an_ir_valley_locs[k]]) * (n_x_dc_max_idx - an_ir_valley_locs[k]); // ir
      n_x_ac = an_x[an_ir_valley_locs[k]] + n_x_ac / (an_ir_valley_locs[k + 1] - an_ir_valley_locs[k]);
      n_x_ac = an_x[n_y_dc_max_idx] - n_x_ac; // subracting linear DC compoenents from raw
      n_nume = (n_y_ac * n_x_dc_max) >> 7;    //prepare X100 to preserve floating value
      n_denom = (n_x_ac * n_y_dc_max) >> 7;
      if (n_denom > 0 && n_i_ratio_count < 5 && n_nume != 0) {
        an_ratio[n_i_ratio_count] = (n_nume * 100) / n_denom; //formular is ( n_y_ac *n_x_dc_max) / ( n_x_ac *n_y_dc_max) ;
        n_i_ratio_count++;
      }
    }
  }

  sort_ascend(an_ratio, n_i_ratio_count);
  n_middle_idx = n_i_ratio_count / 2;

  long uch_spo2_table[15];

  if (n_middle_idx > 1)
    n_ratio_average = (an_ratio[n_middle_idx - 1] + an_ratio[n_middle_idx]) / 2; // use median
  else
    n_ratio_average = an_ratio[n_middle_idx];

  if (n_ratio_average > 2 && n_ratio_average < 184) {
    n_spo2_calc = uch_spo2_table[n_ratio_average];
    *pn_spo2 = uch_spo2_table[n_ratio_average];
    *pch_spo2_valid = 1; //  float_SPO2 =  -45.060*n_ratio_average* n_ratio_average/10000 + 30.354 *n_ratio_average/100 + 94.845 ;  // for comparison with table
  } else {
    *pn_spo2 = -999; // do not use SPO2 since signal an_ratio is out of range
    *pch_spo2_valid = 0;
  }

  NRF_LOG_INFO("Values %x.", pn_spo2);
  NRF_LOG_FLUSH();
  adc_rdy = false;
  // get output values
}

#define PIN_IN ARDUINO_12_PIN

#ifdef BSP_LED_0
#define PIN_OUT BSP_LED_0
#endif
#ifndef PIN_OUT
#error "Please indicate output pin"
#endif

void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
  //nrf_drv_gpiote_out_toggle(PIN_OUT);
  nrf_drv_gpiote_out_toggle(PIN_OUT);

  switch (action) {
  case NRF_GPIOTE_POLARITY_HITOLO:
    adc_rdy = true;
  default:
    break;
  }

  //adc_rdy = true;
  //NRF_LOG_INFO("Interrupted by adcrdy");
  //NRF_LOG_FLUSH();
  //switch(action) {
  //  case GPIOTE_CONFIG_POLARITY_Toggle:
  //    nrf_drv_gpiote_out_toggle(PIN_OUT);
  //    adc_rdy = true;
  //    break;
  //  default:
  //    break;
  //}
  //NRF_LOG_INFO("Gpio init XS");
  //NRF_LOG_FLUSH();
}

static void gpio_init(void) {
  ret_code_t err_code;

  err_code = nrf_drv_gpiote_init();
  APP_ERROR_CHECK(err_code);

  nrf_gpio_cfg_output(PIN_OUT);
  nrf_gpio_pin_set(PIN_OUT);

  nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
  in_config.pull = NRF_GPIO_PIN_PULLUP;

  err_code = nrf_drv_gpiote_in_init(PIN_IN, &in_config, in_pin_handler);
  APP_ERROR_CHECK(err_code);

  nrf_drv_gpiote_in_event_enable(PIN_IN, true);

  NRF_LOG_INFO("Gpio init");
  NRF_LOG_FLUSH();
}

#define WIDE_BUFFER_SIZE 50

uint32_t IR_BUFFER[WIDE_BUFFER_SIZE];
uint32_t RED_BUFFER[WIDE_BUFFER_SIZE];
uint32_t BUFFER_INDEX = 0;
uint32_t HR_INDEX = 0;
uint32_t NUMBER_OF_PEAKS = 0;
uint32_t avgHR = 0;
uint32_t peaksIdx[WIDE_BUFFER_SIZE];

void resetBuffers(void) {
  memset(IR_BUFFER, 0, sizeof(IR_BUFFER));
  memset(RED_BUFFER, 0, sizeof(RED_BUFFER));
  BUFFER_INDEX = 0;
}

void addLEDvalsToBuffers(void) {
  IR_BUFFER[BUFFER_INDEX] = heartrate5_getLed2val();
  RED_BUFFER[BUFFER_INDEX] = heartrate5_getAled2val_led3val();
}

int main(void) {
  ret_code_t err_code;
  uint8_t address;
  uint8_t sample_data;
  bool detected_device = false;
  APP_ERROR_CHECK(NRF_LOG_INIT(get_rtc_counter));
  //uint32_t err_code
  //err_code = NRF_LOG_INIT();
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  nrf_gpio_cfg_output(BSP_QSPI_IO0_PIN);
  nrf_gpio_cfg_output(15);
  gpio_init();
  nrf_delay_ms(500);

  NRF_LOG_INFO("TWI scanner started.");
  NRF_LOG_FLUSH();
  twi_init();

  err_code = nrf_drv_twi_rx(&m_twi, HR5_ADDR, &sample_data, sizeof(sample_data));
  NRF_LOG_INFO("Searching %x.", HR5_ADDR);

  if (err_code == NRF_SUCCESS) {
    NRF_LOG_INFO("Heart Rate 5 Click detected at address 0x%x.", HR5_ADDR);
    NRF_LOG_FLUSH();
    detected_device = true;
    heartrate5_init();
    nrf_delay_ms(500);

    while (true) {
      // 1. Everytime adc is rdy add vals at specific interval to buffer

      // 2. Check if buffer is filled

      // if filled
      // do algorithm for vals
      // if not
      // return

      nrf_delay_ms(1);

      if (adc_rdy) {
        if (BUFFER_INDEX <= WIDE_BUFFER_SIZE) {
          addLEDvalsToBuffers();
        }

        if (BUFFER_INDEX == WIDE_BUFFER_SIZE) {
          // DO ALGORITHM
          //for (int i = 0; i < BUFFER_SIZE; i++) {
          //  NRF_LOG_INFO("%d;%d", RED_BUFFER[i], IR_BUFFER[i]);
          //  NRF_LOG_FLUSH();
          //}

          // TO MOVE
          uint32_t windowSize = 4;
          uint32_t windowIndexRight = 0;
          uint32_t windowIndexLeft = 0;
          
          uint32_t tempWidth = 0;
          uint32_t peaksWidth[WIDE_BUFFER_SIZE];
          uint32_t peaksNumber = 0;
          uint32_t rightScore = 0;
          uint32_t leftScore = 0;

          for (int i = 0; i <= WIDE_BUFFER_SIZE; i++) {
            rightScore = 0;
            leftScore = 0;

            for (int k = 0; k <= windowSize; k++) {
              windowIndexRight = i + k;
              windowIndexLeft = i - k;

              if (windowIndexRight <= WIDE_BUFFER_SIZE) {
                if (RED_BUFFER[i] > RED_BUFFER[windowIndexRight]) {
                  // possible peak if true for window
                  rightScore += 1;
                }
              }

              if (windowIndexLeft > 0) {
                if (RED_BUFFER[i] > RED_BUFFER[windowIndexLeft]) {
                  // possible peak if true for window
                  leftScore += 1;
                }
              }
            }

            if (rightScore == windowSize && leftScore == windowSize) {
              // peak detected
              peaksIdx[i] = i;
              peaksNumber += 1;
            }
          }

          uint32_t calculatedHR = peaksNumber;
          //uint32_t calculatedHR = peaksNumber * 60 / 0.032 * WIDE_BUFFER_SIZE;

          for (int i = 0; i < WIDE_BUFFER_SIZE; i++) {
            NRF_LOG_INFO("%d;%d", RED_BUFFER[i], IR_BUFFER[i]);
            NRF_LOG_FLUSH();
          }

          NRF_LOG_INFO("Number of Peaks %d", peaksNumber);
          NRF_LOG_INFO("Number of calculatedHR %d", calculatedHR);

          avgHR += peaksNumber;
          
          if (HR_INDEX > 150) {
            HR_INDEX = 0;

            //for (int k = 0; k < WIDE_BUFFER_SIZE; k++) {
            //  if (peaksIdx[k] > 0) {
            //    avgHR += 1;
            //  }
            //}
            avgHR = avgHR;
            NRF_LOG_INFO("Number of AVG LONG HR %d", avgHR);
            memset(peaksIdx, 0, sizeof(peaksIdx));
            //avgHr 
            avgHR = 0;
          }
  
          NRF_LOG_FLUSH();
        }

        if (BUFFER_INDEX == WIDE_BUFFER_SIZE || BUFFER_INDEX > WIDE_BUFFER_SIZE) {
          resetBuffers();
        }

        adc_rdy = false;
        BUFFER_INDEX += 1;
        HR_INDEX += 1;
      }

      //for (int i = 0; i < BUFFER_SIZE; i++) {
      //  NRF_LOG_INFO("Buffer %d:%d", i, IR_BUFFER[i]);
      //  NRF_LOG_FLUSH();
      //}
      //NRF_LOG_INFO("adc redi");
      //NRF_LOG_FLUSH();
      //getReading();
      //get_hr_vals_init();
      //nrf_delay_ms(40);

      //if (adc_rdy == true) {
      //  //getReading();
      //  get_hr_vals_init();
      //  //nrf_delay_ms(100);
      //  //NRF_LOG_INFO("Adc rdy is trueee");
      //  //NRF_LOG_FLUSH();
      //  //adc_rdy = false;
      //}
      //get_hr_vals_init();
      //getReading();
      //nrf_delay_ms(1);

      //if (!adc_rdy) {
      //  //NRF_LOG_INFO("ADC READY FALSEEE")
      //  //NRF_LOG_FLUSH();

      //}

      //nrf_delay_ms(1000);
      //get_hr_vals_init();
      //float valOne = heartrate5_getLed2val();
      //float valTwo = (float)heartrate5_getAled2val_led3val();
      //float valThree = (float)heartrate5_getLed1val();
      //float valFour = (float)heartrate5_getAled1val();
      //float valFive = (float)heartrate5_getLed2_aled2val();
      //float valSix = (float)heartrate5_getLed1_aled1val();

      //NRF_LOG_INFO(";%u;%u;%u;", valOne, valTwo, valThree);
      //NRF_LOG_INFO(";%u;%u;", valTwo, valThree);
      //NRF_LOG_INFO(";%u;%u;%u;%u;%u;%u;", valOne, valTwo, valThree, valFour, valFive, valSix);
      //NRF_LOG_INFO("%" PRIu32 "\n", heartrate5_getLed2val())
      //NRF_LOG_FLUSH();
      //nrf_delay_ms(40);
    }
  }

  if (!detected_device) {
    NRF_LOG_INFO("No device was found.");
    NRF_LOG_FLUSH();
  }

  //while (true) {
  //  NRF_LOG_INFO("LOL.");
  //  NRF_LOG_FLUSH();
  //  nrf_delay_ms(1000);
  //}
}
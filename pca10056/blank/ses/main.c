#include "app_error.h"
#include "app_util_platform.h"
#include "boards.h"
#include "nrf_delay.h"
#include "nrf_drv_timer.h"
#include "nrf_drv_twi.h"
#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "nrf.h"
#include "nrf_drv_gpiote.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "hr5_drv.h"
#include "nrf_drv_twi.h"

#include "app_timer.h"
#include "ble.h"
#include "ble_advdata.h"
#include "ble_advertising.h"
#include "ble_bas.h"
#include "ble_conn_params.h"
#include "ble_conn_state.h"
#include "ble_dis.h"
#include "ble_err.h"
#include "ble_hci.h"
#include "ble_hrs.h"
#include "ble_srv_common.h"
#include "bsp_btn_ble.h"
#include "fds.h"
#include "nordic_common.h"
#include "nrf_ble_gatt.h"
#include "nrf_ble_lesc.h"
#include "nrf_ble_qwr.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "nrf_sdh_soc.h"
#include "peer_manager.h"
#include "peer_manager_handler.h"
#include "sensorsim.h"
#include "ble_types.h"

/* TWI instance ID. */
#if TWI0_ENABLED
#define TWI_INSTANCE_ID 0
//elif TWI1_ENABLED
//#define TWI_INSTANCE_ID 1
#endif

/* Number of possible TWI addresses. */
#define TWI_ADDRESSES 127

#define DEVICE_NAME "Heart Rate SpO2 Device"
#define MANUFACTURER_NAME "UwU"
#define APP_ADV_INTERVAL 300

#define APP_ADV_DURATION 18000

#define APP_BLE_CONN_CFG_TAG 1
#define APP_BLE_OBSERVER_PRIO 3 /**< Application's BLE observer priority. You shouldn't need to modify this value. */

#define LESC_DEBUG_MODE 0 /**< Set to 1 to use LESC debug keys, allows you to use a sniffer to inspect traffic. */

#define SEC_PARAM_BOND 1                               /**< Perform bonding. */
#define SEC_PARAM_MITM 0                               /**< Man In The Middle protection not required. */
#define SEC_PARAM_LESC 1                               /**< LE Secure Connections enabled. */
#define SEC_PARAM_KEYPRESS 0                           /**< Keypress notifications not enabled. */
#define SEC_PARAM_IO_CAPABILITIES BLE_GAP_IO_CAPS_NONE /**< No I/O capabilities. */
#define SEC_PARAM_OOB 0                                /**< Out Of Band data not available. */
#define SEC_PARAM_MIN_KEY_SIZE 7                       /**< Minimum encryption key size. */
#define SEC_PARAM_MAX_KEY_SIZE 16                      /**< Maximum encryption key size. */

#define MIN_CONN_INTERVAL MSEC_TO_UNITS(400, UNIT_1_25_MS) /**< Minimum acceptable connection interval (0.4 seconds). */
#define MAX_CONN_INTERVAL MSEC_TO_UNITS(650, UNIT_1_25_MS) /**< Maximum acceptable connection interval (0.65 second). */
#define SLAVE_LATENCY 0                                    /**< Slave latency. */
#define CONN_SUP_TIMEOUT MSEC_TO_UNITS(4000, UNIT_10_MS)   /**< Connection supervisory timeout (4 seconds). */

#define FIRST_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(5000) /**< Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds). */
#define NEXT_CONN_PARAMS_UPDATE_DELAY APP_TIMER_TICKS(30000) /**< Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds). */
#define MAX_CONN_PARAMS_UPDATE_COUNT 3                       /**< Number of attempts before giving up the connection parameter negotiation. */

static uint16_t m_conn_handle = BLE_CONN_HANDLE_INVALID; /**< Handle of the current connection. */

NRF_BLE_QWR_DEF(m_qwr);             /**< Context for the Queued Write module.*/
NRF_BLE_GATT_DEF(m_gatt);           /**< GATT module instance. */
BLE_ADVERTISING_DEF(m_advertising); /**< Advertising module instance. */

static ble_uuid_t m_adv_uuids[] = /**< Universally unique service identifiers. */
    {
        {BLE_UUID_PLX_SERVICE, BLE_UUID_TYPE_BLE},
        {BLE_UUID_BATTERY_SERVICE, BLE_UUID_TYPE_BLE},
        {BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}};

/* TWI instance. */
static const nrf_drv_twi_t m_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);
const nrf_drv_timer_t TIMER_LED = NRF_DRV_TIMER_INSTANCE(0);

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
#define PIN_OUT BSP_LED_2

//#ifdef BSP_LED_0
//#define PIN_OUT BSP_LED_0
//#endif
//#ifndef PIN_OUT
//#error "Please indicate output pin"
//#endif

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

/**@brief Function for handling BLE events.
 *
 * @param[in]   p_ble_evt   Bluetooth stack event.
 * @param[in]   p_context   Unused.
 */
static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context) {
  ret_code_t err_code;

  switch (p_ble_evt->header.evt_id) {
  case BLE_GAP_EVT_CONNECTED:
    NRF_LOG_INFO("Connected.");
    err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
    APP_ERROR_CHECK(err_code);
    m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_GAP_EVT_DISCONNECTED:
    NRF_LOG_INFO("Disconnected, reason %d.",
        p_ble_evt->evt.gap_evt.params.disconnected.reason);
    m_conn_handle = BLE_CONN_HANDLE_INVALID;
    break;

  case BLE_GAP_EVT_PHY_UPDATE_REQUEST: {
    NRF_LOG_DEBUG("PHY update request.");
    ble_gap_phys_t const phys =
        {
            .rx_phys = BLE_GAP_PHY_AUTO,
            .tx_phys = BLE_GAP_PHY_AUTO,
        };
    err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
    APP_ERROR_CHECK(err_code);
  } break;

  case BLE_GATTC_EVT_TIMEOUT:
    // Disconnect on GATT Client timeout event.
    NRF_LOG_DEBUG("GATT Client Timeout.");
    err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_GATTS_EVT_TIMEOUT:
    // Disconnect on GATT Server timeout event.
    NRF_LOG_DEBUG("GATT Server Timeout.");
    err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
        BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
    APP_ERROR_CHECK(err_code);
    break;

  case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
    NRF_LOG_DEBUG("BLE_GAP_EVT_SEC_PARAMS_REQUEST");
    break;

  case BLE_GAP_EVT_AUTH_KEY_REQUEST:
    NRF_LOG_INFO("BLE_GAP_EVT_AUTH_KEY_REQUEST");
    break;

  case BLE_GAP_EVT_LESC_DHKEY_REQUEST:
    NRF_LOG_INFO("BLE_GAP_EVT_LESC_DHKEY_REQUEST");
    break;

  case BLE_GAP_EVT_AUTH_STATUS:
    NRF_LOG_INFO("BLE_GAP_EVT_AUTH_STATUS: status=0x%x bond=0x%x lv4: %d kdist_own:0x%x kdist_peer:0x%x",
        p_ble_evt->evt.gap_evt.params.auth_status.auth_status,
        p_ble_evt->evt.gap_evt.params.auth_status.bonded,
        p_ble_evt->evt.gap_evt.params.auth_status.sm1_levels.lv4,
        *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_own),
        *((uint8_t *)&p_ble_evt->evt.gap_evt.params.auth_status.kdist_peer));
    break;

  default:
    // No implementation needed.
    break;
  }
}

/**@brief GATT module event handler.
 */
static void gatt_evt_handler(nrf_ble_gatt_t *p_gatt, nrf_ble_gatt_evt_t const *p_evt) {
  if (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED) {
    NRF_LOG_INFO("GATT ATT MTU on connection 0x%x changed to %d.",
        p_evt->conn_handle,
        p_evt->params.att_mtu_effective);
  }

  //ble_hrs_on_gatt_evt(&m_hrs, p_evt);
}

static void ble_stack_init(void) {
  ret_code_t err_code;

  err_code = nrf_sdh_enable_request();
  APP_ERROR_CHECK(err_code);

  // Configure the BLE stack using the default settings.
  // Fetch the start address of the application RAM.
  uint32_t ram_start = 0;
  err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
  APP_ERROR_CHECK(err_code);

  // Enable BLE stack.
  err_code = nrf_sdh_ble_enable(&ram_start);
  APP_ERROR_CHECK(err_code);

  // Register a handler for BLE events.
  NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_evt_handler, NULL);
  //NRF_LOG_FLUSH();
}

/**@brief Function for initializing the GATT module.
 */
static void gatt_init(void) {
  ret_code_t err_code;

  err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
  APP_ERROR_CHECK(err_code);

  //err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, 64);
  //APP_ERROR_CHECK(err_code);
}

///**@brief Function for putting the chip into sleep mode.
// *
// * @note This function will not return.
// */
//static void sleep_mode_enter(void)
//{
//    ret_code_t err_code;

//    err_code = bsp_indication_set(BSP_INDICATE_IDLE);
//    APP_ERROR_CHECK(err_code);

//    // Prepare wakeup buttons.
//    err_code = bsp_btn_ble_sleep_mode_prepare();
//    APP_ERROR_CHECK(err_code);

//    // Go to system-off mode (this function will not return; wakeup will cause a reset).
//    err_code = sd_power_system_off();
//    APP_ERROR_CHECK(err_code);
//}

/**@brief Function for handling advertising events.
 *
 * @details This function will be called for advertising events which are passed to the application.
 *
 * @param[in] ble_adv_evt  Advertising event.
 */
static void on_adv_evt(ble_adv_evt_t ble_adv_evt) {
  ret_code_t err_code;

  switch (ble_adv_evt) {
  case BLE_ADV_EVT_FAST:
    NRF_LOG_INFO("Fast advertising.");
    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    APP_ERROR_CHECK(err_code);
    break;

    //case BLE_ADV_EVT_IDLE:
    //  sleep_mode_enter();
    //  break;

  default:
    break;
  }
}

static void gap_params_init(void) {
  ret_code_t err_code;
  ble_gap_conn_params_t gap_conn_params;
  ble_gap_conn_sec_mode_t sec_mode;

  BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

  err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *)DEVICE_NAME, strlen(DEVICE_NAME));
  APP_ERROR_CHECK(err_code);

  memset(&gap_conn_params, 0, sizeof(gap_conn_params));

  gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
  gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
  gap_conn_params.slave_latency = SLAVE_LATENCY;
  gap_conn_params.conn_sup_timeout = CONN_SUP_TIMEOUT;

  err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for initializing the Advertising functionality.
 */
static void advertising_init(void) {
  ret_code_t err_code;
  ble_advertising_init_t init;

  memset(&init, 0, sizeof(init));

  init.advdata.name_type = BLE_ADVDATA_FULL_NAME;
  init.advdata.include_appearance = true;
  init.advdata.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE;
  init.advdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);
  init.advdata.uuids_complete.p_uuids = m_adv_uuids;

  init.config.ble_adv_fast_enabled = true;
  init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;
  init.config.ble_adv_fast_timeout = APP_ADV_DURATION;

  init.evt_handler = on_adv_evt;

  err_code = ble_advertising_init(&m_advertising, &init);
  APP_ERROR_CHECK(err_code);

  ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);
}

/**@brief Function for initializing services that will be used by the application.
 *
 * @details Initialize the Heart Rate, Battery and Device Information services.
 */
static void services_init(void) {
  //ret_code_t err_code;
  //ble_hrs_init_t hrs_init;
  //ble_bas_init_t bas_init;
  //ble_dis_init_t dis_init;
  //nrf_ble_qwr_init_t qwr_init = {0};
  //uint8_t body_sensor_location;

  //// Initialize Queued Write Module.
  //qwr_init.error_handler = nrf_qwr_error_handler;

  //err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
  //APP_ERROR_CHECK(err_code);

  //// Initialize Heart Rate Service.
  //body_sensor_location = BLE_HRS_BODY_SENSOR_LOCATION_FINGER;

  //memset(&hrs_init, 0, sizeof(hrs_init));

  //hrs_init.evt_handler = NULL;
  //hrs_init.is_sensor_contact_supported = true;
  //hrs_init.p_body_sensor_location = &body_sensor_location;

  //// Here the sec level for the Heart Rate Service can be changed/increased.
  //hrs_init.hrm_cccd_wr_sec = SEC_OPEN;
  //hrs_init.bsl_rd_sec = SEC_OPEN;

  //err_code = ble_hrs_init(&m_hrs, &hrs_init);
  //APP_ERROR_CHECK(err_code);

  //// Initialize Battery Service.
  //memset(&bas_init, 0, sizeof(bas_init));

  //bas_init.evt_handler = NULL;
  //bas_init.support_notification = true;
  //bas_init.p_report_ref = NULL;
  //bas_init.initial_batt_level = 100;

  //// Here the sec level for the Battery Service can be changed/increased.
  //bas_init.bl_rd_sec = SEC_OPEN;
  //bas_init.bl_cccd_wr_sec = SEC_OPEN;
  //bas_init.bl_report_rd_sec = SEC_OPEN;

  //err_code = ble_bas_init(&m_bas, &bas_init);
  //APP_ERROR_CHECK(err_code);

  //// Initialize Device Information Service.
  //memset(&dis_init, 0, sizeof(dis_init));

  //ble_srv_ascii_to_utf8(&dis_init.manufact_name_str, (char *)MANUFACTURER_NAME);

  //dis_init.dis_char_rd_sec = SEC_OPEN;

  //err_code = ble_dis_init(&dis_init);
  //APP_ERROR_CHECK(err_code);
}

#define WIDE_BUFFER_SIZE 50
#define WIDE_BUFFER_SIZE_CUT 46

float IR_BUFFER[WIDE_BUFFER_SIZE];
float WIDE_IR_BUFFER[WIDE_BUFFER_SIZE * 3];
float RED_BUFFER[WIDE_BUFFER_SIZE];
float WIDE_RED_BUFFER[WIDE_BUFFER_SIZE * 3];

uint32_t BUFFER_INDEX = 0;
uint32_t HR_INDEX = 0;
float NUMBER_OF_PEAKS = 0;
float avgHR = 0;
float peaksIdx[WIDE_BUFFER_SIZE];

float IRmean, REDmean, IRThreshold;
uint32_t k;
float IR_processed[WIDE_BUFFER_SIZE];
float RED_processed[WIDE_BUFFER_SIZE];
uint32_t IR_valley_locations[WIDE_BUFFER_SIZE];
float RED_AC, IR_AC;

void resetBuffers(void) {
  memset(IR_BUFFER, 0, sizeof(IR_BUFFER));
  memset(RED_BUFFER, 0, sizeof(RED_BUFFER));
  BUFFER_INDEX = 0;
}

void addLEDvalsToBuffers(void) {
  RED_BUFFER[BUFFER_INDEX] = heartrate5_getAled2val_led3val();
  IR_BUFFER[BUFFER_INDEX] = heartrate5_getLed2val();
}

void IRcalculateDCmean(void) {
  IRmean = 0;

  for (k = 0; k < WIDE_BUFFER_SIZE; k++) {
    IRmean += IR_BUFFER[k];
  }

  //NRF_LOG_INFO("IRMean before %d", IRmean);
  //NRF_LOG_FLUSH();

  IRmean = IRmean / WIDE_BUFFER_SIZE;
  //NRF_LOG_INFO("IRMean after %d", IRmean);
  //NRF_LOG_FLUSH();
}

void IRremoveDCandInvert(void) {
  for (k = 0; k < WIDE_BUFFER_SIZE; k++) {
    IR_processed[k] = (float)IRmean - (float)IR_BUFFER[k];
  }
}

void IRmoveAverage(void) {
  // why
  for (k = 0; k < WIDE_BUFFER_SIZE_CUT; k++) {
    double addedVal = IR_processed[k] + IR_processed[k + 1] + IR_processed[k + 2] + IR_processed[k + 3];
    //NRF_LOG_INFO("Added val %d", addedVal);
    //NRF_LOG_FLUSH();
    IR_processed[k] = addedVal / (int)4;
  }
}

void IRcalcThreshold(void) {
  IRThreshold = 0;

  for (k = 0; k < WIDE_BUFFER_SIZE_CUT; k++) {
    IRThreshold += IR_processed[k];
  }

  IRThreshold = IRThreshold / WIDE_BUFFER_SIZE_CUT;
}

void IR_sort_indices_descend() {
}

/**@brief Clear bond information from persistent storage.
 */
static void delete_bonds(void) {
  ret_code_t err_code;

  NRF_LOG_INFO("Erase bonds!");

  err_code = pm_peers_delete();
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for starting advertising.
 */
void advertising_start(bool erase_bonds) {
  if (erase_bonds == true) {
    delete_bonds();
    // Advertising is started by PM_EVT_PEERS_DELETE_SUCCEEDED event.
  } else {
    ret_code_t err_code;

    err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST);
    APP_ERROR_CHECK(err_code);
  }
}

/**@brief Function for handling Peer Manager events.
 *
 * @param[in] p_evt  Peer Manager event.
 */
static void pm_evt_handler(pm_evt_t const *p_evt) {
  pm_handler_on_pm_evt(p_evt);
  pm_handler_flash_clean(p_evt);

  switch (p_evt->evt_id) {
  case PM_EVT_PEERS_DELETE_SUCCEEDED:
    advertising_start(false);
    break;

  default:
    break;
  }
}

/**@brief Function for handling the Connection Parameters Module.
 *
 * @details This function will be called for all events in the Connection Parameters Module which
 *          are passed to the application.
 *          @note All this function does is to disconnect. This could have been done by simply
 *                setting the disconnect_on_fail config parameter, but instead we use the event
 *                handler mechanism to demonstrate its use.
 *
 * @param[in] p_evt  Event received from the Connection Parameters Module.
 */
static void on_conn_params_evt(ble_conn_params_evt_t *p_evt) {
  ret_code_t err_code;

  if (p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED) {
    err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
    APP_ERROR_CHECK(err_code);
  }
}

/**@brief Function for handling a Connection Parameters error.
 *
 * @param[in] nrf_error  Error code containing information about what went wrong.
 */
static void conn_params_error_handler(uint32_t nrf_error) {
  APP_ERROR_HANDLER(nrf_error);
}

/**@brief Function for initializing the Connection Parameters module.
 */
static void conn_params_init(void) {
  ret_code_t err_code;
  ble_conn_params_init_t cp_init;

  memset(&cp_init, 0, sizeof(cp_init));

  cp_init.p_conn_params = NULL;
  cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
  cp_init.next_conn_params_update_delay = NEXT_CONN_PARAMS_UPDATE_DELAY;
  cp_init.max_conn_params_update_count = MAX_CONN_PARAMS_UPDATE_COUNT;
  cp_init.start_on_notify_cccd_handle = BLE_GATT_HANDLE_INVALID;
  //cp_init.start_on_notify_cccd_handle = m_hrs.hrm_handles.cccd_handle;
  cp_init.disconnect_on_fail = true;
  //cp_init.evt_handler = NULL;
  cp_init.evt_handler = on_conn_params_evt;
  cp_init.error_handler = conn_params_error_handler;

  err_code = ble_conn_params_init(&cp_init);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for the Peer Manager initialization.
 */
static void peer_manager_init(void) {
  ble_gap_sec_params_t sec_param;
  ret_code_t err_code;

  err_code = pm_init();
  APP_ERROR_CHECK(err_code);

  memset(&sec_param, 0, sizeof(ble_gap_sec_params_t));

  // Security parameters to be used for all security procedures.
  sec_param.bond = SEC_PARAM_BOND;
  sec_param.mitm = SEC_PARAM_MITM;
  sec_param.lesc = SEC_PARAM_LESC;
  sec_param.keypress = SEC_PARAM_KEYPRESS;
  sec_param.io_caps = SEC_PARAM_IO_CAPABILITIES;
  sec_param.oob = SEC_PARAM_OOB;
  sec_param.min_key_size = SEC_PARAM_MIN_KEY_SIZE;
  sec_param.max_key_size = SEC_PARAM_MAX_KEY_SIZE;
  sec_param.kdist_own.enc = 1;
  sec_param.kdist_own.id = 1;
  sec_param.kdist_peer.enc = 1;
  sec_param.kdist_peer.id = 1;

  err_code = pm_sec_params_set(&sec_param);
  APP_ERROR_CHECK(err_code);

  err_code = pm_register(pm_evt_handler);
  APP_ERROR_CHECK(err_code);
}

/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    ret_code_t err_code;

    err_code = nrf_ble_lesc_request_handler();
    APP_ERROR_CHECK(err_code);

    if (NRF_LOG_PROCESS() == false)
    {
        //nrf_pwr_mgmt_run();
    }
}

int main(void) {
  ret_code_t err_code;
  uint8_t address;
  uint8_t sample_data;
  bool detected_device = false;
  bool erase_bonds;

  APP_ERROR_CHECK(NRF_LOG_INIT(get_rtc_counter));
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  //nrf_gpio_cfg_output(BSP_QSPI_IO0_PIN);
  //nrf_gpio_cfg_output(15);
  gpio_init();
  nrf_delay_ms(500);

  ble_stack_init();
  gap_params_init();
  gatt_init();
  advertising_init();
  conn_params_init();
  peer_manager_init();
  advertising_start(erase_bonds);

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
      idle_state_handle();
      if (adc_rdy) {
        //nrf_delay_ms(10);
        //WIDE_IR_BUFFER[HR_INDEX] = heartrate5_getLed2val();
        //WIDE_RED_BUFFER[HR_INDEX] = heartrate5_getAled2val_led3val();

        if (BUFFER_INDEX <= WIDE_BUFFER_SIZE) {
          addLEDvalsToBuffers();
        }

        if (BUFFER_INDEX == WIDE_BUFFER_SIZE) {
          // DO ALGORITHM
          //for (int i = 0; i < BUFFER_SIZE; i++) {
          //  NRF_LOG_INFO("%d;%d", RED_BUFFER[i], IR_BUFFER[i]);
          //  NRF_LOG_FLUSH();
          //}

          // Data preprocessing

          IRcalculateDCmean();
          //for (int i = 0; i < WIDE_BUFFER_SIZE; i++) {
          //  NRF_LOG_INFO(";%d", IR_BUFFER[i]);
          //  NRF_LOG_FLUSH();
          //}
          IRremoveDCandInvert();
          //for (int i = 0; i < WIDE_BUFFER_SIZE; i++) {
          //  NRF_LOG_INFO(";%d", IR_processed[i]);
          //  NRF_LOG_FLUSH();
          //}
          IRmoveAverage();
          //for (int i = 0; i < WIDE_BUFFER_SIZE; i++) {
          //  NRF_LOG_INFO(";%d", IR_processed[i]);
          //  NRF_LOG_FLUSH();
          //}
          IRcalcThreshold();
          //NRF_LOG_INFO("Threshold %d", IRThreshold);

          uint32_t windowSize = 4;
          uint32_t peakWidth = 0;
          uint32_t numberOfPeaks = 0;
          uint32_t i, j = 0;

          // find peaks above min height

          while (i < WIDE_BUFFER_SIZE_CUT - 1) {
            if (IR_processed[i] > IRThreshold && IR_processed[i] > IR_processed[i - 1]) { // left edge
              peakWidth = 1;

              while (i + peakWidth < WIDE_BUFFER_SIZE_CUT && IR_processed[i] == IR_processed[i + peakWidth]) { // flat peaks
                peakWidth += 1;
              }

              if (IR_processed[i] > IR_processed[i + peakWidth] && numberOfPeaks < 15) {
                numberOfPeaks += 1;
                IR_valley_locations[numberOfPeaks] = i;
                i += peakWidth + 1;
              } else {
                i += peakWidth;
              }
            } else {
              i += 1;
            }
          }

          // Remove close peaks

          // sort indices descend

          uint32_t tempNumber = 0;

          for (i = 1; i < numberOfPeaks; i++) {
            tempNumber = IR_valley_locations[i];

            for (j = 1; j > 0 && IR_processed[tempNumber] > IR_processed[IR_valley_locations[j - 1]]; j--) {
              IR_valley_locations[j] = IR_valley_locations[j - 1];
            }

            IR_valley_locations[j] = tempNumber;
          }

          // remove peaks

          float numberOfOldPeaks, peakDistance = 0;

          for (i = -1; i < numberOfPeaks; i++) {
            numberOfOldPeaks = numberOfPeaks;
            numberOfPeaks = i + 1;

            for (j = i + 1; j < numberOfOldPeaks; j++) {
              peakDistance = IR_valley_locations[j] - (i == -1 ? -1 : IR_valley_locations[i]);

              if (peakDistance > windowSize || peakDistance < -windowSize) {
                numberOfPeaks += 1;
                IR_valley_locations[numberOfPeaks] = IR_valley_locations[j];
              }
            }
          }

          // resort indices

          for (i = 1; i < numberOfPeaks; i++) {
            tempNumber = IR_valley_locations[i];

            for (j = i; j > 0 && tempNumber < IR_valley_locations[j - 1]; j--) {
              IR_valley_locations[j] = IR_valley_locations[j - 1];
            }

            IR_valley_locations[j] = tempNumber;
          }

          //for (int i = 0; i < WIDE_BUFFER_SIZE; i++) {
          //  NRF_LOG_INFO(";%d", IR_processed[i]);
          //  NRF_LOG_FLUSH();
          //}

          //numberOfPeaks = min(numberOfPeaks, 15);

          for (k = 0; k < WIDE_BUFFER_SIZE; k++) {
            IR_processed[k] = IR_BUFFER[k];
            RED_processed[k] = RED_BUFFER[k];
          }

          //NRF_LOG_INFO("Number of Peaks %d", numberOfPeaks);

          float IRexactValleyLocationsCount, ratioAverage = 0;
          uint32_t iRatioCount = 0;
          float IRRedRatio[5];
          float SPOVal = 0;
          float IRMax, REDMax = -16777216; // x - ir, y - red

          for (k = 0; k < 5; k++) {
            IRRedRatio[k] = 0;
          }

          IRexactValleyLocationsCount = numberOfPeaks;

          for (k = 0; k < IRexactValleyLocationsCount; k++) {
            if (IR_valley_locations[k] > WIDE_BUFFER_SIZE) {
              SPOVal = -999;
            }
          }

          uint32_t REDMax_idx, IRMax_idx;

          for (k = 0; k < IRexactValleyLocationsCount - 1; k++) {
            REDMax = -16777216;
            IRMax = -16777216;
            if (IR_valley_locations[k + 1] - IR_valley_locations[k] > 3) {
              for (i = IR_valley_locations[k]; i < IR_valley_locations[k + 1]; i++) {
                if (IR_processed[i] > IRMax) {
                  IRMax = IR_processed[i];
                  IRMax_idx = i;
                }
                if (RED_processed[i] > REDMax) {
                  REDMax = RED_processed[i];
                  REDMax_idx = i;
                }
              }
              RED_AC = (RED_processed[IR_valley_locations[k + 1]] - RED_processed[IR_valley_locations[k]]) * (REDMax_idx - IR_valley_locations[k]); //red
              RED_AC = RED_processed[IR_valley_locations[k]] + RED_AC / (IR_valley_locations[k + 1] - IR_valley_locations[k]);
              RED_AC = RED_processed[REDMax_idx] - RED_AC;
              // subracting linear DC compoenents from raw
              IR_AC = (IR_processed[IR_valley_locations[k + 1]] - IR_processed[IR_valley_locations[k]]) * (IRMax_idx - IR_valley_locations[k]); // ir
              IR_AC = IR_processed[IR_valley_locations[k]] + IR_AC / (IR_valley_locations[k + 1] - IR_valley_locations[k]);
              IR_AC = IR_processed[REDMax_idx] - IR_AC; // subracting linear DC compoenents from raw

              NRF_LOG_INFO("%d;%d;%d;%d", IR_AC, RED_AC, IRMax, REDMax);
              NRF_LOG_FLUSH();

              if (IR_AC > 0 && RED_AC > 0) {
                IRRedRatio[iRatioCount] = RED_AC * IRMax * 100 / IR_AC * REDMax;
                iRatioCount++;
              }

              //n_nume = (RED_AC * IRMax) >> 7;           //prepare X100 to preserve floating value
              //n_denom = (IR_AC * REDMax) >> 7;
              //if (n_denom > 0 && iRatioCount < 5 && n_nume != 0) {
              //  an_ratio[iRatioCount] = (n_nume * 100) / n_denom; //formular is ( RED_AC *IRMax) / ( IR_AC *REDMax) ;

              //}
            }
          }

          uint32_t middleIdx;

          middleIdx = iRatioCount / 2;

          if (middleIdx > 1) {
            ratioAverage = (IRRedRatio[middleIdx - 1] + IRRedRatio[middleIdx]) / 2; // use median
          } else {
            ratioAverage = IRRedRatio[middleIdx];
          }
        }

        adc_rdy = false;
        HR_INDEX += 1;

        if (BUFFER_INDEX == WIDE_BUFFER_SIZE || BUFFER_INDEX > WIDE_BUFFER_SIZE) {
          resetBuffers();
        } else {
          BUFFER_INDEX += 1;
        }

        NRF_LOG_FLUSH();
      }
    }

    if (!detected_device) {
      NRF_LOG_INFO("No device was found.");
      NRF_LOG_FLUSH();
    }
  }
}

//while (true) {
//  NRF_LOG_INFO("LOL.");
//  NRF_LOG_FLUSH();
//  nrf_delay_ms(1000);
//}
//}
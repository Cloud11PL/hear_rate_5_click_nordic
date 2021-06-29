uint8_t hr3_set_settings(sw_reset_t sw_reset, diag_mode_t diag_mode,
    susp_count_t susp_counter, reg_read_t reg_read) {
  uint8_t reg = DIAGNOSIS;
  uint8_t temp[3] = {0};

  temp[2] |= (sw_reset << DIAG_SW_RST);
  temp[2] |= (diag_mode << DIAG_EN);
  temp[2] |= (susp_counter << DIAG_TM_CNT_RST);
  temp[2] |= (reg_read << DIAG_REG_READ);

  hr3_hal_write(&reg, temp, 3);

  return 0;
}

uint8_t hr3_set_led2_sample_start_end(uint16_t start, uint16_t end) {
  uint8_t reg_st = SMPL_LED2_ST;
  uint8_t reg_end = SMPL_LED2_END;
  uint8_t temp_st[3] = {0};
  uint8_t temp_end[3] = {0};

  if (start > 65535 || end > 65535)
    return -1;

  temp_st[1] = start >> 8;
  temp_st[2] = (uint8_t)start;

  hr3_hal_write(&reg_st, temp_st, 3);

  temp_end[1] = end >> 8;
  temp_end[2] = (uint8_t)end;

  hr3_hal_write(&reg_end, temp_end, 3);

  return 0;
}

uint8_t hr3_set_led1_start_end(uint16_t start, uint16_t end) {
  uint8_t reg_st = LED1_ST;
  uint8_t reg_end = LED1_END;
  uint8_t temp_st[3] = {0};
  uint8_t temp_end[3] = {0};

  if (start > 65535 || end > 65535)
    return -1;

  temp_st[1] = start >> 8;
  temp_st[2] = (uint8_t)start;

  hr3_hal_write(&reg_st, temp_st, 3);

  temp_end[1] = end >> 8;
  temp_end[2] = (uint8_t)end;

  hr3_hal_write(&reg_end, temp_end, 3);

  return 0;
}

uint8_t hr3_set_led3_sample_start_end(uint16_t start, uint16_t end) {
  uint8_t reg_st = SMPL_LED3_ST;
  uint8_t reg_end = SMPL_LED3_END;
  uint8_t temp_st[3] = {0};
  uint8_t temp_end[3] = {0};

  if (start > 65535 || end > 65535)
    return -1;

  temp_st[1] = start >> 8;
  temp_st[2] = (uint8_t)start;

  hr3_hal_write(&reg_st, temp_st, 3);

  temp_end[1] = end >> 8;
  temp_end[2] = (uint8_t)end;

  hr3_hal_write(&reg_end, temp_end, 3);

  return 0;
}

uint8_t hr3_set_led1_sample_start_end(uint16_t start, uint16_t end) {
  uint8_t reg_st = SMPL_LED1_ST;
  uint8_t reg_end = SMPL_LED1_END;
  uint8_t temp_st[3] = {0};
  uint8_t temp_end[3] = {0};

  if (start > 65535 || end > 65535)
    return -1;

  temp_st[1] = start >> 8;
  temp_st[2] = (uint8_t)start;

  hr3_hal_write(&reg_st, temp_st, 3);

  temp_end[1] = end >> 8;
  temp_end[2] = (uint8_t)end;

  hr3_hal_write(&reg_end, temp_end, 3);

  return 0;
}

uint8_t hr3_set_led2_start_end(uint16_t start, uint16_t end) {
  uint8_t reg_st = LED2_ST;
  uint8_t reg_end = LED2_END;
  uint8_t temp_st[3] = {0};
  uint8_t temp_end[3] = {0};

  if (start > 65535 || end > 65535)
    return -1;

  temp_st[1] = start >> 8;
  temp_st[2] = (uint8_t)start;

  hr3_hal_write(&reg_st, temp_st, 3);

  temp_end[1] = end >> 8;
  temp_end[2] = (uint8_t)end;

  hr3_hal_write(&reg_end, temp_end, 3);

  return 0;
}

uint8_t hr3_set_amb1_sample_start_end(uint16_t start, uint16_t end) {
  uint8_t reg_st = SMPL_AMB1_ST;
  uint8_t reg_end = SMPL_AMB1_END;
  uint8_t temp_st[3] = {0};
  uint8_t temp_end[3] = {0};

  if (start > 65535 || end > 65535)
    return -1;

  temp_st[1] = start >> 8;
  temp_st[2] = (uint8_t)start;

  hr3_hal_write(&reg_st, temp_st, 3);

  temp_end[1] = end >> 8;
  temp_end[2] = (uint8_t)end;

  hr3_hal_write(&reg_end, temp_end, 3);

  return 0;
}

uint8_t hr3_set_led2_convert_start_end(uint16_t start, uint16_t end) {
  uint8_t reg_st = LED2_CONV_ST;
  uint8_t reg_end = LED2_CONV_END;
  uint8_t temp_st[3] = {0};
  uint8_t temp_end[3] = {0};

  if (start > 65535 || end > 65535)
    return -1;

  temp_st[1] = start >> 8;
  temp_st[2] = (uint8_t)start;

  hr3_hal_write(&reg_st, temp_st, 3);

  temp_end[1] = end >> 8;
  temp_end[2] = (uint8_t)end;

  hr3_hal_write(&reg_end, temp_end, 3);

  return 0;
}

uint8_t hr3_set_led3_convert_start_end(uint16_t start, uint16_t end) {
  uint8_t reg_st = LED3_CONV_ST;
  uint8_t reg_end = LED3_CONV_END;
  uint8_t temp_st[3] = {0};
  uint8_t temp_end[3] = {0};

  if (start > 65535 || end > 65535)
    return -1;

  temp_st[1] = start >> 8;
  temp_st[2] = (uint8_t)start;

  hr3_hal_write(&reg_st, temp_st, 3);

  temp_end[1] = end >> 8;
  temp_end[2] = (uint8_t)end;

  hr3_hal_write(&reg_end, temp_end, 3);

  return 0;
}

uint8_t hr3_set_led1_convert_start_end(uint16_t start, uint16_t end) {
  uint8_t reg_st = LED1_CONV_ST;
  uint8_t reg_end = LED1_CONV_END;
  uint8_t temp_st[3] = {0};
  uint8_t temp_end[3] = {0};

  if (start > 65535 || end > 65535)
    return -1;

  temp_st[1] = start >> 8;
  temp_st[2] = (uint8_t)start;

  hr3_hal_write(&reg_st, temp_st, 3);

  temp_end[1] = end >> 8;
  temp_end[2] = (uint8_t)end;

  hr3_hal_write(&reg_end, temp_end, 3);

  return 0;
}

uint8_t hr3_set_amb1_convert_start_end(uint16_t start, uint16_t end) {
  uint8_t reg_st = AMB1_CONV_ST;
  uint8_t reg_end = AMB1_CONV_END;
  uint8_t temp_st[3] = {0};
  uint8_t temp_end[3] = {0};

  if (start > 65535 || end > 65535)
    return -1;

  temp_st[1] = start >> 8;
  temp_st[2] = (uint8_t)start;

  hr3_hal_write(&reg_st, temp_st, 3);

  temp_end[1] = end >> 8;
  temp_end[2] = (uint8_t)end;

  hr3_hal_write(&reg_end, temp_end, 3);

  return 0;
}

uint8_t hr3_set_adc_reset0_start_end(uint16_t start, uint16_t end) {
  uint8_t reg_st = ADC_RST_P0_ST;
  uint8_t reg_end = ADC_RST_P0_END;
  uint8_t temp_st[3] = {0};
  uint8_t temp_end[3] = {0};

  if (start > 65535 || end > 65535)
    return -1;

  temp_st[1] = start >> 8;
  temp_st[2] = (uint8_t)start;

  hr3_hal_write(&reg_st, temp_st, 3);

  temp_end[1] = end >> 8;
  temp_end[2] = (uint8_t)end;

  hr3_hal_write(&reg_end, temp_end, 3);

  return 0;
}

uint8_t hr3_set_adc_reset1_start_end(uint16_t start, uint16_t end) {
  uint8_t reg_st = ADC_RST_P1_ST;
  uint8_t reg_end = ADC_RST_P1_END;
  uint8_t temp_st[3] = {0};
  uint8_t temp_end[3] = {0};

  if (start > 65535 || end > 65535)
    return -1;

  temp_st[1] = start >> 8;
  temp_st[2] = (uint8_t)start;

  hr3_hal_write(&reg_st, temp_st, 3);

  temp_end[1] = end >> 8;
  temp_end[2] = (uint8_t)end;

  hr3_hal_write(&reg_end, temp_end, 3);

  return 0;
}

uint8_t hr3_set_adc_reset2_start_end(uint16_t start, uint16_t end) {
  uint8_t reg_st = ADC_RST_P2_ST;
  uint8_t reg_end = ADC_RST_P2_END;
  uint8_t temp_st[3] = {0};
  uint8_t temp_end[3] = {0};

  if (start > 65535 || end > 65535)
    return -1;

  temp_st[1] = start >> 8;
  temp_st[2] = (uint8_t)start;

  hr3_hal_write(&reg_st, temp_st, 3);

  temp_end[1] = end >> 8;
  temp_end[2] = (uint8_t)end;

  hr3_hal_write(&reg_end, temp_end, 3);

  return 0;
}

uint8_t hr3_set_adc_reset3_start_end(uint16_t start, uint16_t end) {
  uint8_t reg_st = ADC_RST_P3_ST;
  uint8_t reg_end = ADC_RST_P3_END;
  uint8_t temp_st[3] = {0};
  uint8_t temp_end[3] = {0};

  if (start > 65535 || end > 65535)
    return -1;

  temp_st[1] = start >> 8;
  temp_st[2] = (uint8_t)start;

  hr3_hal_write(&reg_st, temp_st, 3);

  temp_end[1] = end >> 8;
  temp_end[2] = (uint8_t)end;

  hr3_hal_write(&reg_end, temp_end, 3);

  return 0;
}

uint8_t hr3_set_prpct_count(uint16_t count) {
  uint8_t reg = PRPCT;
  uint8_t temp[3] = {0};

  if (count > 65535)
    return -1;

  temp[1] = count >> 8;
  temp[2] = (uint8_t)count;

  hr3_hal_write(&reg, temp, 3);

  return 0;
}

uint8_t hr3_set_timer_and_average_num(bool enable, uint8_t av_num) {
  uint8_t reg = TIM_NUMAV;
  uint8_t temp[3] = {0};

  if (av_num > 15 || av_num < 0)
    return -1;

  if (enable) {
    temp[1] |= (1 << TIMEREN);
    temp[2] |= (av_num << NUMAV);
    hr3_hal_write(&reg, temp, 3);
  } else {
    temp[2] |= (av_num << NUMAV);
    hr3_hal_write(&reg, temp, 3);
  }

  return 0;
}

uint8_t hr3_set_seperate_tia_gain(bool seperate, uint8_t cf_setting,
    uint8_t gain_setting) {
  uint8_t reg = TIA_GAINS2;
  uint8_t temp[3] = {0};

  if (cf_setting > 7 || gain_setting > 7)
    return -1;

  if (seperate) {
    temp[1] = TIA_ENSEPGAIN;
    temp[2] |= (cf_setting << TIA_CF_SEP);
    temp[2] |= (gain_setting << TIA_GAIN_SEP);
    hr3_hal_write(&reg, temp, 3);
  } else {
    temp[2] |= (cf_setting << TIA_CF_SEP);
    temp[2] |= (gain_setting << TIA_GAIN_SEP);
    hr3_hal_write(&reg, temp, 3);
  }

  return 0;
}

uint8_t hr3_set_tia_gain(bool replace, uint8_t cf_setting,
    uint8_t gain_setting) {
  uint8_t reg = TIA_GAINS1;
  uint8_t temp[3] = {0};

  if (cf_setting > 7 || gain_setting > 7)
    return -1;

  if (replace) {
    temp[1] = TIA_PROG_TG_EN;
    temp[2] |= (cf_setting << TIA_CF);
    temp[2] |= (gain_setting << TIA_GAIN);
    hr3_hal_write(&reg, temp, 3);
  } else {
    temp[1] = 0;
    temp[2] |= (cf_setting << TIA_CF_SEP);
    temp[2] |= (gain_setting << TIA_GAIN_SEP);
    hr3_hal_write(&reg, temp, 3);
  }

  return 0;
}

uint8_t hr3_replace_adc(bool replace) {
  uint8_t reg = TIA_GAINS1;
  uint8_t temp[3] = {0};

  hr3_read_enable();
  hr3_hal_read(&reg, temp, 3);
  hr3_read_disable();

  if (replace) {
    temp[1] |= TIA_PROG_TG_EN;
    hr3_hal_write(&reg, temp, 3);
  } else {
    temp[1] &= ~(TIA_PROG_TG_EN);
    hr3_hal_write(&reg, temp, 3);
  }

  return 0;
}

uint8_t hr3_set_led_currents(uint8_t led1_current, uint8_t led2_current,
    uint8_t led3_current) {
  uint8_t reg = LED_CONFIG;
  uint8_t temp[3] = {0};
  unsigned long currents = 0;

  if (led1_current > 63 ||
      led2_current > 63 ||
      led3_current > 63)
    return -1;

  currents |= (led3_current << 12);
  currents |= (led2_current << 6);
  currents |= led1_current;

  temp[2] |= currents;
  temp[1] |= currents >> 8;
  temp[0] |= currents >> 16;

  hr3_hal_write(&reg, temp, 3);

  return 0;
}
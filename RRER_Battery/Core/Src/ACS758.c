//
// Created by kenke on 2022/12/25.
//

#include "ACS758.h"
#include "c_values.h"

#define SENSE ((20.0f * TP1 / 5.0f / 1000.0f));

float read_v_s;
float vout_s;
float vdiff_s;
float curr_s;

float calc_amp(int value, float vref) {
  read_v_s = vref * (float)value / 4095.0f;
  vout_s = read_v_s * 10.1f / 6.8f;
  vdiff_s = vout_s - (TP1/2.0f);
  curr_s = vdiff_s / SENSE;

  return curr_s;
}

int amp_to_val(float amp, float vref) {
  float vdiff = amp * SENSE;
  float vout = vdiff + (TP1/2.0f);
  float read_v = vout / (10.1f / 6.8f);
  float value = read_v / vref * 4095.0f;
  return (int)value;
}
//
// Created by kenke on 2022/12/25.
//

#include "7520.h"
#include "c_values.h"


#define GAIN ((TP1/0.512f))

float calc_voltage(int val, float vref) {
  float read_v;
  float vout;
  float vin_diff;
  float vin;
  float V;
  read_v = vref * (float)val / (4095.0f);
  vout = read_v * 10.1f / 6.8f;
  vin_diff = (vout - (TP1 / 2.0f)) / GAIN;
  vin = vin_diff + TP2;
  V = vin / 3.3f * (3.3f + 7.5f + 200.0f);

  return V;
}

int voltage_to_val(float v, float vref) {
  float vin = v * 3.3f / (3.3f + 7.5f + 200.0f);
  float vdiff = vin - TP2;
  float vout = (vdiff * GAIN) + (TP1 / 2.0f);
  float readv = vout * 6.8f / 10.1f;
  float val = readv / vref * 4095.0f;
  return (int)val;
}

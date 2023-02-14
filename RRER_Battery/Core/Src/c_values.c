//
// Created by kenke on 2023/02/13.
//

#include "c_values.h"

float calc_v_ref(int vrefint_value) {
  return (4095.0f / (float)vrefint_value * 1.212f);
}
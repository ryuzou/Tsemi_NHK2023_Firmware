//
// Created by kenke on 2022/12/25.
//

#ifndef POWER_BOARD_ACS758_H
#define POWER_BOARD_ACS758_H

float calc_amp(int value, float vref);
int amp_to_val(float amp, float vref);

#endif //POWER_BOARD_ACS758_H

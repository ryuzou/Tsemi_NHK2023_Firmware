//
// Created by kenke on 2023/04/06.
//

#include "PID.h"

int calc_PID(PIDparamshandleTypeDef *handler, float target, float curr_val, float dt) {

  handler->e = target - curr_val;

  if (handler->is_windup == 0) {
    handler->e_int += (handler->e + handler->e_pre) * dt / 2.0f;
  }

  handler->e_diff = (handler->e_pre_pre - 4.0f*handler->e_pre + 3.0f*handler->e) / (2.0f * dt);

  handler->e_pre_pre = handler->e_pre;
  handler->e_pre = handler->e;

  return (int)(handler->Kp * handler->e + handler->Ki * handler->e_int + handler->Kd * handler->e_diff);
}